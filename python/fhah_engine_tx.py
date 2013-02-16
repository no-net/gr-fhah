#!/usr/bin/env python
#
# Copyright 2013 <+YOU OR YOUR COMPANY+>.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import numpy
import random
import struct
from gnuradio import gr
from gruel import pmt
#from gnuradio import uhd
#import gnuradio.extras  # brings in gr.block
import Queue
import math
import gnuradio.extras as gr_extras


BROADCAST_ADDR = 255

#block port definitions - inputs
OUTGOING_PKT_PORT = 0
INCOMING_PKT_PORT = 1
CTRL_PORT = 2

#block port definitions - outputs
TO_FRAMER_PORT = 0
CTRL_PORT = 1
APP_PORT = 2

#Time state machine
LOOKING_FOR_TIME = 0
HAVE_TIME = 0

#RX state machine
RX_INIT = 0
RX_SEARCH = 1
RX_FOUND = 2

#other
LOST_SYNC_THRESHOLD = 5

#Protocol Fields
HAS_DATA = numpy.array([1], dtype='uint8')
IS_RTS = numpy.array([31], dtype='uint8')
IS_CTS = numpy.array([32], dtype='uint8')
IS_BCN = numpy.array([159], dtype='uint8')


class fhah_engine_tx(gr.block):
    """
    docstring for block fhah_engine_tx
    """

    def __init__(self, hop_interval, pre_guard, post_guard, addr, link_speed,
                 freq_list):
        gr.block.__init__(self,
                          name="fhah_engine_tx",
                          in_sig=[numpy.complex64],
                          out_sig=None,
                          num_msg_inputs=3,
                          num_msg_outputs=3,
                          )

        self.mgr = pmt.pmt_mgr()
        for i in range(64):
            self.mgr.set(pmt.pmt_make_blob(10000))

        self.hop_interval = hop_interval
        self.post_guard = post_guard
        self.pre_guard = pre_guard
        self.link_bps = link_speed
        self.tx_freq_list = map(float, freq_list.split(','))
        self.tx_freq_list_length = len(self.tx_freq_list)
        self.hop_index = 0

        self.slot_duration = self.hop_interval - self.pre_guard - self.post_guard
        self.bytes_per_slot = int(self.slot_duration * self.link_bps / 8)

        self.queue = Queue.Queue()  # queue for msg destined for ARQ path
        self.tx_queue = Queue.Queue()

        self.last_rx_time = 0
        self.last_rx_rate = 0
        self.samples_since_last_rx_time = 0

        self.next_interval_start = 0
        self.next_transmit_start = 0

        self.know_time = False
        self.found_time = False
        self.found_rate = False
        self.set_tag_propagation_policy(gr_extras.TPP_DONT)

        self.has_old_msg = False
        self.overhead = 20
        self.pad_data = numpy.zeros((1, 5), dtype='uint8')[0]
        self.tx_slots_passed = 0

        self.rx_state = RX_INIT
        self.plkt_received = False
        self.rx_delay = 0.006
        self.rx_hop_index = 0
        self.consecutive_miss = 0

        self.max_hops_to_beacon = 100
        self.hops_to_beacon = 100
        self.diff_last_beacon = 0
        self.beacon_msg = numpy.ones((1, 5), dtype='uint8')[0]

        self.synced = False

        self.time_tune_start = 0

        self.rts_msg = numpy.array([0, 0, 1, 0, 0], dtype='uint8')[0]
        self.got_cts = False
        self.waiting_for_cts = False
        self.hops_to_retx = 0
        self.max_hops_to_retx = 10
        self.retx_no = 1
        self.got_rts = False

        self.own_adr = numpy.array([addr], dtype='uint8')
        self.bcst_adr = numpy.array([0], dtype='uint8')
        if self.own_adr == numpy.array([1], dtype='uint8'):
            self.dst_adr = numpy.array([2], dtype='uint8')
        else:
            self.dst_adr = numpy.array([1], dtype='uint8')

    def _time_to_msg(self, time_obj):
        """
        Return the message (bytewise numpy.array) interpration of the time_obj.
        """
        string = struct.pack('d', time_obj)

        return numpy.fromstring(string, dtype=numpy.uint8)

    def _msg_to_time(self, time_msg):
        """
        Return time_obj as float.
        """
        string = time_msg.tostring()

        return struct.unpack('d', string)

    def hop(self):
        """
        Hop to next slot in frequency list.
        """
        usrps = ['usrp_sink', 'usrp_source']

        for usrp in usrps:
            self.post_msg(CTRL_PORT,
                          pmt.pmt_string_to_symbol(usrp + '.set_command_time'),
                          pmt.from_python((('#!\nfrom gnuradio import uhd\narg = uhd.time_spec_t(' + repr(self.interval_start) + ')', ), {})),
                          pmt.pmt_string_to_symbol('fhss'))
            self.post_msg(CTRL_PORT,
                          pmt.pmt_string_to_symbol(usrp + '.set_center_freq'),
                          pmt.from_python(((self.tx_freq_list[self.hop_index], ), {})),
                          pmt.pmt_string_to_symbol('fhss'))
            self.post_msg(CTRL_PORT,
                          pmt.pmt_string_to_symbol(usrp + '.clear_command_time'),
                          pmt.from_python(((0, ), {})),
                          pmt.pmt_string_to_symbol('fhss'))

        self.hop_index = (self.hop_index + 1) % self.tx_freq_list_length

    def get_cts(self):
        """
        Send RTS after random amount of time and wait for CTS.
        """
        # Create RTS msg, transmit. Send RTS within first half of slot
        self.waiting_for_cts = True
        max_delay_in_slot = self.slot_duration / 2

        self.tx_signaling(max_delay_in_slot, IS_RTS, self.dst_adr)

    def send_cts(self):
        """
        Send CTS after RTS was received.
        """
        # TODO: Send CTS within the same slot!
        # Create CTS msg, send without delay.
        max_delay_in_slot = 0.001

        self.tx_signaling(max_delay_in_slot, IS_CTS, self.dst_adr)

    def send_beacon(self):
        """
        Send at least one beacon in max_hops_to_beacon.
        """
        # Randomly set no of hops to next beacon (add diff from last beacon, so
        # that only one beacon is sent in max_hops_to_beacon!)
        beacon_slot = random.randint(1, self.max_hops_to_beacon)
        self.hops_to_beacon = self.diff_last_beacon + beacon_slot
        self.diff_last_beacon = self.max_hops_to_beacon - beacon_slot

        # Distribute time to send beacon randomly in slot
        max_delay_in_slot = self.slot_duration - 0.001

        self.tx_signaling(max_delay_in_slot, IS_BCN, self.bcst_adr)

    def tx_signaling(self, max_delay_in_slot, msg_type, dst_adr):
        """
        Send signaling/control frames (no data).
        """
        # Send after random amount of time in this bin/slot/hop
        delay = random.uniform(0, max_delay_in_slot)
        ant_start = self.antenna_start + delay

        time_msg = self._time_to_msg(self.interval_start)
        next_hop_index = numpy.array([self.hop_index], dtype='uint8')

        time_object = int(math.floor(ant_start)), (ant_start % 1)

        # Create msg and add to tx_queue before calling transmit
        data = numpy.concatenate([msg_type,
                                  self.own_adr,
                                  dst_adr,
                                  time_msg,
                                  next_hop_index])
        more_frames = 0
        tx_object = time_object, data, more_frames
        self.post_msg(TO_FRAMER_PORT,
                      pmt.pmt_string_to_symbol('full'),
                      pmt.from_python(tx_object),
                      pmt.pmt_string_to_symbol('tdma'))

        #print msg_type, " - ", self.interval_start % 1, " - TO: ", dst_adr, " - at: ", repr(self.interval_start), " - time now: ", repr(self.time_update)

    def tx_data(self):
        """
        Put messages from input into tx_queue.
        """
        #get all of the packets we want to send
        total_byte_count = 0
        frame_count = 0

        #put residue from previous execution
        if self.has_old_msg:
            length = len(pmt.pmt_blob_data(self.old_msg.value)) + self.overhead
            total_byte_count += length
            self.tx_queue.put(self.old_msg)
            frame_count += 1
            self.has_old_msg = False
            print 'old msg'

        # TODO: Check for length IF OLD_MSG and don't get new msgs if old_msg!
        # TODO: Get dst addr from MSG

        #fill outgoing queue until empty or maximum bytes queued for slot
        while(not self.queue.empty()):
            msg = self.queue.get()
            length = len(pmt.pmt_blob_data(msg.value)) + self.overhead
            total_byte_count += length
            if total_byte_count >= self.bytes_per_slot:
                self.has_old_msg = True
                self.old_msg = msg
                self.got_cts = True
                print 'residue'
                continue
            else:
                self.has_old_msg = False
                self.tx_queue.put(msg)
                self.got_cts = False
                frame_count += 1

        if frame_count > 0:

            time_object = int(math.floor(self.antenna_start)), (self.antenna_start % 1)

            #print frame_count,self.queue.qsize(), self.tx_queue.qsize()
            #send first frame w tuple for tx_time and number of frames to put
            #in slot
            blob = self.mgr.acquire(True)  # block
            more_frames = frame_count - 1
            msg = self.tx_queue.get()
            data = numpy.concatenate([HAS_DATA,
                                      self.own_adr,
                                      self.dst_adr,
                                      pmt.pmt_blob_data(msg.value)])
            #print "DATA-SEND: %s" % data
            tx_object = time_object, data, more_frames
            self.post_msg(TO_FRAMER_PORT,
                          pmt.pmt_string_to_symbol('full'),
                          pmt.from_python(tx_object),
                          pmt.pmt_string_to_symbol('tdma'))
            print " MSG POSTED!"
            print " - Ant start:", repr(self.antenna_start), " - TO: ", self.dst_adr, " - at: ", repr(self.interval_start), " - time now: ", repr(self.time_update)
            frame_count -= 1

            #old_data = []
            #print 'frame count: ',frame_count
            #send remining frames, blob only
            while(frame_count > 0):
                msg = self.tx_queue.get()
                data = numpy.concatenate([HAS_DATA,
                                          self.own_adr,
                                          self.dst_adr,
                                          pmt.pmt_blob_data(msg.value)])
                blob = self.mgr.acquire(True)  # block
                pmt.pmt_blob_resize(blob, len(data))
                pmt.pmt_blob_rw_data(blob)[:] = data
                self.post_msg(TO_FRAMER_PORT,
                              pmt.pmt_string_to_symbol('d_only'),
                              blob,
                              pmt.pmt_string_to_symbol('tdma'))
                frame_count -= 1

            #print total_byte_count

    def work(self, input_items, output_items):

        if self.rx_state == RX_INIT:
            for usrp in ['usrp_source', 'usrp_sink']:
                self.post_msg(CTRL_PORT,
                              pmt.pmt_string_to_symbol(usrp + '.set_center_freq'),
                              pmt.from_python(((self.tx_freq_list[self.rx_hop_index], ), {})),
                              pmt.pmt_string_to_symbol('fhss'))

            self.rx_state = RX_SEARCH

        #check for msg inputs when work function is called
        if self.check_msg_queue():
            try:
                msg = self.pop_msg_queue()
            except:
                return -1

            if msg.offset == OUTGOING_PKT_PORT:
                self.queue.put(msg)  # if outgoing, put in queue for processing

            elif msg.offset == INCOMING_PKT_PORT:
                pkt = pmt.pmt_blob_data(msg.value)
                print "MSG from ", pkt[1], " - to ", pkt[2], " type: ", pkt[0]
                if pkt[1] != self.own_adr and pkt[2] in [self.own_adr, self.bcst_adr]:
                    if pkt[0] == HAS_DATA:
                        print "DATA received"
                        blob = self.mgr.acquire(True)  # block
                        pmt.pmt_blob_resize(blob, len(pkt) - 1)
                        pmt.pmt_blob_rw_data(blob)[:] = pkt[1:]
                        self.post_msg(APP_PORT,
                                      pmt.pmt_string_to_symbol('rx'),
                                      blob,
                                      pmt.pmt_string_to_symbol('fhss'))
                    elif pkt[0] == IS_RTS:
                        print "RTS received"
                        # TODO: Check own queues, if transmission is running
                        self.got_rts = True
                    elif pkt[0] == IS_CTS:
                        print "CTS received"
                        if pkt[1] == self.dst_adr:
                            self.got_cts = True
                            self.waiting_for_cts = False
                            self.hops_to_retx = 0
                            self.retx_no = 1
                    elif pkt[0] == IS_BCN:
                        # Sync to beacon if pkt is from node with higher prio!
                        if (pkt[1] < self.own_adr):  # and not self.synced:
                            self.time_tune_start = self._msg_to_time(pkt[3:11])[0] + (2 * self.hop_interval)
                            self.interval_start = self.time_tune_start

                            # Send tune command before the USRP has to tune
                            self.time_tune_start -= 10 * self.post_guard

                            self.hop_index = (pkt[11] + 1) % self.tx_freq_list_length
                            if self.hops_to_beacon != 0:
                                self.hops_to_beacon -= 1

                            if not self.synced:
                                print "SYNCED!"
                            self.synced = True
                            # TODO: check if time is in future
                            #print "SYNCED to: ", int(math.floor(self.time_tune_start)), " - ", self.time_tune_start % 1, "--- TIME NOW: ", int(math.floor(self.time_update)), " - ", self.time_update % 1

                        #elif pkt[1] > self.own_adr:
                        #    print "Not syncing to higher device address!"
                    else:
                        print "ERROR: Wrong Type!"
                #else:
                #    print "Not addressed to this station - adr to: ", pkt[2]

            else:
                pass

        nread = self.nitems_read(0)  # number of items read on port 0
        ninput_items = len(input_items[0])

        if not self.know_time:
            #process streaming samples and tags here

            #read all tags associated with port 0 for items
            tags = self.get_tags_in_range(0, nread, nread + ninput_items)

            #find all of our tags, making the adjustments to our timing
            for tag in tags:
                key_string = pmt.pmt_symbol_to_string(tag.key)
                if key_string == "rx_time":
                    self.samples_since_last_rx_time = 0
                    self.current_integer, self.current_fractional = pmt.to_python(tag.value)
                    self.time_update = self.current_integer + self.current_fractional
                    self.found_time = True
                elif key_string == "rx_rate":
                    self.rate = pmt.to_python(tag.value)
                    self.sample_period = 1 / self.rate
                    self.found_rate = True

            if self.found_time and self.found_rate:
                self.know_time = True

        #get/update current time
        self.time_update += (self.sample_period * ninput_items)

        # Set first tuning time 20 sec in future (hope that we receive beacon
        # pkg within this time for sync -> assume that we're the only node if not)
        if self.time_tune_start == 0:
            if self.own_adr == 1:  # TODO: ONLY FOR DEBUGGING
                self.interval_start = self.time_update + 2
            else:
                self.interval_start = self.time_update + 20
            self.time_tune_start = self.interval_start - self.post_guard

        #determine if it's time for us to start tx'ing, start process
        #10 * self.post_guard before our slot actually begins (deal with latency)
        if self.time_update > self.time_tune_start:
            self.antenna_start = self.interval_start + self.pre_guard

            self.hop()

            if self.got_cts:
                self.got_cts = False
                self.tx_data()
                self.hops_to_beacon += 1

            elif self.got_rts:
                self.send_cts()
                self.got_rts = False
                self.hops_to_beacon += 1

            elif self.hops_to_beacon == 0:
                self.send_beacon()

            elif not self.waiting_for_cts:
                if not self.queue.empty():
                    self.get_cts()
                    self.waiting_for_cts = True
                    self.hops_to_retx = self.retx_no * self.max_hops_to_retx
                else:
                    pass

            else:
                # Waiting for CTS - Set random time to retransmit RTS!
                if self.hops_to_retx == 0:
                    self.get_cts()
                    self.retx_no += 1
                    self.hops_to_retx = random.randint((self.retx_no - 1) * self.max_hops_to_retx, self.retx_no * self.max_hops_to_retx)
                self.hops_to_retx -= 1

            self.interval_start += self.hop_interval
            self.time_tune_start = self.interval_start - (10 * self.post_guard)

            #print "Next Hop: ", int(math.floor(self.interval_start)), " - ", self.interval_start % 1, " ----- INDEX: ", self.hop_index

            self.hops_to_beacon -= 1

        return ninput_items
