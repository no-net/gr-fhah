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
from gnuradio import gr
from gruel import pmt
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
HAS_DATA = numpy.ones((1, 1), dtype='uint8')[0]
HAS_NO_DATA = numpy.zeros((1, 1), dtype='uint8')[0]

RTS_IND = numpy.array([1, 0], dtype='uint8')
CTS_IND = numpy.array([0, 1], dtype='uint8')

BCN_IND = numpy.array([0, 0], dtype='uint8')


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
        #print self.pad_data
        self.tx_slots_passed = 0

        self.rx_state = RX_SEARCH
        self.plkt_received = False
        self.tune_lead = 0.003
        self.rx_hop_index = 0
        self.consecutive_miss = 0

        self.max_hops_to_beacon = 100
        self.hops_to_beacon = 100
        self.diff_last_beacon = 0
        self.beacon_msg = numpy.ones((1, 5), dtype='uint8')[0]

        self.rts_msg = numpy.array([0, 0, 1, 0, 0], dtype='uint8')[0]
        self.got_cts = False

        self.own_adr = numpy.array([0, 0, 1], dtype='uint8')

    def hop(self):
        """
        Hop to next slot in frequency list.
        """
        #send_sob
        #self.post_msg(TO_FRAMER_PORT,
        #              pmt.pmt_string_to_symbol('tx_sob'),
        #              pmt.PMT_T, pmt.pmt_string_to_symbol('tx_sob')
        #              )

        #a = pmt.from_python(((self.tx_freq_list[self.hop_index], ), {}))
        self.post_msg(CTRL_PORT,
                      pmt.pmt_string_to_symbol('usrp_sink.set_command_time'),
                      pmt.from_python(((self.interval_start, ), {})),
                      pmt.pmt_string_to_symbol('fhss'))
        #print "NEXT USRP CMD: %s" % self.interval_start
        self.post_msg(CTRL_PORT,
                      pmt.pmt_string_to_symbol('usrp_sink.set_center_freq'),
                      pmt.from_python(((self.tx_freq_list[self.hop_index], ), {})),  # (( , ), {})),
                      pmt.pmt_string_to_symbol('fhss'))
        #print "----> NEXT CMD: %s" % self.tx_freq_list[self.hop_index]
        self.post_msg(CTRL_PORT,
                      pmt.pmt_string_to_symbol('usrp_sink.clear_comman.d_time'),
                      pmt.from_python(((0, ),
                      {})),
                      pmt.pmt_string_to_symbol('fhss'))
        self.hop_index = (self.hop_index + 1) % self.tx_freq_list_length
        #print self.hop_index,self.interval_start

    def get_cts(self):
        """
        Send RTS after random amount of time and wait for CTS.
        """
        # Create RTS msg and call transmit, wait for CTS
        max_delay_in_slot = self.slot_duration / 2

        # TODO: RTS msg muss Quelle und Ziel enthalten und als CTS umgekehrt
        # fordern! -> rts_indicator + src + dest
        self.tx_signaling(max_delay_in_slot, self.rts_msg)

    def send_beacon(self):
        """
        Send at least one beacon in max_hops_to_beacon.
        """
        # Randomly set no of hops to next beacon (add diff from last beacon, so
        # that send only one beacon in mac_hops_to_beacon!)
        beacon_slot = random.randint(1, self.max_hops_to_beacon)
        self.hops_to_beacon = self.diff_last_beacon + beacon_slot
        self.diff_last_beacon = self.max_hops_to_beacon - beacon_slot
        #print self.diff_last_beacon

        max_delay_in_slot = self.slot_duration - 0.001

        dst_adr = numpy.array([0, 0, 0], dtype='uint8')  # Broadcast Address

        self.tx_signaling(max_delay_in_slot, BCN_IND, dst_adr, self.beacon_msg)

    def tx_signaling(self, max_delay_in_slot, msg_type, dst_adr, msg):
        """
        Send signaling/control frames (no data).
        """
        # Send after random amount of time in this bin/slot/hop
        delay = random.uniform(0, max_delay_in_slot)
        ant_start = self.antenna_start + delay

        time_object = int(math.floor(ant_start)), (ant_start % 1)

        # Create msg and add to tx_queue before calling transmit
        data = numpy.concatenate([HAS_NO_DATA,
                                  msg_type,
                                  self.own_adr,
                                  dst_adr,
                                  msg])
        more_frames = 0
        tx_object = time_object, data, more_frames
        self.post_msg(TO_FRAMER_PORT,
                      pmt.pmt_string_to_symbol('full'),
                      pmt.from_python(tx_object),
                      pmt.pmt_string_to_symbol('tdma'))

        print "Beacon sent at: ", time_object

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

        #fill outgoing queue until empty or maximum bytes queued for slot
        while(not self.queue.empty()):
            msg = self.queue.get()
            length = len(pmt.pmt_blob_data(msg.value)) + self.overhead
            total_byte_count += length
            if total_byte_count >= self.bytes_per_slot:
                self.has_old_msg = True
                self.old_msg = msg
                print 'residue'
                continue
            else:
                self.has_old_msg = False
                self.tx_queue.put(msg)
                self.got_cts = False
                frame_count += 1

        if frame_count > 0:

            time_object = int(math.floor(self.antenna_start)), (self.antenna_start % 1)
            print time_object

            #print frame_count,self.queue.qsize(), self.tx_queue.qsize()
            #send first frame w tuple for tx_time and number of frames to put
            #in slot
            blob = self.mgr.acquire(True)  # block
            more_frames = frame_count - 1
            msg = self.tx_queue.get()
            data = numpy.concatenate([HAS_DATA, pmt.pmt_blob_data(msg.value)])
            #print "DATA-SEND: %s" % data
            tx_object = time_object, data, more_frames
            self.post_msg(TO_FRAMER_PORT,
                          pmt.pmt_string_to_symbol('full'),
                          pmt.from_python(tx_object),
                          pmt.pmt_string_to_symbol('tdma'))
            frame_count -= 1

            #old_data = []
            #print 'frame count: ',frame_count
            #send remining frames, blob only
            while(frame_count > 0):
                msg = self.tx_queue.get()
                data = numpy.concatenate([HAS_DATA,
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
            self.post_msg(CTRL_PORT,
                          pmt.pmt_string_to_symbol('usrp_source.set_center_freq'),
                          pmt.from_python((self.rx_freq_list[self.rx_hop_index], ), {}),
                          pmt.pmt_string_to_symbol('fhss'))
            print 'Initialized to channel %s.  Searching...' % self.rx_hop_index
            self.rx_state == RX_SEARCH

        #check for msg inputs when work function is called
        if self.check_msg_queue():
            try:
                msg = self.pop_msg_queue()
            except:
                return -1

            if msg.offset == OUTGOING_PKT_PORT:
                self.queue.put(msg)  # if outgoing, put in queue for processing

            elif msg.offset == INCOMING_PKT_PORT:
                # CHECK FOR CTS
                # --> Reset timing
                pkt = pmt.pmt_blob_data(msg.value)
                print pkt
                print pkt[0]
                if not pkt[0]:
                    print "--TX: Signaling Packet received."
                    # TODO: Wenn hier noch DATA bgefangen/weitergeleitet wird,
                    # dann ist der empfaener hier schon vollstaendig integriert
                    # und der block wird zum transceiver!!!
                    # TODO: Synchronisation auf BCN-pakete noch implementieren!
                    if pkt[1:3] == RTS_IND:
                        print "----RTS received"
                    if pkt[1:3] == CTS_IND:
                        print "----CTS received"
                    if pkt[1:3] == BCN_IND:
                        print "----BCN received"
                # TODO: CHECKE OB DATEN TATSAECHLICH BINAER VOM FRAMER
                # INTERPRETIERT WERDEN!!! ->speichern der nachbarn etc.

            else:
                pass  # CONTROL port

        #in0 = input_items[0]
        nread = self.nitems_read(0)  # number of items read on port 0
        ninput_items = len(input_items[0])

        if not self.know_time:  # TODO: Move this to next IF!
            #process streaming samples and tags here

            #read all tags associated with port 0 for items in this work function
            tags = self.get_tags_in_range(0, nread, nread + ninput_items)

            #lets find all of our tags, making the appropriate adjustments to our
            #timing
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

        #determine first transmit slot when we learn the time
        if not self.know_time:
            if self.found_time and self.found_rate:
                self.know_time = True
                #TODO: this stuff is left over from tdma.py, see if we can re-use this somehow
                #self.frame_period = self.slot_interval * self.num_slots
                #my_fraction_frame = ( self.initial_slot * 1.0 ) / ( self.num_slots)
                #frame_count = math.floor(self.time_update / self.frame_period)
                #current_slot_interval = ( self.time_update % self.frame_period ) / self.frame_period
                #self.time_transmit_start = (frame_count + 2) * self.frame_period + ( my_fraction_frame * self.frame_period ) - self.lead_limit
                self.time_transmit_start = self.time_update + (self.post_guard * 10.0)  # TODO: ser pre_guard time!
                self.interval_start = self.time_transmit_start + self.post_guard

        #get current time
        self.time_update += (self.sample_period * ninput_items)

        #determine if it's time for us to start tx'ing, start process self.lead_limit seconds
        #before our slot actually begins (i.e. deal with latency)
        if self.time_update > self.time_transmit_start:
            self.antenna_start = self.interval_start + self.pre_guard
            self.hop()
            if self.hops_to_beacon == 0:
                self.send_beacon()
            #   self.send_beacon() -> wie get_cts (mit sensing)
            #       --> setzt next_beacon_slot eins/zufaellig hoeher wenn Kanal
            #       belegt!
        elif not self.got_cts:  # TODO: NUR DEBUG -> NEGIEREN!!!
                self.tx_data()   # do more than this?
            else:
                self.get_cts()
            self.interval_start += self.hop_interval
            self.time_transmit_start = self.interval_start - self.post_guard
            self.hops_to_beacon -= 1

        return ninput_items
