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
from gnuradio import gr
from gruel import pmt
import gnuradio.extras as gr_extras


BROADCAST_ADDR = 255

#block port definitions - inputs
INCOMING_PKT_PORT = 0
CTRL_PORT = 1
#block port definitions - outputs
APP_PORT = 0
CTRL_PORT = 1

#Time state machine
LOOKING_FOR_TIME = 0
HAVE_TIME = 0

#RX state machine
RX_INIT = 0
RX_SEARCH = 1
RX_FOUND = 2

#Pkt filter field
HAS_DATA = 1
HAS_NO_DATA = 0

#other
LOST_SYNC_THRESHOLD = 15


class fhah_engine_rx(gr.block):
    """
    docstring for block fhah_engine_rx
    """
    def __init__(self, hop_interval, pre_guard, post_guard, addr, link_speed,
                 freq_list):
        gr.gr.block.__init__(self,
                             name="fhah_engine_rx",
                             in_sig=[numpy.complex64],
                             out_sig=None,
                             num_msg_inputs=2,
                             num_msg_outputs=2)

        self.mgr = pmt.pmt_mgr()
        for i in range(64):
            self.mgr.set(pmt.pmt_make_blob(10000))

        self.hop_interval = hop_interval
        self.post_guard = post_guard
        self.pre_guard = pre_guard
        self.link_bps = link_speed
        self.rx_freq_list = map(float, freq_list.split(','))
        self.rx_freq_list_length = len(self.rx_freq_list)

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
        self.pad_data = numpy.zeros((1, 40), dtype='uint8')[0]
        self.tx_slots_passed = 0

        self.rx_state = RX_INIT
        self.plkt_received = False
        self.tune_lead = 0.010
        self.rx_hop_index = 0
        self.consecutive_miss = 0

    def work(self, input_items, output_items):

        if self.rx_state == RX_INIT:
            self.rx_hop_index = 0
            self.rx_state = RX_SEARCH
            self.post_msg(CTRL_PORT,
                          pmt.pmt_string_to_symbol('usrp_source.set_center_freq'),
                          pmt.from_python((self.rx_freq_list[self.rx_hop_index], ), {}),
                          pmt.pmt_string_to_symbol('fhss'))
            self.rx_hop_index = (self.rx_hop_index + 1) % self.rx_freq_list_length
            print 'Initialized to channel 0.  Searching...'

        #check for msg inputs when work function is called
        if self.check_msg_queue():
            try:
                msg = self.pop_msg_queue()
            except:
                return -1

            if msg.offset == INCOMING_PKT_PORT:
                pkt = pmt.pmt_blob_data(msg.value)
                if pkt[0]:
                    blob = self.mgr.acquire(True)  # block
                    pmt.pmt_blob_resize(blob, len(pkt) - 1)
                    pmt.pmt_blob_rw_data(blob)[:] = pkt[1:]
                    self.post_msg(APP_PORT,
                                  pmt.pmt_string_to_symbol('rx'),
                                  blob,
                                  pmt.pmt_string_to_symbol('fhss'))
                if self.know_time:
                    if self.rx_state == RX_SEARCH:
                        self.rx_state = RX_FOUND
                        self.pkt_received = True
                        self.next_tune_time = self.time_update + self.hop_interval - self.tune_lead
                        self.start_hop = self.next_tune_time - self.lead_limit
                        print 'Received packet.  Locked.  Hopping initialized.'
                    else:
                        self.pkt_received = True
                        #print 'pkt_rcved_2',self.time_update,self.start_hop,self.next_tune_time

            else:
                pass  # CONTROL port

        #process streaming samples and tags here
        #in0 = input_items[0]
        nread = self.nitems_read(0)  # number of items read on port 0
        ninput_items = len(input_items[0])

        #read all tags associated with port 0 for items in this work function
        tags = self.get_tags_in_range(0, nread, nread + ninput_items)

        #lets find all of our tags, making the appropriate adjustments to our timing
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

        #get current time
        self.time_update += (self.sample_period * ninput_items)

        if self.rx_state == RX_FOUND:
            if self.time_update > self.start_hop:
                #print 'set: ', self.rx_freq_list[self.rx_hop_index], self.time_update, self.next_tune_time
                self.post_msg(CTRL_PORT,
                              pmt.pmt_string_to_symbol('usrp_source.set_command_time'),
                              pmt.from_python((self.next_tune_time, ), {}),
                              pmt.pmt_string_to_symbol('fhss'))
                self.post_msg(CTRL_PORT,
                              pmt.pmt_string_to_symbol('usrp_source.set_center_freq'),
                              pmt.from_python((self.rx_freq_list[self.rx_hop_index], ), {}),
                              pmt.pmt_string_to_symbol('fhss'))
                self.post_msg(CTRL_PORT,
                              pmt.pmt_string_to_symbol('usrp_source.clear_command_time'),
                              pmt.from_python((0, ), {}),
                              pmt.pmt_string_to_symbol('fhss'))
                self.rx_hop_index = (self.rx_hop_index + 1) % self.rx_freq_list_length
                self.start_hop += self.hop_interval
                self.next_tune_time += self.hop_interval
                #self.next_rx_interval += self.hop_interval - self.tune_lead
                if self.pkt_received:
                    self.consecutive_miss = 0
                else:
                    self.consecutive_miss += 1

                if self.consecutive_miss > LOST_SYNC_THRESHOLD:
                    self.consecutive_miss = 0
                    self.rx_state = RX_INIT
                    print 'Lost Sync: Re-Initializing'

                self.pkt_received = False

        return ninput_items
