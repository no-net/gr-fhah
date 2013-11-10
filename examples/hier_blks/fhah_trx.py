#!/usr/bin/env python
##################################################
# Gnuradio Python Flow Graph
# Title: FHAH TRANSCEIVER
# Author: NO
# Generated: Thu Jun 13 15:22:15 2013
##################################################

from gnuradio import digital
from gnuradio import gr
from gnuradio import uhd
from gnuradio.gr import firdes
import fhah
import gnuradio.extras as gr_extras
#import precog
import time

class fhah_trx(gr.hier_block2):

    def __init__(self, lead_limit=0.001, tx_gain=0, samp_per_sym=4, link_speed=200, pre_guard=0.003, rate=1e6, hop_interval=0.02, rx_gain=15, ampl=0.7, freq=425e6, args="", freq_list="4251e5,4261e5,4271e5", post_guard=0.001, dev_addr=1):
        gr.hier_block2.__init__(
            self, "FHAH TRANSCEIVER",
            gr.io_signature(1, 1, gr.sizeof_char*1),
            gr.io_signature(1, 1, gr.sizeof_char*1),
        )

        ##################################################
        # Parameters
        ##################################################
        self.lead_limit = lead_limit
        self.tx_gain = tx_gain
        self.samp_per_sym = samp_per_sym
        self.link_speed = link_speed
        self.pre_guard = pre_guard
        self.rate = rate
        self.hop_interval = hop_interval
        self.rx_gain = rx_gain
        self.ampl = ampl
        self.freq = freq
        self.args = args
        self.freq_list = freq_list
        self.post_guard = post_guard
        self.dev_addr = dev_addr

        ##################################################
        # Variables
        ##################################################
        self.samp_rate_0 = samp_rate_0 = rate
        self.samp_rate = samp_rate = rate

        ##################################################
        # Blocks
        ##################################################
        self.uhd_source = uhd.usrp_source(
            device_addr=args,
            stream_args=uhd.stream_args(
                cpu_format="fc32",
                channels=range(1),
            ),
        )
        self.uhd_source.set_clock_source("gpsdo", 0)
        self.uhd_source.set_time_source("gpsdo", 0)
        self.uhd_source.set_samp_rate(samp_rate)
        #self.uhd_source.set_auto_dc_offset(False)
        self.uhd_source.set_center_freq(freq, 0)
        self.uhd_source.set_gain(rx_gain, 0)
        self.uhd_source.set_antenna("RX2", 0)
        self.uhd_sink = uhd.usrp_sink(
            device_addr=args,
            stream_args=uhd.stream_args(
                cpu_format="fc32",
                channels=range(1),
            ),
        )
        self.uhd_sink.set_clock_source("gpsdo", 0)
        self.uhd_sink.set_time_source("gpsdo", 0)
        self.uhd_sink.set_samp_rate(samp_rate)
        self.uhd_sink.set_center_freq(freq, 0)
        self.uhd_sink.set_gain(tx_gain, 0)
        self.uhd_sink.set_antenna("TX/RX", 0)
        self.precog_packet_framer_0 = fhah.packet_framer(
            samples_per_symbol=samp_per_sym,
            bits_per_symbol=1,
            access_code="",
        )
        self.precog_packet_deframer_0 = fhah.packet_deframer(
            access_code="",
            threshold=2,
        )
        self.gr_multiply_const_vxx_0 = gr.multiply_const_vcc((ampl, ))
        self.gmsk_mod = digital.gmsk_mod(
            samples_per_symbol=samp_per_sym,
            bt=0.35,
            verbose=False,
            log=False,
        )
        self.gmsk_demod = digital.gmsk_demod(
            samples_per_symbol=samp_per_sym,
            gain_mu=0,
            mu=0.5,
            omega_relative_limit=0.005,
            freq_error=0.0,
            verbose=False,
            log=False,
        )
        self.fhah_fhah_engine_tx_0 = fhah.fhah_engine_tx(hop_interval, pre_guard, post_guard, dev_addr, samp_rate/samp_per_sym, freq_list)
        self.extras_pmt_rpc_0 = gr_extras.pmt_rpc(obj=self, result_msg=True)
        self.burst_gate_0 = fhah.burst_gate()

        ##################################################
        # Connections
        ##################################################
        self.connect((self.uhd_source, 0), (self.gmsk_demod, 0))
        self.connect((self.uhd_source, 0), (self.fhah_fhah_engine_tx_0, 0))
        self.connect((self, 0), (self.fhah_fhah_engine_tx_0, 1))
        self.connect((self.burst_gate_0, 0), (self.uhd_sink, 0))
        self.connect((self.gmsk_mod, 0), (self.gr_multiply_const_vxx_0, 0))
        self.connect((self.fhah_fhah_engine_tx_0, 0), (self.precog_packet_framer_0, 0))
        self.connect((self.fhah_fhah_engine_tx_0, 1), (self.extras_pmt_rpc_0, 0))
        self.connect((self.precog_packet_framer_0, 0), (self.gmsk_mod, 0))
        self.connect((self.gr_multiply_const_vxx_0, 0), (self.burst_gate_0, 0))
        self.connect((self.precog_packet_deframer_0, 0), (self.fhah_fhah_engine_tx_0, 2))
        self.connect((self.fhah_fhah_engine_tx_0, 2), (self, 0))
        self.connect((self.gmsk_demod, 0), (self.precog_packet_deframer_0, 0))


    def get_lead_limit(self):
        return self.lead_limit

    def set_lead_limit(self, lead_limit):
        self.lead_limit = lead_limit

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_sink.set_gain(self.tx_gain, 0)

    def get_samp_per_sym(self):
        return self.samp_per_sym

    def set_samp_per_sym(self, samp_per_sym):
        self.samp_per_sym = samp_per_sym

    def get_link_speed(self):
        return self.link_speed

    def set_link_speed(self, link_speed):
        self.link_speed = link_speed

    def get_pre_guard(self):
        return self.pre_guard

    def set_pre_guard(self, pre_guard):
        self.pre_guard = pre_guard

    def get_rate(self):
        return self.rate

    def set_rate(self, rate):
        self.rate = rate
        self.set_samp_rate(self.rate)
        self.set_samp_rate_0(self.rate)

    def get_hop_interval(self):
        return self.hop_interval

    def set_hop_interval(self, hop_interval):
        self.hop_interval = hop_interval

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_source.set_gain(self.rx_gain, 0)

    def get_ampl(self):
        return self.ampl

    def set_ampl(self, ampl):
        self.ampl = ampl
        self.gr_multiply_const_vxx_0.set_k((self.ampl, ))

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_sink.set_center_freq(self.freq, 0)
        self.uhd_source.set_center_freq(self.freq, 0)

    def get_args(self):
        return self.args

    def set_args(self, args):
        self.args = args

    def get_freq_list(self):
        return self.freq_list

    def set_freq_list(self, freq_list):
        self.freq_list = freq_list

    def get_post_guard(self):
        return self.post_guard

    def set_post_guard(self, post_guard):
        self.post_guard = post_guard

    def get_dev_addr(self):
        return self.dev_addr

    def set_dev_addr(self, dev_addr):
        self.dev_addr = dev_addr

    def get_samp_rate_0(self):
        return self.samp_rate_0

    def set_samp_rate_0(self, samp_rate_0):
        self.samp_rate_0 = samp_rate_0

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_sink.set_samp_rate(self.samp_rate)
        self.uhd_source.set_samp_rate(self.samp_rate)


