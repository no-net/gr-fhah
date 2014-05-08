#!/usr/bin/env python
##################################################
# Gnuradio Python Flow Graph
# Title: Simple Trx
# Generated: Mon Apr  8 15:22:22 2013
##################################################

import os
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio import blocks
#from gnuradio.gr import firdes
from optparse import OptionParser
#import gnuradio.extras as gr_extras

script_dir = os.path.dirname(os.path.abspath(__file__))
execfile(script_dir + "/hier_blks/fhah_trx.py")

class simple_trx(gr.top_block):

	def __init__(self, max_arq_attempts=10, ampl=0.7, arq_timeout=.10, rate=1e6, rx_antenna="TX/RX", radio_addr=86, lead_limit=0.001, rate_0=1e6, link_speed=200, ampl_0=0.7, dest_addr=85, port="12357", rx_freq=4266e5, tx_freq=4266e5, tx_gain=0, dev_addr=1, freq=426e6, rx_gain=5, args="addr=192.168.10.4", samp_per_sym=5, post_guard=0.003, pre_guard=0.005, hop_interval=0.05, freq_list="4241e5,4251e5,4261e5,4271e5"):
		gr.top_block.__init__(self, "Simple Trx")

		##################################################
		# Parameters
		##################################################
		self.max_arq_attempts = max_arq_attempts
		self.ampl = ampl
		self.arq_timeout = arq_timeout
		self.rate = rate
		self.rx_antenna = rx_antenna
		self.radio_addr = radio_addr
		self.lead_limit = lead_limit
		self.rate_0 = rate_0
		self.link_speed = link_speed
		self.ampl_0 = ampl_0
		self.dest_addr = dest_addr
		self.port = port
		self.rx_freq = rx_freq
		self.tx_freq = tx_freq
		self.tx_gain = tx_gain
		self.dev_addr = dev_addr
		self.freq = freq
		self.rx_gain = rx_gain
		self.args = args
		self.samp_per_sym = samp_per_sym
		self.post_guard = post_guard
		self.pre_guard = pre_guard
		self.hop_interval = hop_interval
		self.freq_list = freq_list

		##################################################
		# Variables
		##################################################
		self.samp_rate = samp_rate = rate

		##################################################
		# Blocks
		##################################################
		self.fhah_trx_0 = fhah_trx(
			lead_limit=0.001,
			tx_gain=tx_gain,
			samp_per_sym=samp_per_sym,
			link_speed=samp_rate/samp_per_sym,
			pre_guard=pre_guard,
			rate=rate_0,
			hop_interval=hop_interval,
			rx_gain=rx_gain,
			ampl=ampl,
			freq=freq,
			args=args,
			freq_list=freq_list,
			post_guard=post_guard,
			dev_addr=dev_addr,
		)
		#self.extras_socket_msg_0_0 = gr_extras.socket_msg("TCP", "127.0.0.1", port, 0)
		self.extras_socket_msg_0_0 = blocks.socket_pdu("TCP_SERVER", "", "52001", 10000)

		##################################################
		# Connections
		##################################################
		self.connect((self.extras_socket_msg_0_0, 0), (self.fhah_trx_0, 0))
		self.connect((self.fhah_trx_0, 0), (self.extras_socket_msg_0_0, 0))


	def get_max_arq_attempts(self):
		return self.max_arq_attempts

	def set_max_arq_attempts(self, max_arq_attempts):
		self.max_arq_attempts = max_arq_attempts

	def get_ampl(self):
		return self.ampl

	def set_ampl(self, ampl):
		self.ampl = ampl

	def get_arq_timeout(self):
		return self.arq_timeout

	def set_arq_timeout(self, arq_timeout):
		self.arq_timeout = arq_timeout

	def get_rate(self):
		return self.rate

	def set_rate(self, rate):
		self.rate = rate
		self.set_samp_rate(self.rate)

	def get_rx_antenna(self):
		return self.rx_antenna

	def set_rx_antenna(self, rx_antenna):
		self.rx_antenna = rx_antenna

	def get_radio_addr(self):
		return self.radio_addr

	def set_radio_addr(self, radio_addr):
		self.radio_addr = radio_addr

	def get_lead_limit(self):
		return self.lead_limit

	def set_lead_limit(self, lead_limit):
		self.lead_limit = lead_limit

	def get_rate_0(self):
		return self.rate_0

	def set_rate_0(self, rate_0):
		self.rate_0 = rate_0
		self.fhah_trx_0.set_rate(self.rate_0)

	def get_link_speed(self):
		return self.link_speed

	def set_link_speed(self, link_speed):
		self.link_speed = link_speed

	def get_ampl_0(self):
		return self.ampl_0

	def set_ampl_0(self, ampl_0):
		self.ampl_0 = ampl_0

	def get_dest_addr(self):
		return self.dest_addr

	def set_dest_addr(self, dest_addr):
		self.dest_addr = dest_addr

	def get_port(self):
		return self.port

	def set_port(self, port):
		self.port = port

	def get_rx_freq(self):
		return self.rx_freq

	def set_rx_freq(self, rx_freq):
		self.rx_freq = rx_freq

	def get_tx_freq(self):
		return self.tx_freq

	def set_tx_freq(self, tx_freq):
		self.tx_freq = tx_freq

	def get_tx_gain(self):
		return self.tx_gain

	def set_tx_gain(self, tx_gain):
		self.tx_gain = tx_gain
		self.fhah_trx_0.set_tx_gain(self.tx_gain)

	def get_dev_addr(self):
		return self.dev_addr

	def set_dev_addr(self, dev_addr):
		self.dev_addr = dev_addr
		self.fhah_trx_0.set_dev_addr(self.dev_addr)

	def get_freq(self):
		return self.freq

	def set_freq(self, freq):
		self.freq = freq
		self.fhah_trx_0.set_freq(self.freq)

	def get_rx_gain(self):
		return self.rx_gain

	def set_rx_gain(self, rx_gain):
		self.rx_gain = rx_gain
		self.fhah_trx_0.set_rx_gain(self.rx_gain)

	def get_args(self):
		return self.args

	def set_args(self, args):
		self.args = args
		self.fhah_trx_0.set_args(self.args)

	def get_samp_per_sym(self):
		return self.samp_per_sym

	def set_samp_per_sym(self, samp_per_sym):
		self.samp_per_sym = samp_per_sym
		self.fhah_trx_0.set_samp_per_sym(self.samp_per_sym)
		self.fhah_trx_0.set_link_speed(self.samp_rate/self.samp_per_sym)

	def get_post_guard(self):
		return self.post_guard

	def set_post_guard(self, post_guard):
		self.post_guard = post_guard
		self.fhah_trx_0.set_post_guard(self.post_guard)

	def get_pre_guard(self):
		return self.pre_guard

	def set_pre_guard(self, pre_guard):
		self.pre_guard = pre_guard
		self.fhah_trx_0.set_pre_guard(self.pre_guard)

	def get_hop_interval(self):
		return self.hop_interval

	def set_hop_interval(self, hop_interval):
		self.hop_interval = hop_interval
		self.fhah_trx_0.set_hop_interval(self.hop_interval)

	def get_freq_list(self):
		return self.freq_list

	def set_freq_list(self, freq_list):
		self.freq_list = freq_list
		self.fhah_trx_0.set_freq_list(self.freq_list)

	def get_samp_rate(self):
		return self.samp_rate

	def set_samp_rate(self, samp_rate):
		self.samp_rate = samp_rate
		self.fhah_trx_0.set_link_speed(self.samp_rate/self.samp_per_sym)

if __name__ == '__main__':
	parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
	parser.add_option("", "--max-arq-attempts", dest="max_arq_attempts", type="intx", default=10,
		help="Set max_arq_attempts [default=%default]")
	parser.add_option("", "--ampl", dest="ampl", type="eng_float", default=eng_notation.num_to_str(0.7),
		help="Set a [default=%default]")
	parser.add_option("", "--arq-timeout", dest="arq_timeout", type="eng_float", default=eng_notation.num_to_str(.10),
		help="Set arq_timeout [default=%default]")
	parser.add_option("", "--rate", dest="rate", type="eng_float", default=eng_notation.num_to_str(1e6),
		help="Set S [default=%default]")
	parser.add_option("", "--rx-antenna", dest="rx_antenna", type="string", default="TX/RX",
		help="Set rx_antenna [default=%default]")
	parser.add_option("", "--radio-addr", dest="radio_addr", type="intx", default=86,
		help="Set radio_addr [default=%default]")
	parser.add_option("", "--lead-limit", dest="lead_limit", type="eng_float", default=eng_notation.num_to_str(0.001),
		help="Set Lead Limit(s) [default=%default]")
	parser.add_option("", "--rate-0", dest="rate_0", type="eng_float", default=eng_notation.num_to_str(1e6),
		help="Set S [default=%default]")
	parser.add_option("", "--link-speed", dest="link_speed", type="eng_float", default=eng_notation.num_to_str(200),
		help="Set Link Speed(bps) [default=%default]")
	parser.add_option("", "--ampl-0", dest="ampl_0", type="eng_float", default=eng_notation.num_to_str(0.7),
		help="Set a [default=%default]")
	parser.add_option("", "--dest-addr", dest="dest_addr", type="intx", default=85,
		help="Set dest_addr [default=%default]")
	parser.add_option("", "--port", dest="port", type="string", default="12356",
		help="Set port [default=%default]")
	parser.add_option("", "--rx-freq", dest="rx_freq", type="eng_float", default=eng_notation.num_to_str(4266e5),
		help="Set rx_freq [default=%default]")
	parser.add_option("", "--tx-freq", dest="tx_freq", type="eng_float", default=eng_notation.num_to_str(4266e5),
		help="Set tx_freq [default=%default]")
	parser.add_option("", "--tx-gain", dest="tx_gain", type="eng_float", default=eng_notation.num_to_str(5),
		help="Set tx_gain [default=%default]")
	parser.add_option("", "--dev-addr", dest="dev_addr", type="intx", default=1,
		help="Set Device Address [default=%default]")
	parser.add_option("", "--freq", dest="freq", type="eng_float", default=eng_notation.num_to_str(426e6),
		help="Set freq [default=%default]")
	parser.add_option("", "--rx-gain", dest="rx_gain", type="eng_float", default=eng_notation.num_to_str(5),
		help="Set rx_gain [default=%default]")
	parser.add_option("", "--args", dest="args", type="string", default="addr=192.168.10.4",
		help="Set addr=192.168.10.4 [default=%default]")
	parser.add_option("", "--samp-per-sym", dest="samp_per_sym", type="intx", default=5,
		help="Set sps [default=%default]")
	parser.add_option("", "--post-guard", dest="post_guard", type="eng_float", default=eng_notation.num_to_str(0.003),
		help="Set Post Guard(s) [default=%default]")
	parser.add_option("", "--pre-guard", dest="pre_guard", type="eng_float", default=eng_notation.num_to_str(0.06),
		help="Set Pre Guard(s) [default=%default]")
	parser.add_option("", "--hop-interval", dest="hop_interval", type="eng_float", default=eng_notation.num_to_str(0.2),
		help="Set Hop Interval(s) [default=%default]")
	parser.add_option("", "--freq-list", dest="freq_list", type="string", default="4241e5,4251e5,4261e5,4271e5",
		help="Set freq_list [default=%default]")
	(options, args) = parser.parse_args()
	tb = simple_trx(max_arq_attempts=options.max_arq_attempts, ampl=options.ampl, arq_timeout=options.arq_timeout, rate=options.rate, rx_antenna=options.rx_antenna, radio_addr=options.radio_addr, lead_limit=options.lead_limit, rate_0=options.rate_0, link_speed=options.link_speed, ampl_0=options.ampl_0, dest_addr=options.dest_addr, port=options.port, rx_freq=options.rx_freq, tx_freq=options.tx_freq, tx_gain=options.tx_gain, dev_addr=options.dev_addr, freq=options.freq, rx_gain=options.rx_gain, args=options.args, samp_per_sym=options.samp_per_sym, post_guard=options.post_guard, pre_guard=options.pre_guard, hop_interval=options.hop_interval, freq_list=options.freq_list)
	tb.start()
	raw_input('Press Enter to quit: ')
	tb.stop()

