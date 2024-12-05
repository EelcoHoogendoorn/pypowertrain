from pypowertrain.components.controller import *


def maytech_100a():
	"""https://maytech.cn/products/maytech-upgrade-100a-v6-based-vesc"""
	return Controller(
		phase_current_limit=240,
		power_limit=8000,			# no clear figures?
		bus_voltage_limit=44,
		internal_resistance=3e-3,	# FIXME: any data?
		ripple_freq=48e3,
		freq_limit=150_000 / 60,
		width=43e-3,
		length=70e-3,
		height=16e-3,
		modulation_factor=0.95 * Controller.commutation['svpwm'],
	)


def triforce_a50s():
	return Controller(
		phase_current_limit=80,
		power_limit=3000,			# no clear figures?
		bus_voltage_limit=52,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		freq_limit=150_000 / 60,
		width=35.5e-3,
		length=21e-3,
		height=13.8e-3,
		weight=11e-3,
		modulation_factor=0.95 * Controller.commutation['svpwm'],
	)
