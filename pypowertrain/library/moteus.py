from pypowertrain.components.controller import *
from pypowertrain.components.motor import *


def n1():
	return Controller(
		phase_current_limit=100,
		power_limit=1200,
		bus_voltage_limit=54,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		field_weakening=False,
		freq_limit=3000,
		width=46e-3,
		length=46e-3,
		height=8e-3,
		weight=14.6e-3,
	)


def r4():
	return Controller(
		phase_current_limit=100,
		power_limit=500,
		bus_voltage_limit=44,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		field_weakening=False,
		freq_limit=3000,
		width=46e-3,
		length=53e-3,
		height=8e-3,
		weight=14.2e-3,
	)


def c1():
	return Controller(
		phase_current_limit=20,
		power_limit=100,
		bus_voltage_limit=51,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		field_weakening=False,
		freq_limit=3000,
		width=38e-3,
		length=38e-3,
		height=9e-3,
		weight=8.9e-3,
	)


def mj5208():
	"""https://mjbots.com/products/mj5208"""
	Kt = Kt_from_Kv(330)
	R = 50e-3 # line to center
	L = 30e-6 # line-to-neutral

	R = R * 1.5		# convert to q-d frame
	L = L * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		H_limit=500,  # At/mm @ 100C

		poles=7 * 2,
		slots=4 * 3,

		turns=5,  # FIXME: unknown!
		radius=54e-3 / 2,
		stack_height=8e-3,
		tooth_depth=5e-3,  # visual approximate

		# FIXME: all the below is barely considered guesswork!
		magnet_height=2.5e-3,
		airgap=0.7e-3,

		thermal_resistance=9.0,

		# FIXME: wild guesses
		# weight=0.193,
		iron_weight=0.08,
		copper_weight=0.06,
		magnet_weight=0.02,
		structure_weight=0.043
	)
