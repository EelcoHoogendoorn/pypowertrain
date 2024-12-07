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
		modulation_factor=0.9 * Controller.commutation['svpwm'],	# as per SVPWM firmware update https://github.com/mjbots/moteus/releases/tag/0.1-20241114
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
		modulation_factor=0.9 * Controller.commutation['svpwm'],	# as per SVPWM firmware update https://github.com/mjbots/moteus/releases/tag/0.1-20241114
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
		modulation_factor=0.9 * Controller.commutation['svpwm'],	# as per SVPWM firmware update https://github.com/mjbots/moteus/releases/tag/0.1-20241114
	)


def mj5208():
	"""https://mjbots.com/products/mj5208"""
	geometry = Geometry.create(
		pole_pairs=7,
		slot_triplets=4,
		turns=7, # fitted to dimensionless attrs
		coil_fill=0.45,
		slot_depth_fraction=0.4,
		slot_width_fraction=0.4,	# FIXME: important to know if we want to base dimensionless saturation on this

		gap_diameter=54e-3,
		gap_length=8e-3,
		magnet_height=2e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv_ll=330,
		R_ll=50e-3,
		L_ll=30e-6,
		# d0=0.03, d1=0.00003,	# FIXME: tune iron losses? or are defaults fine?
		saturation_nm=0.7,	# as measured by jpieper
	)

	# FIXME: make drone motor specific weight model?
	mass = Mass.from_absolute(geometry=geometry, total=0.193)

	return Motor(
		electrical=electrical,
		thermal=basic_thermal(mass, K0=0.25),	# as measured by josh pieper at low rpm
		mass=mass,
	)
