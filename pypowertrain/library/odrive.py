import numpy as np

from pypowertrain.components.controller import *
from pypowertrain.components.motor import *


def pro():
	"""https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html#id1"""
	return Controller(
		phase_current_limit=120,
		power_limit=5000,
		bus_voltage_limit=56,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		freq_limit=700,
		width=51e-3,
		length=64e-3,
		weight=35e-3,
		modulation_factor=0.99 * Controller.commutation['svpwm'],
	)

def pro_nominal():
	return pro().replace(
		power_limit=3000,
		phase_current_limit=70
	)

def pro_overclock():
	return pro().replace(
		power_limit=7000,
		bus_voltage_limit=58,
		phase_current_limit=120,	# 3s limit according to docs; 150A also mentioned informally
		freq_limit=2000,	# upcoming firmware
	)


def s1():
	"""https://docs.odriverobotics.com/v/latest/hardware/s1-datasheet.html"""
	return Controller(
		phase_current_limit=80,	# this is transient peak; also bus voltage dependent, currently ignored!
		power_limit=2000,		# no known data?
		bus_voltage_limit=48,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		freq_limit=700,
		width=50e-3,
		length=66e-3,
		weight=35e-3,
		modulation_factor=0.78 * Controller.commutation['svpwm'],
	)


def micro():
	"""https://docs.odriverobotics.com/v/latest/hardware/micro-datasheet.html"""
	return Controller(
		phase_current_limit=7,
		power_limit=180,
		bus_voltage_limit=30,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		freq_limit=700,
		width=35e-3,
		length=35e-3,
		weight=7.5e-3,
		modulation_factor=0.99 * Controller.commutation['svpwm'],
	)


def D6374_150KV():
	"""https://odriverobotics.com/shop/odrive-custom-motor-d6374-150kv
	https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html
	"""
	geometry = Geometry.create(
		pole_pairs=7,
		slot_triplets=4,
		turns=5,			# FIXME: unknown!

		gap_diameter=83e-3,
		gap_length=25e-3,
		slot_depth=7e-3,
		magnet_height=2e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv_ll=150,
		R_ll=39e-3*2,
		L_ll=24e-6*2,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.89)

	thermal = basic_thermal(mass, K0=0.66)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)


def D5065_270KV():
	"""https://odriverobotics.com/shop/odrive-custom-motor-d5065
	https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html
	"""
	geometry = Geometry.create(
		pole_pairs=7,
		slot_triplets=4,
		turns=5,			# FIXME: unknown!

		gap_diameter=83e-3,
		gap_length=25e-3,
		slot_depth=7e-3,
		magnet_height=2e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv_ll=270,
		R_ll=39e-3*2,
		L_ll=16e-6*2,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.42)

	thermal = basic_thermal(mass, K0=0.66)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)


def M8325s_100KV():
	"""https://odriverobotics.com/shop/m8325s
	https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html
	"""
	geometry = Geometry.create(
		pole_pairs=20,
		slot_triplets=12,
		turns=2,			# FIXME: unknown!

		gap_diameter=83e-3,
		gap_length=25e-3,
		slot_depth=7e-3,
		magnet_height=2e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv_ll=100,
		R_ll=24e-3*2,
		L_ll=9.9e-6*2,
		# termination='delta',
	)
	# electrical = Electrical.from_absolute(
	# 	geometry=geometry,
	# 	Kv_ll=460/14,
	# 	R_ll=2.732*2,
	# 	L_ll=762e-6*2,
	# )


	mass = Mass.from_absolute(geometry=geometry, total=0.840)

	thermal = basic_thermal(mass, K0=0.66)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)


def M5312s_330KV():
	"""https://odriverobotics.com/shop/dual-shaft-motor-d5212s-300kv
	https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html
	"""
	geometry = Geometry.create(
		pole_pairs=7,
		slot_triplets=4,
		turns=5,			# FIXME: unknown!

		gap_diameter=53e-3,
		gap_length=12e-3,
		slot_depth=7e-3,	# rough estimate
		magnet_height=1.5e-3, # FIXME: getting a decent estimate of this is quite critical to dimensionless core saturation estimation
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv_ll=330,
		R_ll=37e-3*2,
		L_ll=23e-6*2,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.230)

	thermal = basic_thermal(mass, K0=0.66)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)


def botwheel():
	"""https://odriverobotics.com/shop/botwheels"""

	geometry = Geometry.create(
		pole_pairs=15,
		slot_triplets=9,
		turns=12,	# matched via dimensionless comparison

		gap_diameter=130e-3,
		gap_length=54e-3,
		airgap=0.6e-3,
		magnet_height=2e-3,
		slot_depth_fraction=0.08,  # weight matched
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv_ll=8.7,
		# NOTE: as provided on odrive discord
		R_ll=0.8*2,
		L_ll=1.7e-3*2,
	)

	mass = Mass.from_absolute(geometry=geometry, total=2.2-0.2)	# subtract rubber wheel estimate

	thermal = shelled_thermal(mass, rim_exposure=0.0)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)