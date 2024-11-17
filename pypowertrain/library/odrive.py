import numpy as np

from pypowertrain.components.controller import *
from pypowertrain.components.motor import *


def pro():
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
		modulation_factor=0.99 / np.sqrt(3),
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
		phase_current_limit=150,
		freq_limit=2000,	# upcoming firmware
	)


def s1():
	# FIXME: modulation factor 0.7 lower than the pro?
	return Controller(
		phase_current_limit=80,
		power_limit=2000,		# no known data?
		bus_voltage_limit=48,
		internal_resistance=3e-3,
		ripple_freq=48e3,
		freq_limit=700,
		width=50e-3,
		length=66e-3,
		weight=35e-3,
		modulation_factor=0.7 / np.sqrt(3),
	)


def micro():
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
		modulation_factor=0.99 / np.sqrt(3),
	)


def D6374_150KV():
	"""https://odriverobotics.com/shop/odrive-custom-motor-d6374-150kv"""
	geometry = Geometry.create(
		pole_pairs=7,
		slot_triplets=4,
		turns=5,			# FIXME: unknown!

		gap_diameter=83e-3,
		gap_length=25e-3,
		slot_depth=7e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		phase_to_phase_Kv=150,
		phase_to_neutral_R=39e-3,	# FIXME: same as 270kv? seems liek an error
		phase_to_neutral_L=24e-6,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.89)

	thermal = basic_thermal(mass, K0=0.66)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)


def D5065_270KV():
	"""https://odriverobotics.com/shop/odrive-custom-motor-d5065"""
	geometry = Geometry.create(
		pole_pairs=7,
		slot_triplets=4,
		turns=5,			# FIXME: unknown!

		gap_diameter=83e-3,
		gap_length=25e-3,
		slot_depth=7e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		phase_to_phase_Kv=270,
		phase_to_neutral_R=39e-3,
		phase_to_neutral_L=15.7e-6,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.42)

	thermal = basic_thermal(mass, K0=0.66)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)


def M8325s_100KV():
	"""https://odriverobotics.com/shop/m8325s"""
	geometry = Geometry.create(
		pole_pairs=20,
		slot_triplets=12,
		turns=2,			# FIXME: unknown!

		gap_diameter=83e-3,
		gap_length=25e-3,
		slot_depth=7e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		phase_to_phase_Kv=100,			# FIXME: data entry problem? we struggle to match realistic values here in test_compare
		phase_to_neutral_R=24e-3,
		phase_to_neutral_L=9.97e-6,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.840)

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
		magnet_height=1e-3,
		slot_depth_fraction=0.08,  # weight matched
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kt=0.951,
		phase_to_neutral_R=0.8,
		phase_to_neutral_L=1.7e-3,
	)

	mass = Mass.from_absolute(geometry=geometry, total=2.2-0.2)	# subtract rubber wheel?

	thermal = shelled_thermal(mass, rim_exposure=0.0)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)