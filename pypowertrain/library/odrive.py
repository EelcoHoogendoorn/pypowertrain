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
		Kv=150,
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
		Kv=270,
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
	geometry = Geometry.create(
		pole_pairs=20,
		slot_triplets=12,
		turns=5,			# FIXME: unknown!

		gap_diameter=83e-3,
		gap_length=25e-3,
		slot_depth=7e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv=100,
		phase_to_neutral_R=24e-3,
		phase_to_neutral_L=9.97e-6,
		# d0=0.15, d1=0.0002,	# FIXME: set from dimensionless numbers?
	)

	mass = Mass.from_absolute(geometry=geometry, total=2.2)

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
		turns=12,

		gap_diameter=130e-3,
		gap_length=54e-3,
		slot_depth_fraction=0.08, # weight matched
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kt=0.951,
		phase_to_neutral_R=0.8,
		phase_to_neutral_L=1.7e-3,
		# d0=0.15, d1=0.0002,	# FIXME: set from dimensionless numbers?
	)

	mass = Mass.from_absolute(geometry=geometry, total=2.2)

	thermal = shelled_thermal(mass, tire=True)
	return Motor(
		electrical=electrical,
		thermal=thermal,
		mass=mass,
	)