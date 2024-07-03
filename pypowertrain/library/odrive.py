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
	Kt = Kt_from_Kv(150)
	R = 39e-3		# phase resistance
	L = 24e-6		# line-to-neutral

	R = R * 1.5		# convert to q-d frame
	L = L * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		H_limit=500,	# At/mm @ 100C

		poles=7*2,
		slots=4*3,

		turns=5,			# FIXME: unknown!
		radius=53e-3/2,
		stack_height=64e-3,
		tooth_depth=5e-3,	# visual approximate

		# FIXME: all the below is barely considered guesswork!
		magnet_height=2.5e-3,
		airgap=0.8e-3,

		thermal_resistance=10,
		drag_1=0.003,  # intercept of drag; need to lower to meet no-load speed spec?

		# weight=0.890,
		# FIXME: wild guesses
		iron_weight=0.4,
		copper_weight=0.3,
		magnet_weight=0.1,
		structure_weight=0.09
	)


def D5065_270KV():
	"""https://odriverobotics.com/shop/odrive-custom-motor-d5065"""
	Kt = Kt_from_Kv(270)
	R = 39e-3		# phase resistance
	L = 15.7e-6		# line-to-neutral

	R = R * 1.5		# convert to q-d frame
	L = L * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		H_limit=500,	# At/mm @ 100C

		poles=7*2,
		slots=4*3,

		turns=5,			# FIXME: unknown!
		radius=43e-3/2,
		stack_height=54e-3,
		tooth_depth=5e-3,	# visual approximate

		# FIXME: all the below is barely considered guesswork!
		magnet_height=2.5e-3,
		airgap=0.8e-3,

		thermal_resistance=10,
		drag_1 = 0.003,	# intercept of drag; need to lower to meet no-load speed spec?

		# FIXME: wild guesses
		# weight=0.420,
		iron_weight=0.2,
		copper_weight=0.15,
		magnet_weight=0.05,
		structure_weight=0.02
	)


def M8325s_100KV():
	Kt = Kt_from_Kv(100)
	R = 24e-3		# phase-neutral resistance
	L = 9.97e-6		# line-to-neutral

	R = R * 1.5		# convert to q-d frame
	L = L * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,

		poles=20*2,
		slots=12*3,

		turns=5,			# FIXME: unknown!
		radius=83e-3/2,
		stack_height=25e-3,
		tooth_depth=7e-3,	# visual approximate

		# FIXME: all the below is barely considered guesswork!
		magnet_height=2.5e-3,
		airgap=0.8e-3,

		thermal_resistance=10,
		drag_1=0.003,  # intercept of drag; need to lower to meet no-load speed spec?

		# weight=0.840,
		# FIXME: wild guesses
		iron_weight=0.4,
		copper_weight=0.3,
		magnet_weight=0.04,
		structure_weight=0.1
	)


def botwheel():
	"""https://odriverobotics.com/shop/botwheels"""
	Kt = 0.951
	R = 0.8		# phase-neutral resistance
	L = 1.7e-3	# line-to-neutral

	R = R * 1.5	# convert to q-d frame
	L = L * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		H_limit=500,	# At/mm @ 100C

		poles=15*2,
		slots=9*3,

		turns=5,			# FIXME: unknown!
		radius=150e-3/2,
		stack_height=54e-3,
		tooth_depth=5e-3,	# visual approximate

		# FIXME: all the below is barely considered guesswork!
		magnet_height=2.5e-3,
		airgap=0.8e-3,

		# FIXME: wild guesses
		# weight=2.2,
		iron_weight=1.0,
		copper_weight=0.5,
		magnet_weight=0.2,
		structure_weight=0.5,

		thermal_resistance=3.0,
	)
