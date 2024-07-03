from pypowertrain.components.motor import *
from pypowertrain.components.actuator import *
from pypowertrain.components.controller import *


# def all_axle(turns=5):
# 	# grin fast
# 	if turns == 5:
# 		Kt = 0.79
# 		R = 0.1
# 		L = 260e-6
# 	if turns == 6:
# 		# medium
# 		Kt = 0.92
# 		R = 0.145
# 		L = 380e-6
# 	if turns == 8:
# 		# slow
# 		Kt = 1.28
# 		R = 0.268
# 		L = 680e-6
#
# 	return Motor(
# 		Kt=Kt,
# 		R=R,
# 		Lq=L,
# 		Ld=L,
# 		H_limit=500,	# At/mm @ 100C
#
# 		poles=46,
# 		slots=42,
# 		turns=5,
# 		radius=199e-3/2,
# 		stack_height=27e-3,
# 		tooth_depth=16e-3,
# 		magnet_width=10.7e-3,	#2.9 gap
# 		magnet_height=3e-3,
# 		airgap=0.8e-3,
#
# 		hysteresis_drag=0.6, #Nm
# 		eddy_pm=0.005,  # Nm/rad/s, or linear coefficient
# 		eddy_em=1e-8*0,
#
# 		weight=4.,
# 		# FIXME: should try and refine thesse
# 		tooth_weight=0.8,
# 		copper_weight=1.2,
# 		magnet_weight=0.15,
# 		back_weight=0.7,
#
# 		thermal_resistance=250 / 60 / (0.2*np.pi),
# 	)


def all_axle(turns=5):
	"""https://ebikes.ca/amfile/file/download/file/308/product/1859/

	grin values are phase-to-phase
	"""
	# grin fast
	if turns == 5:
		Kt = 0.79
		R = 0.1
		L = 260e-6
	if turns == 6:
		# medium
		Kt = 0.92
		R = 0.145
		L = 380e-6
	if turns == 8:
		# slow
		Kt = 1.28
		R = 0.268
		L = 680e-6

	# convert phase-to-phase to q-d frame
	R = R / 2 * 1.5
	L = L / 2 * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		H_limit=500,	# At/mm @ 100C

		poles=46,
		slots=42,
		turns=5,
		radius=199e-3/2,
		stack_height=27e-3,
		tooth_depth=16e-3,
		magnet_height=3e-3,
		airgap=0.8e-3,

		# hysteresis_drag=0.6, #Nm
		# eddy_pm=0.005,  # Nm/rad/s, or linear coefficient
		eddy_em=0,

		iron_weight=1.5,
		copper_weight=1.2,
		magnet_weight=0.2,
		structure_weight=1.1,

		thermal_resistance=250 / 60 / (0.2*np.pi),
	)


def phaserunner():
	""""""
	return Controller(
		phase_current_limit=90,
		power_limit=5000,
		bus_voltage_limit=75,
		internal_resistance=3e-3,	# why does grin calc show so much higher values?
		ripple_freq=48e3,
		field_weakening=True,
		freq_limit=60_000 / 60,
		width=40e-3,
		length=99e-3,
		height=34e-3,
		weight=0.26,
		# modulation_factor=1/np.sqrt(3),
	)


def actuator(turns):
	return Actuator(motor=all_axle(turns=turns), controller=phaserunner())
