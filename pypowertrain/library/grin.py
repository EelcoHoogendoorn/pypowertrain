from pypowertrain.components.actuator import *
from pypowertrain.components.controller import *
from pypowertrain.components.motor import *


def all_axle(turns=5, statorade=True):
	"""https://ebikes.ca/amfile/file/download/file/308/product/1859/
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


	geometry = Geometry.create(
		poles=46,
		slots=42,
		turns=turns,

		gap_diameter=199e-3,
		gap_length=27e-3,
		slot_depth=12e-3,
		magnet_height=3e-3,
		airgap=0.7e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kt=Kt,
		phase_to_phase_R=R,
		phase_to_phase_L=L,
		d0=0.45, d1=0.0005,
	)

	mass = Mass.from_absolute(geometry=geometry, total=4.0)

	return Motor(
		electrical=electrical,
		thermal=shelled_thermal(mass, statorade=statorade),
		mass=mass,
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
		modulation_factor=0.9/np.sqrt(3),	# FIXME: unknown?
	)


def actuator(turns, statorade=True):
	return Actuator(motor=all_axle(turns=turns, statorade=statorade), controller=phaserunner())
