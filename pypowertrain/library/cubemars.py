from pypowertrain.components.motor import *


def R100():
	"""https://www.cubemars.com/goods-945-R100.html"""
	geometry = Geometry.create(
		pole_pairs=21,
		slot_triplets=12,		# guessed, not observed
		turns=3,  # matched from test_compare

		gap_radius=46e-3,
		gap_length=25e-3,
		slot_depth_fraction=0.5,
		slot_width_fraction=0.5,

		magnet_height=2.0e-3,
		airgap=0.5e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		# Kv=90*1.25,	# this fits the specs better?
		Kv=90,
		phase_to_phase_R=51e-3,
		phase_to_phase_L=33e-6,
	)

	# FIXME: large normalizer to match geometry to motor specs
	mass = Mass.from_absolute(geometry=geometry, total=0.731)
	print(mass.get_attrs())

	return Motor(
		electrical=electrical,
		thermal=basic_thermal(mass, K0=1.0, Kc=3.0, circumferential=20),
		mass=mass,
	)


def RO100():
	"""https://www.cubemars.com/goods-1159-RO100.html"""
	geometry = Geometry.create(
		pole_pairs=21,
		slot_triplets=12,
		turns=5, # matched from dimensionless attributes

		gap_diameter=100e-3,
		gap_length=20e-3,	# from drawing
		slot_depth=9e-3,	# from drawing
		coil_fill=0.5,

		magnet_height=2.5e-3,
		airgap=0.8e-3,	# from drawing
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv=55,
		phase_to_phase_R=143e-3,
		phase_to_phase_L=137e-6,
	)

	# FIXME: make frameless specific weight model?
	mass = Mass.from_absolute(geometry=geometry, total=0.71, tuning={'structure': 0})

	# FIXME: all thermal models of frameless motors are kindof nonsense, no?
	#  it means very little without context about the testing setup
	return Motor(
		electrical=electrical,
		thermal=basic_thermal(mass, K0=1.1, Kc=1.5, circumferential=20),	# tuned to 4Nm @ 2000rpm
		mass=mass,
	)


def RI100():
	"""https://www.cubemars.com/goods-859-RI100.html"""
	geometry = Geometry.create(
		inrunner=True,
		pole_pairs=14,
		slot_triplets=8,
		turns=5, # FIXME: unknown

		gap_diameter=57e-3,
		gap_length=13e-3,
		slot_depth_fraction=0.3,	# visual approximate

		magnet_height=2.5e-3,
		airgap=0.8e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kt=0.104,
		phase_to_phase_R=126e-3,
		phase_to_phase_L=366.7e-6,
	)

	# FIXME: make drone motor specific weight model?
	mass = Mass.from_absolute(geometry=geometry, total=0.5)

	return Motor(
		electrical=electrical,
		thermal=basic_thermal(mass, K0=1/4),
		mass=mass,
		# need to jack up this number beyond usual default to not run into demag limits
		# when trying to match 0 rpm torque spike in documentation
		H_limit=1000,  # At/mm @ 20C
	)
