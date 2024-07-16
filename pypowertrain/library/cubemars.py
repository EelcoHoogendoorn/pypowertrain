from pypowertrain.components.motor import *


def R100():
	"""https://www.cubemars.com/goods-945-R100.html"""
	geometry = Geometry.create(
		pole_pairs=21,
		slot_triplets=12,		# guessed, not observed
		turns=5, # FIXME: unknown

		gap_radius=42e-3,
		gap_length=20e-3,
		slot_depth_fraction=0.2,

		magnet_height=2.0e-3,
		airgap=0.5e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kt=0.106,
		phase_to_phase_R=51e-3,
		phase_to_phase_L=33e-6,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.731)

	return Motor(
		electrical=electrical,
		thermal=basic_thermal(mass, K0=1.8, Kc=2.0, circumferential=30),
		mass=mass,
	)


def RO100():
	"""https://www.cubemars.com/goods-1159-RO100.html"""
	geometry = Geometry.create(
		inrunner=False,
		pole_pairs=21,
		slot_triplets=12,
		turns=5, # FIXME: unknown


		gap_diameter=100e-3,
		gap_length=20e-3,
		slot_depth_fraction=0.16,	# visual approximate

		magnet_height=2.5e-3,
		airgap=0.8e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kt=0.2,
		phase_to_phase_R=143e-3,
		phase_to_phase_L=137e-6,
	)

	# FIXME: make frameless specific weight model?
	mass = Mass.from_absolute(geometry=geometry, total=0.71, tuning={'structure': 0})

	# FIXME: all thermal models of frameless motors are kindof nonsense, no?
	#  it means very little without context about the testing setup
	return Motor(
		electrical=electrical,
		thermal=basic_thermal(mass, K0=2.1),	# tuned to 4Nm @ 2000rpm
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
		# d0=0.03, d1=0.00003,	# FIXME: tune
		# need to jack up this number beyond usual default to not run into demag limits
		# when trying to match 0 rpm torque spike in documentation
		H_limit=1000,	# At/mm @ 20C
	)

	# FIXME: make drone motor specific weight model?
	mass = Mass.from_absolute(geometry=geometry, total=0.5)

	return Motor(
		electrical=electrical,
		thermal=basic_thermal(mass, K0=1/4),
		mass=mass,
	)
