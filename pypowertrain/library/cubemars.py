from pypowertrain.components.motor import Motor


def RI100():
	"""https://www.cubemars.com/goods-859-RI100.html"""
	Kt = 0.104
	R = 0.126
	L = 366.7e-6

	# convert phase-to-phase to q-d frame
	R = R / 2 * 1.5
	L = L / 2 * 1.5

	return Motor(
		inrunner=True,

		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		# need to jack up this number beyond usual default to not run into demag limits
		# when trying to match 0 rpm torque spike in documentation
		H_limit=1000,	# At/mm @ 20C

		poles=14*2,
		slots=8*3,

		turns=5,			# FIXME: unknown!
		radius=57e-3/2,
		stack_height=13e-3,
		tooth_depth=43e-3 / 2,

		thermal_resistance=10,
		drag_1=0.004,  # intercept of drag; need to lower to meet no-load speed spec?

		# # FIXME: all the below is barely considered guesswork!
		magnet_height=2.5e-3,
		airgap=0.8e-3,

		# NOTE: these are frameless specs; total is the sum of parts, no rest/structure weight
		# weight=0.5,
		iron_weight=0.25,
		copper_weight=0.2,
		magnet_weight=0.05,
		structure_weight=0,	# frameless
	)


def R100():
	"""https://www.cubemars.com/goods-945-R100.html"""
	Kt = 0.106
	R = 51e-3
	L = 33e-6

	# convert phase-to-phase to q-d frame
	R = R / 2 * 1.5
	L = L / 2 * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		H_limit=500,	# At/mm @ 100C

		poles=21*2,
		slots=12*3,		# guessed, not observed

		turns=5,			# FIXME: unknown!
		radius=47e-3/2,
		stack_height=15e-3,
		tooth_depth=16e-3,

		# FIXME: all the below is barely considered guesswork!
		magnet_height=2.5e-3,
		airgap=0.8e-3,

		thermal_resistance=8,	# this isnt an open-frame motor is it..
		drag_1=0.003,  # intercept of drag; need to lower to meet no-load speed spec?

		# weight=0.731,
		# FIXME: wild guesses
		iron_weight=0.3,
		copper_weight=0.2,
		magnet_weight=0.031,
		structure_weight=0.2
	)


def RO100():
	"""https://www.cubemars.com/goods-1159-RO100.html"""
	Kt = 0.2
	R = 143e-3
	L = 137e-6

	# convert phase-to-phase to q-d frame
	R = R / 2 * 1.5
	L = L / 2 * 1.5

	return Motor(
		Kt=Kt,
		R=R,
		Lq=L,
		Ld=L,
		H_limit=500,	# At/mm @ 100C

		poles=21*2,
		slots=12*3,

		turns=5,			# FIXME: unknown!
		radius=100e-3/2,
		stack_height=20e-3,
		tooth_depth=8e-3,	# visual approximate

		magnet_height=2.5e-3,
		airgap=0.9e-3,

		# weight=710e-3,	# this is the non-lite version
		# FIXME: wild guesses
		iron_weight=0.3,
		copper_weight=0.2,
		magnet_weight=0.1,
		structure_weight=0.11
	)
