from pypowertrain.components.actuator import *
from pypowertrain.components.battery import *
from pypowertrain.bike.bike import *
from pypowertrain.library import grin, odrive


def define_ebike(kmh, inch=24, turns=8):
	"""commuter ebike"""
	grin_actuator = grin.actuator(turns=turns)
	return Bike(
		rear=grin_actuator,
		front=None,
		battery=define_battery_75v(P=2),
		CdA=0.7*0.8,
		Cr=0.004,
		structure_weight=15,
		nominal_kmh=kmh,
		wheel_diameter=inch*25.4e-3
	)


def define_moped(front=False):
	"""light moped with a grin actuator"""
	grin_actuator = grin.actuator(turns=5)
	return Bike(
		rear=grin_actuator,
		front=grin_actuator if front else None,
		battery=define_battery_limits(v=75, wh=2500),
		CdA=0.6,
		Cr=0.010,	# could be up to 15e-3?
		structure_weight=30,
		nominal_kmh=50,
		wheel_diameter=0.5,	# 20inch wheel moped
	)


def define_motorcycle():
	"""Can we make an electric motorcycle work at highway speed,
	with dual direct drive hub motors?

	With regular aero, it is not looking great.
	Aerodynamic fairing is quite essential to making this puzzle fit
	"""

	actuator = grin.actuator(turns=5).replace(
		geometry__radius_scale=1.2,
		geometry__slot_depth=7e-3,
		controller=odrive.pro_overclock(),
		n_series=3,
		bus=Bus(1, 2e-3**2),
	)
	return Bike(
		rear=actuator,
		front=actuator,
		battery=define_battery_limits(v=75, wh=4000),
		CdA=0.6*0.3,	# slightly more frontal; but lets invest more in aero
		# CdA=0.7*0.8,	# check how regular compares
		Cr=0.02,# according to wikipedia 20e-3? could be as low as 15e-3? impossible to find sources
		structure_weight=30+15,
		nominal_kmh=120,
		wheel_diameter=24*25.3e-3,	# 24inch wheel
		cog_rear=0.7,
		cog_front=0.8,
	)
