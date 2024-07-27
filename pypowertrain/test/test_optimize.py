from pypowertrain.optimize import *
from pypowertrain.system import *
from pypowertrain.components.battery import *
from pypowertrain.components.gearing import *
from pypowertrain.library import grin, odrive


def test_optimize_torque():
	"""Design a powertrain optimized for maximum stall torque per kg"""
	battery = define_battery_75v()
	gearing = Gearing(ratio=1, weight=1, efficiency=0.95, torque_limit=200, thickness=1e-2)
	actuator = grin.actuator(turns=8).replace(
		gearing=gearing,
		motor=grin.all_axle(turns=5),
		controller=odrive.pro_overclock(),
	)
	system = System(actuator=actuator, battery=battery)

	bounds = {
		'__geometry__slot_depth_scale': (0.3, 1.0),
		'__geometry__length_scale': (0.5, 2.0),
		'__geometry__turns': (3, 7),
		'__geometry__radius_scale': (0.5, 1.2),
		# there can be nontrivial tradeoffs in battery and motor mass
		# 'battery__S': (4, 8),
		# 'battery__P': (1, 5),
		# do we care? yeah fatter bus wires are a net win!
		# 'actuator__bus__area': (1e-3**2, 3e-3**2),
	}

	target_torque = [-100, +100]
	target_rpm = [50] * 2
	target_dissipation = [10000] * 2
	target_weight = [(0, 1)] * 2

	# make sure optimized motor meets specs over a range of (adverse) conditions
	conditions = [
		{
			'battery__charge_state': s,
			'actuator__motor__coil_temperature': t,
			'actuator__motor__magnet_temperature': t,
		}
		for s in [0.1, 0.9]
		for t in [0, 40]
	]
	targets = target_torque, target_rpm, target_dissipation, target_weight


	print(system.weight)
	print(system.actuator.weight)

	optimized = system_optimize(system, bounds, targets, conditions)

	optimized.actuator.plot()
	# print(optimized)
	print(optimized.weight)
	print(optimized.actuator.weight)

	system = optimized.replace(battery__charge_state=0.5)
	system_plot(system, targets=targets)


def test_optimize_botwheel():
	"""Seems like the odrive botwheel does better all around with odrive controllers,
	when it has fewer turns.

	Lets find out how many fewer exactly.
	"""
	system = System(
		actuator=Actuator(
			motor=odrive.botwheel(),
			controller=odrive.pro(),
		),
		battery=define_battery_limits(v=58, wh=1e3),
	)

	bounds = {
		'__geometry__turns_scale': (0.2, 1),
	}

	target_torque = [-30, +30]
	target_rpm = [600] * 2
	target_dissipation = [10000] * 2
	target_weight = [(1, 1)] * 2

	# make sure optimized motor meets specs over a range of (adverse) conditions
	conditions = [
		{
			'battery__charge_state': s,
			'actuator__motor__coil_temperature': t,
			'actuator__motor__magnet_temperature': t,
		}
		for s in [0.1, 0.9]
		for t in [0, 40]
	]
	targets = target_torque, target_rpm, target_dissipation, target_weight

	optimized = system_optimize(system, bounds, targets, conditions)

	print(optimized)
	print(optimized.actuator.motor.geometry.turns)

	system = optimized.replace(battery__charge_state=0.1)
	system_plot(system, targets=targets)

