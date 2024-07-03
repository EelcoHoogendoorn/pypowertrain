from pypowertrain.optimize import *
from pypowertrain.system import *
from pypowertrain.components.battery import *
from pypowertrain.components.gearing import *
from pypowertrain.library import grin, odrive


def test_optimize():
	"""Design a powertrain optimized for maximum stall torque per kg"""
	battery = define_battery_75v()
	gearing = Gearing(ratio=1, weight=1, efficiency=0.95, torque_limit=200, thickness=1e-2)
	actuator = grin.actuator(turns=8).replace(
		gearing=gearing,
		motor=grin.all_axle(5),
		controller=odrive.pro_overclock(),
	)
	system = System(actuator=actuator, battery=battery)

	bounds = {
		'actuator__motor__copper': (0.3, 1.0),
		# 'actuator__motor__magnet': (0.8, 1.2),	# shrinking airgap and magnets makes sense
		'actuator__motor__length': (0.5, 2.0),
		'actuator__motor__turns': (0.5, 2),
		'actuator__motor__radius': (0.5, 1.2),
		# there can be nontrivial tradeoffs in battery and motor mass
		'battery__S': (4, 8),
		'battery__P': (1, 5),
		# do we care? yeah fatter bus wires are a net win!
		'actuator__bus__area': (1e-3**2, 3e-3**2),
	}

	target_torque = [-100, +100]
	target_rpm = [50] * 2
	target_dissipation = [10000] * 2
	target_weight = [1] * 2

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

	optimized.actuator.plot()
	print(optimized)
	print(optimized.weight)
	print(optimized.actuator.motor.weight)
	print(optimized.actuator.motor.radius)
	print(optimized.actuator.motor.tooth_depth)
	print(optimized.actuator.motor.stack_height)
	print(optimized.actuator.motor.magnet_height)
	print(optimized.actuator.motor.turns)
	print(optimized.actuator.gearing.ratio)
	print(optimized.actuator.bus.area)

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
			controller=odrive.s1(),
		),
		battery=define_battery_limits(v=24, wh=1e3),
	)

	bounds = {
		'actuator__motor__turns': (0.2, 1),
	}

	target_torque = [-30, +30]
	target_rpm = [10] * 2
	target_dissipation = [10000] * 2
	target_weight = [1] * 2

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
	print(optimized.actuator.motor.turns)

	system = optimized.replace(battery__charge_state=0.99)
	system_plot(system, targets=targets)

