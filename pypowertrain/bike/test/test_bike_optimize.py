
from pypowertrain.optimize import *
from pypowertrain.bike.bike_models import *


def test_moped():
	"""System optimization applied to a dual drive moped"""
	bike = define_moped(front=True)

	system = System(
		actuator=bike.rear.replace(
			controller=odrive.pro(),
		),
		battery=bike.battery
	)
	bounds = {
		'actuator__motor__length': (0.8, 1.2),
		'actuator__motor__radius': (0.8, 1.2),
		'actuator__motor__magnet': (0.8, 1.2),
		'actuator__motor__copper': (0.5, 1.3),

		'actuator__bus__length': (1., 10.)
	}

	# FIXME: spec bike targets in bike coordinates? kmh and G
	rated_rpm = bike.kph_to_rpm(bike.nominal_kmh)
	n = 3
	# solid braking
	target_torque = [-100] * n
	target_rpm = list(np.linspace(rated_rpm * 0.5, rated_rpm * 1.0, n, endpoint=True))
	target_dissipation = [system.actuator.heat_capacity_copper(dT=40) / 2] * n
	target_weight = [3,4,5]

	# top speed capability
	target_torque += [20 / bike.n_motors]
	target_rpm += [rated_rpm]
	target_dissipation += [250]
	target_weight += [10]

	# decent hill climbing ability
	target_torque += [50 / bike.n_motors]
	target_rpm += [rated_rpm*0.8]
	target_dissipation += [250]
	target_weight += [5]

	conditions = [
		{
			'battery__charge_state': s,
			'actuator__motor__coil_temperature': t,
			'actuator__motor__magnet_temperature': t,
		}
		for s in [0.1, 0.5, 0.9]
		for t in [20, 80]
	]
	targets = target_torque, target_rpm, target_dissipation, target_weight
	optimized = system_optimize(system, bounds, targets, conditions).actuator

	# eval the optimized system
	print(optimized.motor)
	print('weight:', optimized.motor.weight)
	print('stack:', optimized.motor.stack_height)
	print('teeth:', optimized.motor.tooth_depth)
	print('radius:', optimized.motor.radius)
	print('magnet:', optimized.motor.magnet_height)
	print('resistance:', optimized.motor.resistance)
	print('bus length:', optimized.bus.length)
	optimized.plot()

	optimized = optimized.replace(
		motor__coil_temperature=60,
		motor__magnet_temperature=60,
	)
	bike = bike.replace(
		rear=optimized,
		front=None if bike.front is None else optimized,
		battery__charge_state=0.1,
		nominal_kmh=50,
		Cf=0.8,
	)

	bike_plot(bike, targets=targets)
