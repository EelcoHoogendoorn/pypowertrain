
from pypowertrain.optimize import *
from pypowertrain.bike.bike_models import *
from pypowertrain.app import system_dash


def test_moped():
	"""System optimization applied to a dual drive moped

	Out of the box, dual drive grin all axle makes for a pretty close to optimal 50kmh moped,
	though we can shave off some weight.

	The grin controller is a tad limiting in this scenario;
	more powerful controller would mean a lighter motor
	"""
	bike = define_moped(rear=True, front=True)
	# bike = bike.replace(__controller=odrive.pro_overclock())

	bounds = {
		'__geometry__turns': (4, 8),
		'__geometry__length_scale': (0.9, 1.1),
		'__geometry__radius_scale': (0.9, 1.1),
		'__geometry__reluctance_scale': (0.9, 1.1),
		'__geometry__slot_depth_scale': (0.4, 1.1),

		# 'actuator__bus__length': (1., 10.)
	}


	def get_dissipation(dt, dT=60):
		"""Translate a target time and temperature delta into a target thermal dissipation limit
		Note: this is computed on the original motor; if the optimized motor is significantly different
		in its thermal model, it invalidates this computation.
		"""
		return dT / bike.actuator.motor.thermal.solve({'coils': 1}, dt=dt)['coils']

	# FIXME: spec bike targets in bike coordinates? kmh and G
	rated_rpm = bike.load.kph_to_rpm(bike.load.nominal_kmh)
	n = 3
	# at 75nm each wheel should legally count as a brake
	target_torque = [-75] * n
	target_rpm = list(np.linspace(rated_rpm * 0.5, rated_rpm * 1.0, n, endpoint=True))
	target_dissipation = [get_dissipation(3)] * n	# should come to full stop in under 3s
	target_weight = [(0,1)]*3

	# top speed capability
	target_torque += [20 / bike.load.n_motors]
	target_rpm += [rated_rpm]
	target_dissipation += [get_dissipation(6000)]	# want this for full duration of a battery charge
	target_weight += [(0,1)]

	# decent hill climbing ability
	target_torque += [60 / bike.load.n_motors]
	target_rpm += [rated_rpm*0.8]
	target_dissipation += [get_dissipation(600)]	# 10 mins hill climbing is good neough
	target_weight += [(0,1)]

	conditions = [
		{
			'battery__charge_state': s,
			'actuator__motor__coil_temperature': t,
			'actuator__motor__magnet_temperature': t/2,
		}
		for s in [0.1, 0.5, 0.9]
		for t in [20, 80]
	]
	targets = target_torque, target_rpm, target_dissipation, target_weight

	optimized = system_optimize(bike, bounds, targets, conditions)

	# eval the optimized system
	motor = optimized.actuator.motor
	print(motor)
	print('weight:', motor.mass.total)
	print('stack:', motor.geometry.length)
	print('turns:', motor.geometry.turns)
	print('teeth:', motor.geometry.slot_depth)
	print('radius:', motor.geometry.radius)
	print('resistance:', motor.resistance)
	# optimized.actuator.plot()

	optimized = optimized.replace(
		__motor__coil_temperature=80,
		__motor__magnet_temperature=50,
	)
	bike = optimized.replace(
		# actuator=optimized.actuator,
		# front=None if bike.front is None else optimized,
		battery__charge_state=0.5,
		nominal_kmh=50,
		# Cf=0.8,
	)

	system_plot(bike, targets=targets, annotations='tdeosa')
