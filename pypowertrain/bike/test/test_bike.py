import numpy as np

from pypowertrain.bike.bike_models import *
from pypowertrain.app import system_dash


def test_example(kmh=40, inch=20):
	"""Simple minimal example"""
	bike = BikeSystem(
		load=BikeLoad(
			CdA=0.7 * 0.8,
			Cr=0.004,
			structure_weight=40,
			rider_weight=80,
			nominal_kmh=kmh,
			wheel_diameter=inch * 25.4e-3,
			front=True, rear=True,
		),
		actuator=Actuator(
			motor=grin.all_axle(turns=8),
			controller=odrive.pro(),
		),
		battery=define_battery(v=58, wh=500),
	)

	system_plot(bike, annotations='tdeosa')
	# system_dash(bike).run()


def test_moped():
	"""limited to 40kmh with a single grin on empty battery
	without optimizations braking could be better
	"""
	bike = define_moped(front=True)
	system_plot(bike)
	# system_dash(bike).run()


def test_motorcycle():
	bike = define_motorcycle()

	bike = bike.replace(
		battery__charge_state=0.1,
		__bus__length=10,
	)
	system_plot(bike)
	# system_dash(bike).run()


def test_abs_braking():
	"""make some plots to gain insight into traction behavior;
	interplay with both wheels, weight shift and friction"""
	# bike = define_motorcycle()	# 200 good value; 2/3g. rear tops out at 150, 0.32g
	bike = define_moped()	# 130 good value; 2/3g. rear tops out at 110, 0.35g

	# braking vs accel
	if True:
		tlims = -np.linspace(50, 300, 21)
		mph = np.linspace(0, bike.load.nominal_kmh, 50)[::-1]
	else:
		tlims = np.linspace(50, 300, 21)
		mph = np.linspace(0, bike.load.nominal_kmh, 50)

	cfs = np.linspace(0.2, 1.0, 22)

	def process(flim, cf):
		"""compute gs of stopping force"""
		force = abs_traction(
			bike.replace(__Cf=cf),
			np.ones_like(mph) * flim,
			np.ones_like(mph) * flim)
		x, _, t = integrate_traject(mph / 3.6, force / bike.weight, np.ones_like(mph), reverse=True)
		# divide by cf to get torque efficiency
		return (bike.load.nominal_kmh/3.6) / t / 9.81 #/ cf

	gs = [[process(flim, cf) for flim in tlims / bike.load.wheel_radius] for cf in cfs]

	import matplotlib.pyplot as plt
	plt.xlabel('torque each wheel')
	plt.ylabel('Cf')
	plt.contourf(tlims, cfs, gs)
	plt.colorbar()

	plt.figure()
	gs = [process(flim, 0.8) for flim in tlims / bike.load.wheel_radius]
	plt.plot(tlims, gs)
	plt.show()
