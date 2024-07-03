from pypowertrain.bike.bike_models import *


def test_example(kmh=40, inch=24):
	"""Simple minimal example"""
	bike = Bike(
		front=None,
		rear=Actuator(
			motor=grin.all_axle(turns=8),
			controller=odrive.pro(),
		),
		battery=define_battery_limits(v=58, wh=500),
		CdA=0.7 * 0.8,
		Cr=0.004,
		structure_weight=15,
		nominal_kmh=kmh,
		wheel_diameter=inch * 25.4e-3
	)
	bike_plot(bike)


def test_ebike():
	"""Test for congruence with grin simulator

	Agrees with grin simulator in broad strokes?
	not perfect though
	Not sure their simulation includes field weakening? if so only a bit

	8t: 18/53 kmh 115nm
	5t: 46/85 kmh 70.5nm

	References
	----------
	8t
	https://ebikes.ca/tools/simulator.html?motor=cust_7.46_0.268_0.680_23_0.6_0.0185_0&batt=cust_75_0.1_80&cont=cust_75_90_0.03_V&wheel=20i&frame=cust_0.56_0.004&mass=120&hp=0&cont_b=cust_100_280_0.01_V&motor_b=MGRIN_SLW_SA&batt_b=B7223_AC&wheel_b=20i&frame_b=cust_0.2_0.008&mass_b=120&hp_b=0&wind=0&autothrot=false&throt=100&grade=0
	"""
	bike = define_ebike(kmh=50, inch=20, turns=8)

	bike = bike.replace(
		battery__charge_state=0.99,
		rear__controller=odrive.pro(),
	)
	print(bike.battery.voltage)

	bike_plot(bike)


def test_moped():
	"""limited to 40kmh with a single grin on empty battery
	without optimizations braking could be better
	"""
	bike = define_moped(front=False)

	bike = bike.replace(
		battery__charge_state=0.5
	)

	bike_plot(bike)


def test_motorcycle():
	bike = define_motorcycle()

	bike = bike.replace(
		battery__charge_state=0.1,
		rear__bus__length=10,
	)
	bike_plot(bike)


def test_abs_braking():
	"""make some plots to gain insight into traction behavior;
	interplay with both wheels, weight shift and friction"""
	bike = define_motorcycle()	# 200 good value; 2/3g. rear tops out at 150, 0.32g
	# bike = define_moped()	# 130 good value; 2/3g. rear tops out at 110, 0.35g

	# braking vs accel
	if True:
		tlims = -np.linspace(50, 300, 21)
		mph = np.linspace(0, bike.nominal_kmh, 50)[::-1]
	else:
		tlims = np.linspace(50, 300, 21)
		mph = np.linspace(0, bike.nominal_kmh, 50)

	cfs = np.linspace(0.2, 1.0, 22)

	def process(flim, cf):
		force = abs_traction(
			bike.replace(Cf=cf),
			np.ones_like(mph) * flim,
			np.ones_like(mph) * flim)
		x, _, t = integrate_traject(mph / 3.6, force / bike.weight, np.ones_like(mph), reverse=True)
		# divide by cf to get torque efficiency
		return (bike.nominal_kmh/3.6) / t / 9.81 #/ cf

	gs = [[process(flim, cf) for flim in tlims / bike.wheel_radius] for cf in cfs]

	import matplotlib.pyplot as plt
	plt.xlabel('torque each wheel')
	plt.ylabel('Cf')
	plt.contourf(tlims, cfs, gs)
	plt.colorbar()

	plt.figure()
	gs = [process(flim, 0.8) for flim in tlims / bike.wheel_radius]
	plt.plot(tlims, gs)
	plt.show()
