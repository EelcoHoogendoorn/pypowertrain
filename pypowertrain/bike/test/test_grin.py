from pypowertrain.bike.bike_models import *


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
