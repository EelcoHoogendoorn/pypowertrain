from pypowertrain.system import *
from pypowertrain.library import odrive
from pypowertrain.components.battery import *


def test_D6374_150KV():
	system = System(
		actuator=Actuator(
			motor=odrive.D6374_150KV(),
			controller=odrive.pro().replace(
				freq_limit=2000
			),
		),
		battery=define_battery_58v(),
	)
	system_plot(system)


def test_D5065_270KV():
	system = System(
		actuator=Actuator(
			motor=odrive.D5065_270KV(),
			controller=odrive.pro().replace(
				freq_limit=2000
			),
		),
		battery=define_battery_58v(),
	)
	system_plot(system)


def test_M8325s_100KV():
	system = System(
		actuator=Actuator(
			motor=odrive.M8325s_100KV(),
			controller=odrive.pro().replace(
				freq_limit=2000
			),
		),
		battery=define_battery_58v(),
	)
	system_plot(system)


def test_botwheel():
	"""This motor seems to be wound more for gimbal controllers
	Rescaling the turns seems like a pareto improvement,
	as far as odrive controllers are concerned
	"""
	system = System(
		actuator=Actuator(
			motor=odrive.botwheel().replace(
				turns=4
			),
			controller=odrive.s1().replace(
				bus_voltage_limit=24,
			),
		),
		battery=define_battery_limits(v=24, wh=1e3).replace(
			charge_state=0.99
		),
		# battery=define_battery_58v(),
	)
	system.actuator.plot()
	system_plot(system)


def test_botwheel_thermal_resistance():
	"""Current thermal model hits 5Nm rated torque right out of the box
	"""
	import matplotlib.pyplot as plt
	dt = 1e4
	kph = np.linspace(0, 50, 100)
	mps = kph / 3.6
	fig, ax = plt.subplots(1)
	thermal = odrive.botwheel().thermal
	W = 1
	K = [thermal.replace(linear=v, circumferential=v * (13/17)).solve({'coils': W}, dt=dt)['coils'] for v in mps]
	ax.plot(kph, W / np.array(K))
	plt.show()
