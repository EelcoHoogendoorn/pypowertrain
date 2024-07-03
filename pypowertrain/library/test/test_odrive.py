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
	"""This thing wasnt built for efficiency; or maybe doing something wrong?
	Rescaling the turns seems like a pareto improvement
	"""
	system = System(
		actuator=Actuator(
			motor=odrive.botwheel().rescale(
				turns=0.4
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
