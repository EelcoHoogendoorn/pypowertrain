from pypowertrain.components.controller import Controller
from pypowertrain.library.vesc import maytech_100a
from pypowertrain.library import moteus

from pypowertrain.system import *
from pypowertrain.library import cubemars
from pypowertrain.components.battery import *


def test_cubemars_R100():
	"""Test to see what it takes to approximate graph on the official store"""
	print()
	system = System(
		actuator=Actuator(
			motor=cubemars.R100(),
			controller=maytech_100a().replace(
				bus_voltage_limit=48,
			),
		),
		battery=define_battery_75v(),
	)
	system_plot(system)


def test_cubemars_RO100():
	"""Test to see what it takes to approximate graph on the official store
	"""
	print()
	system = System(
		actuator=Actuator(
			motor=cubemars.RO100(),
			controller=maytech_100a().replace(
				bus_voltage_limit=48,
			),
		),
		battery=define_battery_75v(),
	)
	system_plot(system)


def test_cubemars_RI100():
	"""Test to see what it takes to approximate graph on the official store

	Seems like a good match, with fairly standard thermal parameters
	"""
	print()
	system = System(
		actuator=Actuator(
			motor=cubemars.RI100(),
			controller=maytech_100a().replace(
			),
		),
		battery=define_battery(v=24, wh=1e3),
	)
	print(system.actuator.motor.geometry.outer_radius)
	print(system.actuator.motor.mass.total)
	# return
	system.actuator.plot()
	system_plot(system)


def test_cubemars_RI50():
	print()
	system = System(
		actuator=Actuator(
			motor=cubemars.RI50(),
			controller=moteus.n1().replace(
				# modulation_factor=0.9 * Controller.commutation['sine']
			),
		),
		battery=define_battery(v=24, wh=1e3),
	)
	# system.actuator.plot()
	system_plot(system)


