from pypowertrain.system import *
from pypowertrain.library import cubemars
from pypowertrain.components.battery import *
from pypowertrain.components.controller import *


def test_cubemars_R100():
	"""Test to see what it takes to approximate graph on the official store

	The 4-ish Nm continuous rated torque seems realistic

	It seems their high rpm numbers are based on a thermal model
	which is quite optimistic about iron losses to cooling rate at high rpm.
	"""
	print()
	system = System(
		actuator=Actuator(
			motor=cubemars.R100(),
			controller=define_ideal_controller().replace(
				bus_voltage_limit=48,
				# modulation_factor=1,  # need the full 48V to match to large rpm=0 torque peak
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
			controller=define_ideal_controller().replace(
				bus_voltage_limit=48,
				modulation_factor=1,	# need the full 48V to match to large rpm=0 torque peak
			),
		),
		battery=define_battery_75v(),
	)
	system_plot(system, max_rpm=6000, max_torque=40)


def test_cubemars_RO100():
	"""Test to see what it takes to approximate graph on the official store
	"""
	print()
	system = System(
		actuator=Actuator(
			motor=cubemars.RO100(),
			controller=define_ideal_controller().replace(
				# field_weakening=False,
				bus_voltage_limit=48,
				# modulation_factor=1/np.sqrt(3)
				modulation_factor=1,
			),
		),
		battery=define_battery_75v(),
	)
	system_plot(system)

