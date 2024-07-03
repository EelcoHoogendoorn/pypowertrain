from pypowertrain.system import *
from pypowertrain.library import vesc
from pypowertrain.library import grin
from pypowertrain.library import cubemars
from pypowertrain.components.battery import *


def test_maytech_100a():
	system = System(
		actuator=Actuator(
			motor=grin.all_axle(turns=8),
			controller=vesc.maytech_100a(),
		),
		battery=define_battery_75v(),
	)
	system_plot(system)


def test_triforce_a50s():
	system = System(
		actuator=Actuator(
			motor=cubemars.RI100(),
			controller=vesc.triforce_a50s(),
		),
		battery=define_battery_75v(),
	)
	system_plot(system)
