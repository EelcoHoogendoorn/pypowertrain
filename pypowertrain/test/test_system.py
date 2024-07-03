from pypowertrain.system import *
from pypowertrain.components.battery import *
from pypowertrain.library import grin


def test_plot():
	system = System(
		battery=define_battery_75v(),
		actuator=grin.actuator(turns=5)
	)
	system_plot(system)
