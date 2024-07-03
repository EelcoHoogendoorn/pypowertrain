import numpy as np

from pypowertrain.system import *
from pypowertrain.library import grin
from pypowertrain.components.battery import *


def test_grin():
	# FIXME: link to empirical data!
	system = System(
		actuator=grin.actuator(turns=8),
		battery=define_battery_75v(),
	)
	# print(system.actuator.motor.hysteresis_drag(np.linspace(0, 100, 10)))

	system_plot(system)
