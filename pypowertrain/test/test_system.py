from pypowertrain.system import *
from pypowertrain.components.battery import *
from pypowertrain.library import grin, moteus, vesc, odrive


def test_plot():
	system = System(
		battery=define_battery_75v(),
		actuator=grin.actuator(turns=5)
	)
	system_plot(system)


def test_dash():
	system = System(
		# battery=define_battery_limits(v=75, wh=1e3),
		battery=define_battery_58v(),
		actuator=grin.actuator(turns=5, statorade=True).replace(
			controller=odrive.pro_overclock(),
			n_series=2,
			__geometry__slot_depth_scale=0.5,
			__geometry__turns=7,
			__geometry__radius_scale=1.00,
		)
	)
	print(system)
	# import pickle
	# print(type(str(pickle.dumps(system))))
	# foo  = (pickle.loads( pickle.dumps(system)))
	# system_plot(foo)
	# return
	system_dash(system)
