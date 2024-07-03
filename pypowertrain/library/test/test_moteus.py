from pypowertrain.system import *
from pypowertrain.library import moteus
from pypowertrain.components.battery import *


def test_moteus_n1():
	"""unlike the cubemars motors, maximum rpm seems conservatively estimated"""
	system = System(
		actuator=Actuator(
			motor=moteus.mj5208(),
			controller=moteus.n1().replace(
				field_weakening=True
			),
		),
		battery=define_battery_58v(P=100),
	).replace(actuator__bus__length=0)
	system_plot(system)
