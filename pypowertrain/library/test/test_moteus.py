from pypowertrain.system import *
from pypowertrain.library import moteus
from pypowertrain.components.battery import *


def test_moteus_n1():
	"""Maximum rpm in the specs is just 70% of what this model predicts.
	It might well be that the lack of rpm-dependent aero drag in our modelling thus far,
	is starting to run into its limits here.
	"""
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
