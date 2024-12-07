
from pypowertrain.system import *
from pypowertrain.library import moteus
from pypowertrain.components.battery import *


def fixture():
	system = System(
		actuator=Actuator(
			motor=moteus.mj5208(),
			controller=moteus.n1().replace(
				field_weakening=True,
			),
		),
		battery=define_battery_58v(P=10),
	)
	return system


def test_moteus_n1():
	"""Maximum rpm in the specs is substantially lower than what this model predicts.
	It might well be that the lack of rpm-dependent aero drag in our modelling thus far,
	is starting to run into its limits here.
	Alternatively, it could be the rpm limit is dictated by bearings.
	"""
	system = fixture()
	system_plot(system, annotations='tdeos')
	# system.actuator.plot()	# phase assignment broken for this motor


def test_saturation():
	system = fixture()
	# print(system.actuator.motor.electrical.attrs['saturation'])
	# return
	import matplotlib.pyplot as plt

	Iq = np.linspace(0, 100, 100)
	S = system.actuator.motor.electrical.saturation
	T = Iq * system.actuator.motor.electrical.Kt / system.actuator.motor.electrical.saturation_factor(Iq)
	plt.plot(Iq, T)
	plt.vlines(S, 0, T.max())
	plt.show()
