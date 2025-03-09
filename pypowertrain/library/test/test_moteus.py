
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


def test_M8318s_110KV():
	"""Compare to moteus loss measurements"""
	from pypowertrain.library import odrive
	motor = odrive.M8325s_100KV()
	motor = motor.replace(__gap_length=18e-3, __turns_scale=1.1, __coil_fill=0.8, __reluctance_scale=0.9)

	print(motor.L_ll)
	print(motor.R_ll)
	print(motor.Kv_ll)
	# return
	system = System(
		actuator=Actuator(
			motor=motor,
			controller=moteus.n1().replace(
				freq_limit=2000,
				ripple_freq=15e3*1,
			),
		),
		battery=define_battery(v=24, wh=1000),
	)

	system_plot(system)

