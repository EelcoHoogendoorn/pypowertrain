
from pypowertrain.system import *
from pypowertrain.library import odrive
from pypowertrain.components.battery import *


def test_D6374_150KV():
	system = System(
		actuator=Actuator(
			motor=odrive.D6374_150KV(),
			controller=odrive.pro().replace(
				freq_limit=2000,
			),
		),
		battery=define_battery(v=24, wh=1e3),
	)
	system_plot(system)


def test_D5065_270KV():
	# FIXME: this combo has a very hugh ripple current
	#  leads to artificially restricted performance; need to tune model accordingly
	system = System(
		actuator=Actuator(
			motor=odrive.D5065_270KV(),
			controller=odrive.micro().replace(
				freq_limit=2000,
				field_weakening=True,
			),
		),
		battery=define_battery(v=24, wh=1e3),
	)
	system_plot(system, color='dissipation', annotations='tdeos')


def test_M8325s_100KV():
	system = System(
		actuator=Actuator(
			motor=odrive.M8325s_100KV(),
			controller=odrive.pro().replace(
				freq_limit=2000
			),
		),
		battery=define_battery(v=58, wh=1000),
	)
	system_plot(system)


def test_botwheel():
	"""This motor seems to be wound more for gimbal controllers
	Rescaling the turns seems like a pareto improvement,
	as far as odrive controllers are concerned
	"""
	system = System(
		actuator=Actuator(
			motor=odrive.botwheel().replace(
				turns=4
			),
			controller=odrive.pro().replace(
				field_weakening=True,
			),
		),
		battery=define_battery(v=48, wh=1e3),
	)
	# system.actuator.motor.electrical.attrs['saturation'] =1000
	system_plot(system, color='power')


def test_botwheel_anim():
	"""Code for generating an animated GIF"""
	system = System(
		actuator=Actuator(
			motor=odrive.botwheel(),
			controller=odrive.micro().replace(
				field_weakening=True,
			),
		),
		battery=define_battery(v=24, wh=1e3).replace(
			charge_state=0.99
		),
	)

	import matplotlib.pyplot as plt
	for i in range(2, 12):
		system = system.replace(__turns=i)
		system_plot(system, max_rpm=1000, max_torque=33)
		plt.savefig(f'rewind{i}.png')
		plt.close()



def test_botwheel_thermal_resistance():
	"""Current thermal model hits 5Nm rated torque right out of the box
	"""
	import matplotlib.pyplot as plt
	dt = 1e4
	kph = np.linspace(0, 50, 100)
	mps = kph / 3.6
	fig, ax = plt.subplots(1)
	thermal = odrive.botwheel().thermal
	W = 1
	K = [thermal.replace(linear=v, circumferential=v * (13/17)).solve({'coils': W}, dt=dt)['coils'] for v in mps]
	ax.plot(kph, W / np.array(K))
	plt.show()
