
from pypowertrain.system import *
from pypowertrain.library import grin
from pypowertrain.components.battery import *



def test_grin():
	# FIXME: link to empirical data!
	system = System(
		actuator=grin.actuator(turns=5),
		battery=define_battery_75v(),
	)

	# system.actuator.plot()
	system_plot(system)


def test_drag():
	"""Specs, from
	https://ebikes.ca/product-info/grin-products/all-axle-hub-motor.html
	0.453+0.00052*rpm

	Translated into dimensionless units
	"""
	motor = grin.all_axle()
	import matplotlib.pyplot as plt
	kmh = np.linspace(0, 50, 100)
	rpm = kmh * 10
	drag =motor.electrical.iron_drag(rpm/60)
	print(motor.electrical.d_1)
	plt.plot(kmh, drag)
	motor = motor.replace(frequency_scale=0.85, slot_depth_scale=0.4)
	print(motor.electrical.d_1)
	drag =motor.electrical.iron_drag(rpm/60)
	plt.plot(kmh, drag)
	plt.show()


def test_thermal_resistance():
	"""Test to see that net thermal resistance is in line with empirical measurements

	https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/
	1.59W/K vanilla motor at standstill is realistic. 1400core 1100 shell estimated cp; K=3+3.5

	average K =2 for vanilla
	https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/post-726183
	goes up to 3 for vented hole approaches
	"""
	import matplotlib.pyplot as plt
	mps = np.linspace(5, 20, 200)

	motor = grin.all_axle()
	print(motor.thermal)
	W = 1
	R = np.array([motor.thermal.replace(linear=v, circumferential=v*0.3).solve({'coils': W}, dt=1e4)['coils'] for v in mps])
	plt.plot(mps, W/R)
	motor = grin.all_axle(statorade=False)
	R = np.array([motor.thermal.replace(linear=v, circumferential=v*0.3).solve({'coils': W}, dt=1e4)['coils'] for v in mps])
	plt.plot(mps, W/R)

	plt.xlabel('V (m/s)')
	plt.ylabel('1/R (K/W)')
	plt.show()
