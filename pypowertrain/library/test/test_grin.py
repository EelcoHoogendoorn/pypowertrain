import matplotlib.pyplot as plt
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
	# print(system.actuator.motor.thermal)
	# return
	print(system.actuator.motor.iron_drag(np.linspace(0, 100, 10)))
	print(system.actuator.motor.R)
	print(system.actuator.motor.L*1000)
	print('dimensionless drag')
	print('field', system.actuator.motor.geometry.iron_field)
	print(system.actuator.motor.electrical.attrs['d_0'])
	print(system.actuator.motor.electrical.attrs['d_1'])
	print(system.actuator.motor.electrical.d_0/system.actuator.motor.electrical.d_1)
	print(system.actuator.motor.electrical.attrs)


# system_plot(system)


def test_drag():
	"""Specs, from
	https://ebikes.ca/product-info/grin-products/all-axle-hub-motor.html
	0.453+0.00052*rpm

	Translated into dimensionless units
	"""
	motor = grin.all_axle()
	f = motor.mass.stator * motor.mass.geometry.gap_radius
	print(0.453/f, 0.00052/f)


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
