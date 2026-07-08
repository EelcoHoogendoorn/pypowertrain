import numpy as np

from pypowertrain.bike.bike_models import *
from pypowertrain.app import system_dash


def test_example(kmh=40, inch=20):
	"""Simple minimal example"""
	bike = BikeSystem(
		load=BikeLoad(
			CdA=0.7 * 0.8,
			Cr=0.004,
			structure_weight=20,
			rider_weight=80,
			nominal_kmh=kmh,
			wheel_diameter=inch * 25.4e-3,
			front=False, rear=True,
			Cf=0.7,
		),
		actuator=Actuator(
			motor=grin.all_axle(turns=8, statorade=True),
			controller=odrive.pro().replace(internal_resistance=0),
		),
		battery=define_battery(v=58, wh=500)#.replace(__peak_charge=0),
	)#.replace(__magnet_temperature=150)
	#.replace(__peak_charge=0)
	#.replace(__statorade=False)
	print(bike.battery.peak_charge_current)
	# return

	print(bike.actuator.motor.mass.total)
	# bike = bike.replace(__gap_length=45e-3)
	print(bike.actuator.motor.mass.total)

	system_plot(bike, color='power', rpm_negative=False, annotations='tdeosa', output='grin_with_odrive.png')
	# system_dash(bike).run()


def test_moped():
	"""limited to 40kmh with a single grin on empty battery
	without optimizations braking could be better
	"""
	bike = define_moped(front=True)
	system_plot(bike)
	# system_dash(bike).run()


def test_motorcycle():
	bike = define_motorcycle()

	bike = bike.replace(
		battery__charge_state=0.1,
		__bus__length=10,
	)
	system_plot(bike)
	# system_dash(bike).run()


def test_abs_braking():
	"""make some plots to gain insight into traction behavior;
	interplay with both wheels, weight shift and friction"""
	# bike = define_motorcycle()	# 200 good value; 2/3g. rear tops out at 150, 0.32g
	bike = define_moped()	# 130 good value; 2/3g. rear tops out at 110, 0.35g

	# braking vs accel
	if False:
		tlims = -np.linspace(10, 300, 21)
		mph = np.linspace(0, bike.load.nominal_kmh, 50)[::-1]
	else:
		tlims = np.linspace(10, 300, 21)
		mph = np.linspace(0, bike.load.nominal_kmh, 50)

	cfs = np.linspace(0.2, 1.0, 22)

	def process(flim, cf):
		"""compute gs of stopping force"""
		force = abs_traction(
			bike.replace(__Cf=cf),
			np.ones_like(mph) * flim,
			np.ones_like(mph) * flim)
		x, _, t = integrate_traject(mph / 3.6, force / bike.weight, np.ones_like(mph), reverse=True)
		# divide by cf to get torque efficiency
		return (bike.load.nominal_kmh/3.6) / t / 9.81 #/ cf

	gs = [[process(flim, cf) for flim in tlims / bike.load.wheel_radius] for cf in cfs]

	import matplotlib.pyplot as plt
	plt.xlabel('torque each wheel')
	plt.ylabel('Cf')
	plt.contourf(tlims, cfs, gs)
	plt.colorbar()

	plt.figure()
	gs = [process(flim, 0.8) for flim in tlims / bike.load.wheel_radius]
	plt.plot(tlims, gs)
	plt.show()


def test_euc():
	"""Lets try and model a high end EUC, like extreme bull commander
	https://ewheels.com/products/extreme-bull-gt-pro-3000wh-battery-4000w-motor-8kw-peak
	Expecting some 300Nm torque for under 10kg motor
	Seems easy enough with rescaled grin motor;
	little over 7kg, given solid rim and cast sideplates EUC has, makes sense.
	Pretty much capable of maxing contact patch even at speed

	with 1 turn we obtain stated free spin speed of 200kph,
	but a little weirded out why that would be the case?
	2 turns reaches 70mph / 112 kph demonstrated road speed

	18x3 slot triplets; so motor has even more than what we are using now
	probably using a delta termination?
	"""
	bike = define_moped(front=False, rear=True).replace(CdA=1.2, Cr=0.02, cog_height=0.9)
	from pypowertrain.library.vesc import maytech_100a
	# NOTE: reusing the bike model here for our EUC. weight shift logic in load model makes no sense but soit
	bike = bike.replace(
		__gap_radius=26e-2/2,
		__wheel_diameter=21*2.56e-2,
		__turns=2,
		__controller=maytech_100a().replace(
			phase_current_limit=1000, bus_voltage_limit=168, power_limit=20e3).replace(internal_resistance=1e-3/6),
		__battery=define_battery(v=168, wh=4.4e3),
	).replace(__termination='delta')
	# FIXME: termination changes thermals;
	#  because of controller internal resistance significant compared to low turn count
	print(bike.actuator.weight)
	system_plot(bike, annotations='tdeosa')
	# system_dash(bike).run()
