from pypowertrain.bike.bike_models import *


def test_ebike():
	"""Test for congruence with grin simulator

	Agrees with grin simulator in broad strokes.

	Load lines seems to agree;
	simulator does not seem to do field weakening,

	There does seem to be a disrepancy in the predicted top speed,
	but then again we do not know the phaserunner modulation depth,
	so we cannot really effectively hone in on that further

	To make low rpm curves match, need to assume about a 5C
	battery discharge limit; which may not be unreasonable,
	though it is not explicitly exposed as a parameter in the grin simulator
	"""
	# FIXME: pretty sure I broke this somehow? or was it only workng on accident?
	# FIXME: seems grin simulator uses trapezoidal commutation? or is this an out of date comment?
	#  https://endless-sphere.com/sphere/threads/new-2017-updates-to-ebikes-ca-motor-simulator-to-try-out.89877/post-1333469
	inch = 20
	bike = define_ebike(kmh=45, inch=inch, turns=8)

	bike = bike.replace(
		battery__charge_state=0.99,
		# battery__cell__peak_discharge=6,	# need this to match
		battery__cell__peak_discharge=7,	# need this to match
		__bus__length=0,
		__controller__phase_current_limit=200,
		# __controller__modulation_factor=1,
		# __controller__power_limit=20000,
		__motor__coil_temperature=60,
	)
	bike.actuator.motor.electrical.attrs['d_1'] = 0

	Kv = bike.actuator.motor.electrical.Kv#/1.5*2 #* 0.9	# FIXME: this is in q frame; convert to ll? dont know what simulator expects
	R = bike.actuator.motor.electrical.R * 2 / 1.5
	L = bike.actuator.motor.electrical.L * 1000 * 2 / 1.5# *0.9
	d0 = bike.actuator.motor.electrical.d_0
	d1 = bike.actuator.motor.electrical.d_1	/ 60 # d1 internally not in rpm
	pp = bike.actuator.motor.geometry.pole_pairs
	A = bike.actuator.controller.phase_current_limit
	V = bike.actuator.controller.bus_voltage_limit
	CR = bike.actuator.controller.resistance
	BR = bike.battery.resistance
	Ah = bike.battery.Ah
	mass = bike.weight
	CdA = bike.load.CdA
	Cr = bike.load.Cr
	print()
	print(bike.actuator.motor.coil_temperature)
	print(f'https://ebikes.ca/tools/simulator.html?motor=cust_{Kv:.3f}_{R:.3f}_{L:.3f}_{pp}_{d0:.3f}_{d1:.5f}_0&batt=cust_75_{BR:.3f}_{Ah:.3f}&cont=cust_{V:.3f}_{A:.3f}_{CR}_V&wheel={inch}i&frame=cust_{CdA:.3f}_{Cr:.3f}&mass={mass:.3f}&hp=0&wind=0&autothrot=false&throt=100&grade=0')

	# bike.rear.motor.electrical.attrs['Kt'] *= 0.95
	# bike.rear.motor.electrical.attrs['L_co'] *= 0.66

	system_plot(bike, annotations='tdeoas')
