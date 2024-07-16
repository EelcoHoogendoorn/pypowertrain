from pypowertrain.bike.bike_models import *


def test_ebike():
	"""Test for congruence with grin simulator

	Agrees with grin simulator in broad strokes.

	Load line and high speed limit seems to agree;
	simulator does not seem to do field weakening,
	and motor constants as currently used to appear correct,
	for the high speed low torque setting.

	To make low rpm curves match, need to assume about a 5C
	battery discharge limit; which may not be unreasonable,
	though it is not explicitly exposed as a parameter in the grin simulator
	"""
	# FIXME: pretty sure I broke this somehow? or was it only workng on accident?
	inch = 20
	bike = define_ebike(kmh=45, inch=inch, turns=5)

	bike = bike.replace(
		battery__charge_state=0.99,
		battery__cell__peak_discharge=6,	# need this to match
		rear__bus__length=0,
		rear__controller__phase_current_limit=200,
		# rear__controller__power_limit=20000,
		rear__motor__coil_temperature=60,
	)
	bike.rear.motor.electrical.attrs['d_1'] = 0

	Kv = bike.rear.motor.electrical.Kv #* 0.9
	R = bike.rear.motor.electrical.R / 1.5 * 2
	L = bike.rear.motor.electrical.L * 1000 / 1.5 * 2# *0.9
	d0 = bike.rear.motor.electrical.d_0
	d1 = bike.rear.motor.electrical.d_1	# FIXME: d1 internally not in rpm!
	pp = bike.rear.motor.geometry.pole_pairs
	A = bike.rear.controller.phase_current_limit
	V = bike.rear.controller.bus_voltage_limit
	CR = bike.rear.controller.resistance
	BR = bike.battery.resistance
	Ah = bike.battery.Ah
	mass = bike.weight
	CdA = bike.CdA
	Cr = bike.Cr
	print()
	print(bike.rear.motor.coil_temperature)
	print(f'https://ebikes.ca/tools/simulator.html?motor=cust_{Kv:.3f}_{R:.3f}_{L:.3f}_{pp}_{d0:.3f}_{d1:.5f}_0&batt=cust_75_{BR:.3f}_{Ah:.3f}&cont=cust_{V:.3f}_{A:.3f}_{CR}_V&wheel={inch}i&frame=cust_{CdA:.3f}_{Cr:.3f}&mass={mass:.3f}&hp=0&wind=0&autothrot=false&throt=100&grade=0')

	# bike_plot(bike)
