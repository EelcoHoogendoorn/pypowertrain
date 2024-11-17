from pypowertrain.system import *
from pypowertrain.components.battery import *
from pypowertrain.library import grin, moteus, vesc, odrive


def mod_lq(system, f):
	system = system.replace()
	system.actuator.motor.electrical.attrs['salience_ratio']=f
	system.actuator.motor.electrical.attrs['L_co']/=(1+f)
	system.actuator.motor.electrical.attrs['L_ew']/=(1+f)
	return system
def mod_ld(system, f):
	system = system.replace()
	system.actuator.motor.electrical.attrs['salience_ratio']=f
	system.actuator.motor.electrical.attrs['L_co']*=(1+f)
	system.actuator.motor.electrical.attrs['L_ew']*=(1+f)
	return system


def test_plot():
	system = System(
		battery=define_battery_75v(),
		actuator=grin.actuator(turns=8).replace(n_series=2),
	)
	print()
	print(system.actuator.motor.electrical.Ld)
	print(system.actuator.motor.electrical.Lq)
	# system = mod_lq(system, -0.4)	# high Lq; inset magnet
	# system = mod_lq(system, 0.6)	# low Lq; lobed rotor
	print(system.actuator.motor.electrical.salience)
	print(system.actuator.motor.electrical.Ld)
	print(system.actuator.motor.electrical.Lq)
	# return
	system_plot(system)
