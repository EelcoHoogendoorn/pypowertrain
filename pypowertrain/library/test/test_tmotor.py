from pypowertrain.library.tmotor import *
from pypowertrain.components.actuator import Actuator
from pypowertrain.components.battery import define_battery
from pypowertrain.library import moteus
from pypowertrain.system import System, system_plot


def test_tmotor_u8():
	""""""
	print()
	# https://de.eco-worthy.com/products/lifepo4-12v-50ah-lithium-eisen-phosphat-batterie
	lifepo = define_battery(v=12.8*1, wh=460*2).replace(__peak_discharge=1.5, __peak_charge=1.5)
	system = System(
		actuator=Actuator(
			motor=tmotor_u8().replace(__turns_scale=150 / 90),
			# controller=odrive.pro().replace(
			controller=moteus.r4().replace(
			# bus_voltage_limit=48,
				field_weakening=True,
				# freq_limit=2400,
			),
		),
		battery=lifepo,
	)
	print(system.actuator.motor.electrical.Kv)
	# print(system.battery.peak_discharge_current)
	# return
	print(system.actuator.motor.mass.total)
	# system.actuator.plot()
	system_plot(system)#, output='s1p2.png')
