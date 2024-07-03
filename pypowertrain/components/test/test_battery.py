from pypowertrain.components.battery import *


def test_battery():
	battery = define_battery_58v()
	print(battery.cell.peak_charge_current)
	print(battery.peak_discharge_current)
	print(battery.peak_charge_power)
	print(battery.weight)
	print(battery.capacity)
	print(battery.capacity / battery.weight)
	print(battery.voltage)
	print(battery.resistance)
