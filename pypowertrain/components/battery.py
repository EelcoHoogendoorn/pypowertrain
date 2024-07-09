import numpy as np
from pypowertrain.utils import *


@dataclasses.dataclass(frozen=True)
class Cell(Base):
	maximum_voltage: float
	nominal_voltage: float
	minimum_voltage: float
	resistance: float
	capacity: float	# Ah
	peak_discharge: float
	peak_charge: float
	mass: float

	@property
	def peak_discharge_current(self):
		return self.peak_discharge * self.capacity
	@property
	def peak_charge_current(self):
		return self.peak_charge * self.capacity

	def voltage(self, state):
		return self.minimum_voltage + (self.maximum_voltage - self.minimum_voltage) * state


@dataclass
class Battery(Base):
	cell: Cell
	S: int
	P: int
	charge_state: float = 1.0
	# FIXME: add temperature model to battery

	@property
	def weight(self):
		return self.S * self.P * self.cell.mass

	@property
	def voltage(self):
		return self.S * self.cell.voltage(self.charge_state)

	@property
	def resistance(self):
		return self.cell.resistance * self.S / self.P

	@property
	def Ah(self): # in Ah
		return self.P * self.cell.capacity
	@property
	def capacity(self): # in kwh
		return self.S * self.P * self.cell.capacity * self.cell.nominal_voltage

	@property
	def peak_discharge_current(self):
		return self.cell.peak_discharge_current * self.P
	@property
	def peak_charge_current(self):
		return self.cell.peak_charge_current * self.P

	@property
	def peak_discharge_power(self):
		return self.peak_discharge_current * self.voltage
	@property
	def peak_charge_power(self):
		return self.peak_charge_current * self.voltage


samsung_21700 = Cell(
	maximum_voltage=4.2,
	nominal_voltage=3.6,
	minimum_voltage=3.0,	# more old fashioned safer value; gives 30% discharge kinda typical
	# minimum_voltage=2.5,
	resistance=7e-3,    # 15A ~ 0.1V drop
	capacity=4.9,       # Ah
	peak_discharge=8,	# in units of C
	peak_charge=5,
	mass=70e-3,
)


def define_battery_58v(P=4):
	return Battery(
		cell=samsung_21700,
		S=14, P=P,
	)


def define_battery_75v(P=4):
	return Battery(
		cell=samsung_21700,
		S=18, P=P,
	)


def define_battery_ideal():
	"""define an effectively unlimited power source"""
	return Battery(
		cell=samsung_21700,
		S=1000, P=1000,
	)


def define_battery_limits(v, wh, cell=samsung_21700):
	"""define min voltage and min capacity battery"""
	S = np.ceil(v / cell.maximum_voltage)
	N = wh / (cell.capacity * cell.nominal_voltage)
	P = np.ceil(N / S)
	return Battery(cell=cell, S=S, P=P)
