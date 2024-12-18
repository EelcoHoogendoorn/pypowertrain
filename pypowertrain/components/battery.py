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

	def state_from_voltage(self, V):
		return (V - self.minimum_voltage) / (self.maximum_voltage - self.minimum_voltage)



@dataclass
class Battery(Base):
	# FIXME: make battery a subclass of cell; or share an interface, so they are arbitrarily nestable.
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
	def capacity(self): # in wh
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

	def charge_to_voltage(self, V):
		s = self.cell.state_from_voltage(V / self.S)
		assert 0 <= s <= 1
		return self.replace(charge_state=s)




samsung_21700 = Cell(
	maximum_voltage=4.2,
	nominal_voltage=3.6,
	minimum_voltage=3.0,	# more old fashioned safer value; gives 30% discharge kinda typical
	resistance=7e-3,    # 15A ~ 0.1V drop
	capacity=4.9,       # Ah
	peak_discharge=8,	# in units of C
	peak_charge=5,
	mass=70e-3,			# in kg
)

JGNE_lifepo4_26650 = Cell(
	maximum_voltage=3.65,
	nominal_voltage=3.2,
	minimum_voltage=2.5,
	resistance=20e-3,
	capacity=4.0,  # Ah
	peak_discharge=3,  # in units of ; this is peak continuous; 5s pulse about double?
	peak_charge=3,
	mass=88e-3,  # in kg
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
		S=1000, P=100,
	)


def define_battery(v, wh=1, cell=samsung_21700):
	"""define min voltage and min capacity battery"""
	S = np.ceil(v / cell.maximum_voltage)
	N = wh / (cell.capacity * cell.nominal_voltage)
	P = np.ceil(N / S)
	return Battery(cell=cell, S=S, P=P).charge_to_voltage(v)
