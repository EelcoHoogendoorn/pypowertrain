import numpy as np

from pypowertrain.utils import *


@dataclass
class Controller(Base):
	"""FOC controller for PM and hybrid-reluctance motors.

	Note: would be neat to add IM motor control support here
	"""
	commutation = {
		'sine': 0.75,
		# cos(30^2; double inscribed circle; once to get neutral zero polygon, second to get rotational symmetry
		'svpwm': np.sqrt(3) / 2,  # cos(30); inscribed circle in hexagon
		'thi': np.sqrt(3) / 2,  # cos(30)	# third harmonic
		'trapezoidal': np.sqrt(3) / 2,  # cos(30)	# works out like this?
	}

	phase_current_limit: float
	power_limit: float

	bus_voltage_limit: float

	internal_resistance: float
	# Rdson: float = 2e-3  # pretty typical fet resistance; tiny compared to motors we use
	# Rshunt: float = 1e-3  # lets assume phase shunts

	ripple_freq: float
	freq_limit: float	# e freq hz
	# FIXME: add dropdown options? also include trapezoidal modulation, plus associated harmonic losses?
	modulation_factor: float
	field_weakening: bool = True

	# some arbitrary defaults; often we dont care anyway
	weight: float = 40e-3
	length: float = 60e-3
	width: float = 40e-3
	height: float = 20e-3

	def dynamic_phase_current_limit(self, bus_voltage):
		# https://docs.odriverobotics.com/v/latest/hardware/s1-datasheet.html#id1
		# FIXME: unused still. implement limit on sum of bus voltage and phase current induced spikes instead?
		fet_rating = 80
		volts_per_amp = 0.5
		# fet_rating = bus_voltage + amps * volts_per_amp
		amps = (fet_rating - bus_voltage) / volts_per_amp
		return np.minimum(self.phase_current_limit, amps)

	@property
	def resistance(self):
		return self.internal_resistance
		# return self.Rdson + self.Rshunt


def define_ideal_controller():
	return Controller(
		phase_current_limit=10_000,
		power_limit=1_000_000,
		bus_voltage_limit=10_000,
		internal_resistance=0,
		ripple_freq=1e6,
		freq_limit=1e6,
		modulation_factor=1/np.sqrt(3),
	)
