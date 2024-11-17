import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry
from pypowertrain.components.motor.mass import Mass
from pypowertrain.components.motor.electrical import Electrical
from pypowertrain.components.thermal import Thermal



@dataclass
class Motor(Base):
	"""PMSM motor model"""

	electrical: Electrical
	thermal: Thermal = None
	mass: Mass = None
	# aero FIXME: needed for high rpm accuracy?

	# FIXME: should we make these part of the thermal model?
	coil_temperature: float = 20
	magnet_temperature: float = 20
	H_limit: float = 500	# safe amp-turns per mm of magnet in circuit; should be a safe value even at 80c

	@property
	def geometry(self):
		return self.electrical.geometry

	@property
	def Kt(self):
		return self.electrical.Kt
	@property
	def R(self):
		return self.electrical.R
	@property
	def L(self):
		return self.electrical.L

	@property
	def phase_current_limit(self):
		# FIXME: EM sat limit?
		#  would be nice to have; even if arbitrary, allows us to weight scaling of reluctance and tooth width
		# EM_limit = 0.3	# keep EM as some fraction of PM field strength
		# reluctance = (self.airgap + self.magnet_height) / 1.26e-6
		# FIXME: need to work airgap into this also? should be conservative this way
		return self.H_limit * 1000 * self.electrical.geometry.magnet_height / self.electrical.geometry.turns

	def iron_drag(self, omega):
		"""Iron drag in Nm for a given omega in mechanical Hz"""
		# FIXME: add in iron temperature effects? losses go down by some 5% at temp
		# https://eprints.whiterose.ac.uk/146185/1/2018-11-06-TIA-Model%20Comparison-Revised-v3-Final.pdf
		return self.electrical.iron_drag(omega)

	@property
	def resistance(self):
		"""resistance, corrected for temperature"""
		scale = 1 + (self.coil_temperature - 20) * 0.393 / 100
		return self.R * scale

	# FIXME: rename to PM flux?
	@property
	def flux(self):
		"""flux, corrected for magnet temperature"""
		# FIXME: whats the temperature relation again? lost the reference... lose like 5% at high temp?
		#  seen some grin data saying this is conservative; nothing happening first 40c
		temp_derate = 1 - (self.magnet_temperature - 20)/100 * 0.05
		return self.Kt / self.electrical.geometry.pole_pairs / 1.5 * temp_derate


def from_defaults(
	radius,
	length,
	slots,
	poles,
	weight,
	turns=5,

):
	"""try and estimate motor props from dimensionless params as much as possible"""
	geometry = Geometry(
		slots=slots,
		poles=poles,
		gap_radius=radius,
		gap_length=length,
		slot_depth_fraction=0.1,	# FIXME: estimate from mass?
	)

