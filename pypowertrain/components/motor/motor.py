
from pypowertrain.utils import *
from pypowertrain.components.motor.mass import Mass
from pypowertrain.components.motor.electrical import Electrical, ll_dq, Kv_from_Kt
from pypowertrain.components.thermal import Thermal



@dataclass
class Motor(Base):
	"""PMSM motor model"""

	electrical: Electrical
	thermal: Thermal = None
	mass: Mass = None
	# FIXME: aero model needed for high rpm accuracy?

	# FIXME: should we make these part of the thermal model?
	coil_temperature: float = 20
	magnet_temperature: float = 20

	@property
	def geometry(self):
		return self.electrical.geometry


	def iron_drag(self, omega):
		"""Iron drag in Nm for a given omega in mechanical Hz"""
		# FIXME: add in iron temperature effects? losses go down by some 5% at temp
		# https://eprints.whiterose.ac.uk/146185/1/2018-11-06-TIA-Model%20Comparison-Revised-v3-Final.pdf
		return self.electrical.iron_drag(omega)

	@property
	def R_dq(self):
		"""resistance, corrected for temperature"""
		# from 20-100c, copper gains 39% resistance
		temp_derate = 1 + (self.coil_temperature - 20) / (100-20) * 0.393
		return self.electrical.R * temp_derate
	@property
	def L_dq(self):
		return self.electrical.L
	@property
	def L_d(self):
		return self.electrical.Ld
	@property
	def L_q(self):
		return self.electrical.Lq
	@property
	def Kt_dq(self):
		"""Kt, corrected for magnet temperature"""
		# from 20 to 100c, we reversibly lose about 5% field strength (seen numbers around 8% too)
		temp_derate = 1 - (self.magnet_temperature - 20) / (100-20) * 0.05
		return self.electrical.Kt * temp_derate
	@property
	def Kv_dq(self):
		return Kv_from_Kt(self.Kt_dq)

	# temperature-corrected line-to-line properties
	@property
	def R_ll(self):
		return self.R_dq / ll_dq['R']
	@property
	def L_ll(self):
		return self.L_dq / ll_dq['L']
	@property
	def Kt_ll(self):
		return self.Kt_dq / ll_dq['Kt']
	@property
	def Kv_ll(self):
		return self.Kv_dq / ll_dq['Kv']

	def demagnetiztion_factor(self, Iq, Id):
		"""If this factor is < 1, demagnetization should not occur"""
		# from 60 to 120c, neodymium typically shift knee halfway to the right
		#  shift of the knee tends to be pretty linear
		temp_derate = 1 - (self.magnet_temperature - 60) / (120-60) * 0.5
		demagnetization = self.electrical.demagnetization * temp_derate
		Id = Id / demagnetization
		Iq = Iq / demagnetization * self.electrical.demagnetization_ratio
		return -np.abs(Id) * Id + Iq * Iq

