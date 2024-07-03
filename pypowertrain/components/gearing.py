import numpy as np

from pypowertrain.utils import *


@dataclass
class Gearing(Base):
	"""Very basic, could use some work"""
	ratio: float
	efficiency: float
	torque_limit: float

	# geometry
	weight: float
	thickness: float


	# def rescale(self, thickness=None, ratio=None):
	# 	# FIXME: add scaling laws here?
	# 	#  scale peak torque with thickness, quadratc in radius?
	# 	#  ratio changes should prob affect efficiency
	# 	#  makes sense to do so... hard to model gearboxes ab-initio
	# 	quit()
	# 	thickness = thickness or self.thickness
	# 	ratio = ratio or self.ratio
	# 	thickness_ratio = thickness / self.thickness
	# 	return self._replace(
	# 		thickness=thickness,
	# 		weight=self.weight * thickness_ratio,
	# 		# FIXME: scale with ratio
	# 		torque_limit=self.torque_limit * thickness_ratio,
	# 		ratio=ratio,
	# 	)

	def forward(self, rpm, torque):
		"""map motor conditions to output conditions"""
		e = 1#np.where(rpm*torque>0, self.efficiency, 1/self.efficiency)
		return rpm/self.ratio, torque*self.ratio*e
	def backward(self, rpm, torque):
		"""map output conditions to motor conditions"""
		e = 1#np.where(rpm*torque>0, self.efficiency, 1/self.efficiency)
		return rpm*self.ratio, torque/(self.ratio*e)


def define_direct():
	"""direct drive default for code simplification"""
	return Gearing(ratio=1, efficiency=1, torque_limit=1e9, weight=0, thickness=0)
