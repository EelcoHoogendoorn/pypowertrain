
from pypowertrain.utils import *


@dataclass
class Bus(Base):
	length: float
	area: float
	c: float = 1.68e-8
	@property
	def resistance(self):
		return self.c * self.length / self.area

	@property
	def weight(self):
		density = 8960
		return self.length * self.area * density