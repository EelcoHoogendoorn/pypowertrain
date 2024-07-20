import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry


copper_density = 8960
coil_density = copper_density
iron_density = 7800
alu_density = 2700


@dataclass
class Mass(Scaled, Base):
	geometry: Geometry

	@staticmethod
	def infer(
			geometry: Geometry,
			total,
			# dimensionless tuning factor relative to default volume*density
			tuning={},
			cps={  # default cps for each component
					'coils': copper_density,
					'stator': iron_density,
					'rotor': iron_density,
					'shell': alu_density,
				}
	):
		scalings = {
			'coils': {'coils_volume_fill': 1},
			'stator': {'stator_volume': 1},
			'rotor': {'rotor_volume': 1},
			'shell': {'structure_volume': 1},
		}

		volumes = {k: getattr(geometry, list(v.keys())[0]) for k, v in scalings.items()}
		masses = {k: volumes[k] * cps[k] * tuning.get(k, 1) for k in volumes}
		normalizer = sum(masses.values()) / total
		# print('normalizer', normalizer)
		attrs = {k: masses[k] / normalizer for k in masses}
		return attrs, scalings

	# FIXME: make frameless preset
	@staticmethod
	def from_absolute(
			geometry: Geometry,
			total,
			# dimensionless tuning factor relative to default volume*density
			tuning={},
			cps={		# default cps for each component
				'coils': copper_density,
				'stator': iron_density,
				'rotor': iron_density,
				'shell': alu_density,
			}
		):
		"""Auto-initialized mass model, deducing weight distributions amongst components from the geometry model"""
		attrs, scalings = Mass.infer(geometry, total, tuning=tuning, cps=cps)

		return Mass.init(geometry=geometry, context=['geometry'], scaling=scalings).from_dimensional(attrs=attrs)

	@property
	def total(self):
		return sum(self.values())

	@property
	def rotor_inertia(self):
		# FIXME: shell mass
		return self.geometry.outer_radius ** 2 * self.rotor
