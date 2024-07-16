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
			'coils': {'coil_volume_fill': 1},
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


# @dataclass
# class Mass(Base):
# 	# FIXME: have different subclasses, or presets, for frameless, vs ebike vs drone?
# 	"""Motor mass model"""
# 	geometry: Geometry
#
# 	# FIXME: reframe as general scaling model
# 	_copper_weight: float
# 	_stator_weight: float
# 	_rotor_weight: float
# 	_structure_weight: float
#
# 	@staticmethod
# 	def from_absolute(
# 		geometry,
# 		weight,
# 		# dimensionless tuning factor relative to standard simple volume*density
# 		copper_weight=1,
# 		stator_weight=1,
# 		rotor_weight=1,
# 		structure_weight=1,
# 	):
# 		"""break down parameters into dimensionless parts"""
#
# 		sv, cv, rv, qv = geometry.stator_volume, geometry.coil_volume, geometry.rotor_volume, geometry.structure_volume
# 		sd, cd, rd, qd = iron_density * stator_weight, coil_density * copper_weight, iron_density*rotor_weight, alu_density*structure_weight
# 		sm, cm, rm, qm = sv * sd, cv * cd, rv * rd, qv * qd
# 		ms = sm + cm + rm + qm
# 		t = weight / ms
# 		sn, cn, rn, qn = sm*t/sv, cm*t/cv, rm*t/rv, qm*t/qv
#
# 		return Mass(
# 			geometry=geometry,
#
# 			_copper_weight=cn,
# 			_stator_weight=sn,
# 			_rotor_weight=rn,
# 			_structure_weight=qn,
# 		)
#
# 	@property
# 	def weight(self):
# 		return self.copper_weight + self.stator_weight + self.rotor_weight + self.structure_weight
# 	@property
# 	def copper_weight(self):
# 		return self._copper_weight * self.geometry.coil_volume
# 	@property
# 	def stator_weight(self):
# 		return self._stator_weight * self.geometry.stator_volume
# 	@property
# 	def rotor_weight(self):
# 		return self._rotor_weight * self.geometry.rotor_volume
# 	@property
# 	def structure_weight(self):
# 		return self._structure_weight * self.geometry.structure_volume
#
# 	@property
# 	def rotor_inertia(self):
# 		# FIXME: shell mass
# 		return self.geometry.outer_radius ** 2 * self.rotor_weight
