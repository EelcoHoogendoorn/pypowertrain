
import numpy as np
from pypowertrain.utils import *
import scipy.sparse


# J / kg / K
copper_cp = 0.376e3
iron_cp = 0.45e3
alu_cp = 0.91e3
mag_cp = 1.05e3
urethane_cp = 2.0e3	# quite a broad range out there


@dataclass
class Thermal(Base):
	"""Thermal resistance network"""
	conductivity: dict
	capacity: dict
	# temperature: dict = defaultdict(default_factory=lambda :20)

	@cached_property
	def keys(self):
		return list(self.capacity.keys())
	@cached_property
	def key_pairs(self):
		return list(k.split('_')[:2] for k in self.conductivity.keys())
	@cached_property
	def idx(self):
		"""Assign a unique int to each named variable"""
		return {k: i for i, k in enumerate(self.keys)}

	@cached_property
	def IJ(self):
		"""Precompute indices for sparse conductivity matrix"""
		idx = self.idx
		q = np.array([[0,0,1,1], [0,1,0,1]]).T
		IJ = np.array([(idx[i], idx[j]) for i,j in self.key_pairs])
		return IJ[:, q].reshape(-1, 2).T
	def assemble_K(self):
		"""Assemble conductivity matrix"""
		k = [+1, -1, -1, +1]
		K = np.outer(list(self.conductivity.values()), k)
		return scipy.sparse.coo_matrix((K.flatten(), self.IJ)).todense()

	def solve(self, source: dict, dt):
		"""Solve (K+C/dt)*dT = q"""
		C = [self.capacity[n] for n in self.keys]
		Q = [source.get(n, 0) for n in self.keys]
		K = self.assemble_K()
		A = K + np.diag(C) / dt
		T = np.linalg.solve(A, Q)
		return {n: t for n, t in zip(self.keys, T)}



@dataclass
class Capacity(Scaled, Base):
	"""Just slap a rescaling cp around the mass model"""
	mass: "Mass"


@dataclass
class Conductivity(Scaled, Base):
	# FIXME: these classes are motor-specific functionaliy, really
	geometry: "Geometry"

	# velocities in m/s
	linear: float = 0			# free stream velocity
	circumferential: float = 0	# velocity at the airgap

	statorade: float = 1.0
	side_exposure: float = 1.0		# weighting factor; how much of the motor is exposed to free stream velocity
	rim_exposure: float = 1.0		# set to zero in case a tire is mounted directly on the motor, for instance
	vented: float = 0.0				# coupling between outside and inside ait
	potted: float = 1.0				# coil-stator potting
	emissivity: float = 1.0			# black body properties of materials

	@staticmethod
	def expand(scalings, attrs):
		"""Expand variable-length conductivity triplet into constant, circumference and linear vel dependent parts"""
		p = ('', '_r', '_v')
		q = {'': {}, '_r': {'circumferential': 1}, '_v': {'linear': 1}}
		attrs = {k+e: v for k, mv in attrs.items() for v, e in zip(mv, p)}
		# FIXME: creating unused scaling laws, need to zip with attrs
		scalings = {k+e: {**v, **q[e]} for k, v in scalings.items() for e in p}
		return scalings, attrs
