
import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry


def Kv_from_Kt(Kt):
	"""Kv in V/rpm"""
	return 60 / (2 * np.pi) / Kt

def Kt_from_Kv(Kv):
	"""Kv in V/rpm"""
	return 60 / (2 * np.pi) / Kv


@dataclass
class Electrical(Scaled, Base):
	"""PMSM motor model

	The paradigm here is that we store all motor parameters as dimensionless quantities,
	which scaled by the relevant properties in the `geometry` object,
	give our dimensional quantities.

	This permits us to simply hot-swap the geometry object
	to attain a motor with appropriately scaled properties

	notes on drag vs magnet strength; effect is quite dramatic,
	so should be careful with anything but flux-neutral-reluctance-length scaling
	https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/page-39
	"""
	geometry: Geometry

	@staticmethod
	def from_absolute(
			geometry,
			Kt=None,
			Kv=None,
			phase_to_phase_R=None,
			phase_to_phase_L=None,
			phase_to_neutral_R=None,
			phase_to_neutral_L=None,
			d0=None,
			d1=None,
			salience=0):
		"""
		# FIXME: what about 'state', like velocities and temperatures? treat them via this mechanism or seperate?

		References
		----------
		We parametrize things a little different here, and we implement more scaling laws than mentioned here,
		but it covers many of the basic ideas
		https://www.researchgate.net/publication/283646083_Scaling_laws_for_synchronous_permanent_magnet_machines

		"""
		# convert R/L to q-d frame
		if phase_to_phase_R:
			# NOTE: this assumes star-termination
			phase_to_neutral_R = phase_to_phase_R / 2
		if phase_to_phase_L:
			phase_to_neutral_L = phase_to_phase_L / 2
		if phase_to_neutral_R:
			R = phase_to_neutral_R * 1.5
		if phase_to_neutral_L:
			L = phase_to_neutral_L * 1.5

		if Kv:
			Kt = Kt_from_Kv(Kv)

		if salience:
			raise NotImplementedError

		scalings = {
			'R_co': {'turns': 2, 'length': 1, 'coil_area_fill': -1},
			'R_ew': {'turns': 2, 'tooth_width': 1, 'coil_area_fill': -1},
			'L_co': {'turns': 2, 'gap_area': 1, 'reluctance_length': -1},
			'L_ew': {'turns': 2, 'tooth_area': 1, 'slot_width': -1},  # FIXME: 'slots': 1
			'Kt': {'turns': 1, 'radius': 2, 'length': 1, 'pm_flux': 1},
			'd_0': {'radius': 1, 'stator_volume': 1, 'iron_field': 1.8}, # NOTE: losses field dependence said to be between 1.6-2.0
			'd_1': {'radius': 1, 'stator_volume': 1, 'iron_field': 1.8},#, 'omega': 1},  # FIXME: 'poles': 1
			# FIXME: define as multiplier on L_co?
			# 'salience': {'turns': 2, 'radius': 1, 'reluctance_length': -1},	# same as Lco; no var relux at end of stack
		}

		# split R and L in estimated coaxial and finite-length 'end winding' effects
		rco, rew = geometry.coil_ratios()
		lco, lew = geometry.flux_ratios()


		attrs = {
			'L_ew': L * lew,
			'L_co': L * lco,
			'R_ew': R * rew,
			'R_co': R * rco,
			'Kt': Kt,
			'salience': salience,
		}

		nondim_attrs = {}
		if d0 is None:
			# NOTE: these defaults are taken from grin data
			nondim_attrs['d_0'] = 19e3
			nondim_attrs['d_1'] = nondim_attrs['d_0'] / 370
		else:
			attrs['d_0'] = d0
			attrs['d_1'] = geometry.rpm_to_electric_radians(d1)


		return (Electrical.init(geometry=geometry, scaling=scalings, context=['geometry']).
				from_dimensional(attrs).from_dimensionless(nondim_attrs))
		# return Electrical.from_unscaled(geometry=geometry, scaling=scalings, context=['geometry'], attrs=attrs)

	@property
	def Kv(self): # rpm/V
		return Kv_from_Kt(self.Kt)
	@property
	def Km(self):
		# FIXME: this is usualy done relative to phase-to-phase resistance? we have A-BC resistance
		return self.Kt / np.sqrt(self.R)
	# FIXME: in attr getting, get all keys stripped of suffix and sum automatically?
	@property
	def R(self):
		return self.R_ew + self.R_co
	@property
	def L(self):
		return self.L_ew + self.L_co
	@property
	def Lq(self):
		return self.L + self.salience
	@property
	def Ld(self):
		return self.L - self.salience
	def iron_drag(self, omega):
		return self.d_0 + self.d_1 * omega
