
import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry


def Kv_from_Kt(Kt):
	"""Kv in V/rpm"""
	return 60 / (2 * np.pi) / Kt

def Kt_from_Kv(Kv):
	"""Kv in V/rpm"""
	return 60 / (2 * np.pi) / Kv

def Kt_from_phase_to_phase_Kv(Kv):
	"""Kv in V/rpm, V measured at peak, phase-to-phase

	This convention is common in drone type motors,
	but is not universal.
	"""
	return (3/2) / np.sqrt(3) * 60 / (2 * np.pi) / Kv


@dataclass
class Electrical(Scaled, Base):
	"""PMSM motor electromagnetic model

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

	# adding these scalings gives us nondim values nice and close to unity
	mu0: float = 1.256e-6
	R0: float = 1.68e-8

	@property
	def coil_resistance(self):
		"""Coil resistance per unit length"""
		return self.R0 / self.geometry.coil_area_fill * 4	# x4, since going back and forth through same space
	@property
	def gap_reluctance(self):
		return self.geometry.reluctance_length / self.mu0
	@property
	def slot_reluctance(self):
		return self.geometry.slot_width / self.mu0


	@staticmethod
	def from_absolute(
			geometry,
			Kt=None,	# Nm per Iq phase amp
			phase_to_phase_Kv=None,	# voltage maximum measured phase-to-phase, per rpm
			phase_to_phase_R=None,
			phase_to_phase_L=None,
			phase_to_neutral_R=None,
			phase_to_neutral_L=None,
			d0=None,
			d1=None,
			saturation_nm=None,	# limit of linear region in Nm
			saturation_amps=None,	# limit of linear region in amps
			demagnetization_amps=None,	# demag limit in the d-axis, in amps
			demagnetization_ratio=0.7,  # Iq/Id demag sensitivity difference

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

		if phase_to_phase_Kv:
			Kt = Kt_from_phase_to_phase_Kv(phase_to_phase_Kv)

		if salience:
			raise NotImplementedError

		scalings = {
			# electrical props
			'Kt': {'turns': 1, 'radius': 1, 'pm_flux_scale': 1},
			'R_co': {'turns': 2, 'length': 1, 'coil_resistance': 1, 'slots': 1},
			'R_ew': {'turns': 2, 'ew_length': 1, 'coil_resistance': 1, 'slots': 1},
			# FIXME: add phase wire lead fraction to resistance?
			'L_co': {'turns': 2, 'tooth_gap_area': 1, 'gap_reluctance': -1, 'slots': 1},
			'L_ew': {'turns': 2, 'tooth_area': 1, 'slot_reluctance': -1, 'slots': 1},

			# magnetic props
			'd_0': {'radius': 1, 'stator_volume': 1, 'iron_pm_field_scale': 1.8, 'poles': 1}, # NOTE: losses field dependence said to be between 1.6-2.0
			'd_1': {'radius': 1, 'stator_volume': 1, 'iron_pm_field_scale': 1.8, 'poles': 2}, # this scales with omega too; but we handle that in dedicated function now; omega not treated as state of motor
			'saturation': {'iron_em_field_scale': -1},	# A/T scaling
			'demagnetization': {'iron_em_field_scale': -1, 'iron_pm_field_scale': 1},  # interesting; gap length drops out. buffers EM, but also works against PM
		}

		# split given R and L in estimated coaxial and finite-length 'end winding' effects
		rco, rew = geometry.coil_ratios()
		lco, lew = geometry.flux_ratios()


		attrs = {
			'L_ew': L * lew,
			'L_co': L * lco,
			'R_ew': R * rew,
			'R_co': R * rco,
			'Kt': Kt,
			'salience_ratio': salience,
		}

		nondim_attrs = {}
		if d0 is not None:
			attrs['d_0'] = d0
			attrs['d_1'] = d1 * 60  # to rotor hz
		else:
			# NOTE: these nondim defaults are taken to coincide with grin data
			nondim_attrs['d_0'] = 454
			nondim_attrs['d_1'] = 0.66

		if saturation_nm is not None:
			attrs['saturation'] = saturation_nm / Kt
		elif saturation_amps is not None:
			attrs['saturation'] = saturation_amps
		else:
			# this value is proportional to amount of EM-induced field in the stator iron in T.
			# this seems to be a decent default
			nondim_attrs['saturation'] = 0.25

		if demagnetization_amps is not None:
			attrs['demagnetization'] = demagnetization_amps	# dimensional units in Id amps
		else:
			# this nondimensional value *is proportinal to* to extent to which we allow the PM field to be reversed
			# about halfway is a reasonable default for decent-quality neodymium at operating temperature
			# note that this nondimensional factor isnt calibrated to coincide with magnitude of PM field reversal exactly
			# the primary use is such that saturation limits set in amps scale correctly.
			# the proportionality constant between this number and actual field reversal has not been calibrated
			# FIXME: dial in this proportionality constant such that the number below is actual field reversal intensity
			# FIXME: add scaling of this value dependent of magnet temperature state?
			nondim_attrs['demagnetization'] = 0.5

		# NOTE: there are several papers out there [1] citing one another claiming pure Iq current
		# has no impact on demagnetization. In my opinion, none of these papers demonstrates any such thing.
		# Performing limited FEMM simulations, indeed it takes more Iq to reach a given maximum-field reversal
		# over all PM material; but Iq current is only marginally less demagnetizing than Id current.
		# This makes sense; while in theory Iq is 'orthogonal to' demagnetizing Id, in practice, for some rotor angles,
		# you will find partial but substantial overlap between opposing slots and poles of nonzero Iq.
		# but if you think Iq is irrelevant to demagnetization, set this value to zero
		# [1] https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/iet-pel.2018.6162
		attrs['demagnetization_ratio'] = demagnetization_ratio

		return (Electrical.init(geometry=geometry, scaling=scalings, context=['geometry']).
				from_dimensional(attrs).from_dimensionless(nondim_attrs))

	@property
	def Kv(self): # rpm/V
		"""Kv in q frame, in V/rpm"""
		return Kv_from_Kt(self.Kt)
	@property
	def Km(self):
		# FIXME: this is usualy done relative to phase-to-phase resistance? we have A-BC resistance
		return self.Kt / np.sqrt(self.R)

	@property
	def R(self):
		"""R in the q/d frame, such that dissipated power is P=I^2*R"""
		return self.R_ew + self.R_co
	@property
	def L(self):
		"""L in the q/d frame; dI/dt per V, in the circuit from phase A->BC"""
		return self.L_ew + self.L_co
	@property
	def Lq(self):
		return self.L / (1+self.salience_ratio)
	@property
	def Ld(self):
		return self.L * (1+self.salience_ratio)
	@property
	def salience(self):
		return self.Ld - self.Lq


	def iron_drag(self, omega):
		"""Iron drag in Nm for a given omega in mechanical Hz"""
		return self.d_0 + self.d_1 * np.abs(omega)

	def saturation_factor(self, amps):
		"""This saturation curve is based on the following rule of thumb:
		saturation-wise its generally possible [1] to get double the torque one gets at the end of the linear range,
		but obtaining that doubled torque requires some 3x the current, rather than 2x;
		that is the average Kt beyond the linear range is roughly cut in half
		Beyond that range things might get more nonlinear still;
		given the absence of empirical data on real motors in that regime,
		going deeper than 2x beyond the linear torque range should be taken with a particularly big grain of salt.

		[1] as judged by a few outrunners for which such data is available
		"""
		softmaxout = lambda x, y, h: np.log(np.exp(x * h) + np.exp(y * h)) / h
		return softmaxout(1, (np.abs(amps) / self.saturation - 1) / 4 + 1, 20)

	def demagnetiztion_factor(self, Iq, Id):
		"""If this factor is < 1, demagnetization should not occur"""
		demagnetization = self.demagnetization
		Id = Id / demagnetization
		Iq = Iq / demagnetization * self.demagnetization_ratio
		return -np.abs(Id) * Id + Iq * Iq


@dataclass
class Magnetic(Scaled, Base):
	# FIXME: is there value in breaking this out? only organizationally?
	def iron_drag(self, omega):
		pass

	def saturation_factor(self, amps):
		pass

	def demagnetiztion_factor(self, Iq, Id):
		pass