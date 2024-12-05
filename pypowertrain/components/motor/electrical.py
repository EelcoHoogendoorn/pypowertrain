
import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry


def Kv_from_Kt(Kt):
	"""Kv in rpm/V"""
	return 60 / (2 * np.pi) / Kt

def Kt_from_Kv(Kv):
	"""Kv in rpm/V"""
	return 60 / (2 * np.pi) / Kv

def rpm_to_radians(rpm):
	return rpm / 60 * (2*np.pi)
def radians_to_rpm(radians):
	return radians / (2*np.pi) * 60



ll_phase = {
	'star': {'R': 1 / 2, 'L': 1 / 2, 'Kv': np.sqrt(3)},
	'delta': {'R': 3 / 2, 'L': 3 / 2, 'Kv': 1},
}
phase_dq = {
	'star': {'R': 3 / 2, 'L': 3 / 2, 'Kt': 3 / 2},
	'delta': {'R': 1 / 2, 'L': 1 / 2, 'Kt': np.sqrt(3)/2},
}
# combined transform from line-to-line to dq does not depend on termination
ll_dq = {
	'R': 3/4, 'L': 3/4, 'Kt': np.sqrt(3)/2, 'Kv': 2/np.sqrt(3)
}
# FIXME: how to convert to star-equivalent amps? or phase-magnitude amps in the delta case?
L_M = {
	'star': {'A': 1},	# peak line amps * 1 = peak phase amps
	'delta': {'A': np.sqrt(3)/2},   #
}
# convert line to line quantities into dq frame
ll_to_star_dq = {'R': 3/4, 'L': 3/4, 'Kt': np.sqrt(3)/2}


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
	def sturns(self):
		"""Star-terminated-equivalent turns. Delta termination can be seen as star with sqrt(3) fewer turns"""
		d = {'star': 1, 'delta': 1 / np.sqrt(3)}
		return self.geometry.turns * d[self.geometry.termination]
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
	@property
	def pm_flux_scale(self):
		"""scaling factor proportional to total pm flux in the circuit"""
		return self.geometry.magnets_volume / self.geometry.reluctance_length
	@property
	def em_flux_scale(self):
		"""scaling factor proportional to em-flux in the circuit per phase amp"""
		tip_fraction = 0.9 # mu0-path is constricted by gap between teeth, so cant use full gap area # FIXME: configurable
		reluctance = self.geometry.reluctance_length / (self.geometry.gap_area*tip_fraction * self.mu0)
		mmf = self.sturns	# mmf per phase amp
		return mmf / reluctance
	@property
	def iron_pm_field_scale(self):
		"""scaling factor proportional to pm-induced stator iron field density"""
		return self.pm_flux_scale / self.geometry.stator_flux_area
	@property
	def iron_em_field_scale(self):
		"""scaling factor proportional to em-tesla in the stator iron per phase amp"""
		return self.em_flux_scale / self.geometry.stator_flux_area


	@staticmethod
	def from_absolute(
			geometry,
			Kv_ll=None,	# rpm per voltage-amplitude measured line-to-line
			R_ll=None,
			L_ll=None,
			Kv_phase=None,
			R_phase=None,
			L_phase=None,
			Kt_dq=None,	# Nm per 3phase amp
			d0=None,
			d1=None,
			saturation_nm=None,	# limit of linear region in Nm
			demagnetization_amps=None,	# demag limit in the d-axis, in amps
			demagnetization_ratio=0.7,  # Iq/Id demag sensitivity difference
			salience_ratio=0,
	):
		"""

		Notes
		-----
		All electrical variables are postfixed with the reference frame in which they are measured
		These are:
			ll: line-to-line; amplitude or peak value
			phase: properties of a single set of coils; between two termination endpoints
			dq: values in the Clark transformed frame
		Internally, values are stored in terms of phase quantities, as they are invariant to changes in termination
		Note these are physical coils; what sits between the terminations, not an equivalent after a mathematical transform

		Our clark transformed frames are defined as such:
			1 unit A_dq = ABC = [1, -1/2, -1/2]; the amplitude of the 3-phase line AC
			1 unit V_dq = ABC = [1, 0, 0]; a vertex of the switching hexagon
		Note that this implies a different constant k in front of the Clark transform
		of both current and voltage, k=2/3 and k=1 respectively. We feel these are the most natural units though.

		References
		----------
		The way in which the electrical parameters scale with geometrical parameters is inspired by [1]
		We parametrize things a little different in this code, and we implement more scaling laws,
		they cover only electrical parameters and we cover magnetic scaling laws too,
		but the basic ideas are the same.
			[1] https://www.researchgate.net/publication/283646083_Scaling_laws_for_synchronous_permanent_magnet_machines
		The simplest way of stating is, is that every empirical parameter is nondimensionalized,
		by the estimated magnitude of that parameter as derived from a highly simplified 1d-equivalent-circuit formula.
		That formula needs not be accurate in an absolute sense; but as long as it correctly captures the relevant scaling laws,
		our empirical values, including all real world higher order effects and manufacturing and material realities,
		should scale appropriately as well.

		"""
		# FIXME: for proper star/delta switching, properties now expressed in terms of line-amps,
		#  need to be stored internally as per-phase properties instead
		#  things like demagnetization and saturation currents need to be stored in per phase quantities
		#  should be the case they kick in at the same Nm of output no?
		#  or do they? note all else equal, delta has sqrt(3) higher line currents;
		#  and sqrt(3) lower phase-per-line currents, after rotating unit current by 30deg to find max over 3phase

		# convert to phase values
		if R_ll:
			R_phase = R_ll * ll_phase[geometry.termination]['R']
		if L_ll:
			L_phase = L_ll * ll_phase[geometry.termination]['L']
		if Kv_ll:
			Kv_phase = Kv_ll * ll_phase[geometry.termination]['Kv']	# rpm/V
			Kt_phase = Kt_from_Kv(Kv_phase)					# V / axle-rad/s, Nm per line-amp
		elif Kv_phase:
			Kt_phase = Kt_from_Kv(Kv_phase)
		elif Kt_dq:
			Kt_phase = Kt_dq / phase_dq[geometry.termination]['Kt']

		Kt_dq = Kt_phase * phase_dq[geometry.termination]['Kt']
		R_dq = R_phase * phase_dq[geometry.termination]['R']
		L_dq = L_phase * phase_dq[geometry.termination]['L']

		if salience_ratio:
			raise NotImplementedError

		scalings = {
			# electrical props
			'Kt_': {'sturns': 1, 'radius': 1, 'pm_flux_scale': 1},
			'R_co': {'sturns': 2, 'length': 1, 'coil_resistance': 1, 'slots': 1},
			'R_ew': {'sturns': 2, 'ew_length': 1, 'coil_resistance': 1, 'slots': 1},
			# FIXME: add phase wire lead fraction to resistance?
			'L_co': {'sturns': 2, 'tooth_gap_area': 1, 'gap_reluctance': -1, 'slots': 1},
			'L_ew': {'sturns': 2, 'tooth_area': 1, 'slot_reluctance': -1, 'slots': 1},

			# magnetic props
			'd_0': {'radius': 1, 'stator_volume': 1, 'iron_pm_field_scale': 1.8, 'poles': 1}, # NOTE: losses field dependence said to be between 1.6-2.0
			'd_1': {'radius': 1, 'stator_volume': 1, 'iron_pm_field_scale': 1.8, 'poles': 2}, # this scales with omega too; but we handle that in dedicated function now; omega not treated as state of motor
			'saturation': {'iron_em_field_scale': -1},	# A/T scaling
			'demagnetization': {'iron_em_field_scale': -1, 'iron_pm_field_scale': 1},  # interesting; gap length drops out. buffers EM, but also works against PM in equal amount
		}

		# split given R and L in estimated coaxial and finite-length 'end winding' effects
		rco, rew = geometry.coil_ratios()
		lco, lew = geometry.flux_ratios()


		attrs = {
			'L_ew': L_dq * lew,
			'L_co': L_dq * lco,
			'R_ew': R_dq * rew,
			'R_co': R_dq * rco,
			'Kt_': Kt_dq,
			'salience_ratio': salience_ratio,
		}

		# add magnetic properties
		nondim_attrs = {}
		if d0 is not None:
			attrs['d_0'] = d0
			attrs['d_1'] = d1 * 60  # from rpm to rotor hz
		else:
			# NOTE: these nondim defaults are taken to coincide with grin data
			nondim_attrs['d_0'] = 454
			nondim_attrs['d_1'] = 0.66

		if saturation_nm is not None:
			saturation_amps = saturation_nm / Kt_dq
			attrs['saturation'] = saturation_amps
		else:
			# this value is proportional to amount of EM-induced field in the stator iron in T.
			# this seems to be a decent default
			nondim_attrs['saturation'] = 0.25

		if demagnetization_amps is not None:
			attrs['demagnetization'] = demagnetization_amps	# dimensional units in Id amps
		else:
			# this nondimensional value *is proportinal to* to extent to which we allow the PM field to be reversed
			# about halfway is a reasonable default for decent-quality neodymium at 60C
			# note that this nondimensional factor isnt calibrated to coincide with magnitude of PM field reversal exactly
			# the primary use is such that saturation limits set in amps scale correctly.
			# the proportionality constant between this number and actual field reversal has not been calibrated
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
		"""Kv in dq frame, in V/rpm"""
		return Kv_from_Kt(self.Kt)
	@property
	def Km(self):
		"""Km in dq frame; Nm per unit power"""
		return self.Kt / np.sqrt(self.R)

	@property
	def Kt(self):
		"""Kt in dq frame"""
		return self.Kt_ #* phase_dq[self.geometry.termination]['Kt']
	@property
	def R(self):
		"""R in dq frame"""
		return (self.R_ew + self.R_co) #* phase_dq[self.geometry.termination]['R']
	@property
	def L(self):
		"""L in dq frame"""
		return (self.L_ew + self.L_co) #* phase_dq[self.geometry.termination]['L']
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


@dataclass
class Magnetic(Scaled, Base):
	# FIXME: is there value in breaking this out? only organizationally?
	def iron_drag(self, omega):
		pass

	def saturation_factor(self, amps):
		pass

	def demagnetiztion_factor(self, Iq, Id):
		pass

