"""Several configurable motor thermal models"""

import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.thermal import *
from pypowertrain.components.motor.mass import *


copper_k = 401
iron_k = 70
si_steel_k = 28
alu_k = 180	# cast alloy?
mag_k = 100 # cast alloy
air_k = 2.88e-2 # @ 60C
ferrofluid_k = 0.45		# this is a bulk value


def radiative_k(ta, tb, eps=1.0):
	"""radiative conductivity, linearlized around a pair of temperatures in C"""
	ta, tb = ta + 273, tb + 273
	s = 5.6703e-8
	return s * eps * (ta ** 4 - tb ** 4) / (ta - tb)


def generic_capacity(mass: Mass):
	"""generic capacity model"""
	# FIXME add hub, axle, bearing?
	# FIXME: is there a good reason not to just fold this logic into mass model?
	attrs = {
		'coils': copper_cp * mass.coils,
		'stator': iron_cp * mass.stator,
		'rotor': iron_cp * mass.rotor,
		'shell': alu_cp * mass.shell,
		'inner': 1,  # 1 per gram
		'air': 1e9,  # inf reservoir
	}
	scalings = {
		'coils': {'coils': 1},
		'stator': {'stator': 1},
		'rotor': {'rotor': 1},
		'shell': {'shell': 1},
	}
	return Capacity.init(mass=mass, context=['mass'], scaling=scalings).from_dimensional(attrs=attrs)


def basic_conductivity(geometry, K0, Kc, Kl, linear, circumferential):
	# NOTE: triplets are constant, radial and linear velocity scaled components
	attrs = {
		'coils_stator': (10, ),	# generic big number; only seperate coils and stator because API calls for it
		'stator_air': (K0, K0+Kc, K0+Kl),
	}

	scalings = {
		'coils_stator': {'coils_contact_area': 1, 'coil_thickness': -1},	# FIXME: volume: 1, thickness: -2 instead?
		'stator_air': {'coils_contact_area': 1},
	}

	scalings, attrs = Conductivity.expand(scalings, attrs)
	return Conductivity.init(
		geometry=geometry, context=['geometry'], scaling=scalings,
		linear=linear, circumferential=circumferential,
	).from_dimensional(attrs)


def basic_thermal(mass, K0, Kc=0, Kl=0, linear=40, circumferential=20):
	"""basic thermal model; only need a K0 at minimum"""
	return Thermal(
		capacity=generic_capacity(mass),
		conductivity=basic_conductivity(
			mass.geometry, K0, Kc, Kl, linear, circumferential)
	)


class ShelledConductivity(Conductivity):
	"""Closed shell ebike style motor
	minimal flow directly over the coils.
	behavior is dominated by thermal resistance in and out of shell boundary layer
	"""

	@staticmethod
	def from_mixed(
			geometry,
			# statorade=False, vented=False, potted=False, painted=0.4, tire=False, block=0.25,
			statorade = 1.0,
			side_exposure = 1.0,		# weighting factor; how much of the motor is exposed to free stream velocity
			rim_exposure = 1.0,
			vented = 0.0,
			potted = 1.0,
			emissivity = 1.0,
	):
		"""Set from dimensionless attributes, so that it scales with motor design"""
		gap_radiation = radiative_k(80, 40)
		shell_radiation = radiative_k(80, 40) * 2	# 2 shells

		# https://www.engineeringtoolbox.com/convective-heat-transfer-d_430.html
		# empirical fits puts our convective curve slope much higher than above;
		# but above does not account for active stirring effects
		# https://ebikes.ca/documents/BionX_Statorade_Study.pdf
		h0, h1 = 25*2, 2.0*2

		# rim = 0 if tire else 0.5	# 0.5 since only one rim
		# rim = rim_exposure / 2
		# venting = 100 if vented else 0
		# w = 1 - block	# correction factor for blockage of free stream linear airflow
		f = 1.0		# flange factor; side plate is bigger than rotor side area
		# stator->shell / r goes from 2.5 to 3.5 0-50kph
		# shell->air r+v goes 4.5-14.5 0-50kph
		# https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/post-1095860
		# stator->shell not a function of speed with ff
		# https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/post-1113292
		# crystallite h staorade
		# https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/page-41
		# og grin thremal curves with statorade
		# https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/post-1116645
		# potting 6.5 vs 3.5c copper stator difference @ 150w
		# https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/post-1082371
		# from 4-15 for mux motor suggested here over relevant speed range; internal h quite different!
		# https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/post-1076622
		# post claims gap R proportional to length
		# https://endless-sphere.com/sphere/threads/definitive-tests-on-the-heating-and-cooling-of-hub-motors.48753/post-722948
		# two below and two above should add to about 3.0 at 200rpm

		# NOTE: triplets are constant, radial and linear velocity scaled components
		attrs = {
			# hard to model coil-stator contact ab initio; just tuned to typical temp diff in experiments
			'coils_stator_direct': (5,),
			'coils_stator_potting': (10,),
			# at low rpm and <1mm gaps, laminar flow and no dependence on speed
			'stator_rotor_air': (air_k,),
			# dont we need fill factor accounting at low speed? as long as rotating at all, full surface wetted by oil layer of ferro_k?
			'stator_rotor_ff': (ferrofluid_k, ferrofluid_k/15),	# presumed speed effect; doubling at 15m/s
			'stator_rotor_rad': (gap_radiation,),
			# stator surface area is smaller than shell, but its a standin for all internal structures, which face the shell with an area similar to the shell
			'stator_shell_rad': (shell_radiation,),
			# assumed that diffusion to the shell is mostly limited by iron path. seems to fit the data?
			'rotor_shell': (iron_k,),
			'inner_stator': (h0, h1),
			'inner_shell': (h0, h1*0.6),
			'shell_air': (h0*f, h1*f, h1*f),
			'rotor_air': (h0, h1, h1),
			'inner_air': (100, 5, 10),		# coupling between inner and outer air
		}

		scalings = {
			'coils_stator_direct': {'coils_contact_area': 1, 'coil_thickness': -1},
			'coils_stator_potting': {'coils_contact_area': 1, 'coil_thickness': -1, 'potted': 1},
			'stator_rotor_air': {'gap_area': 1, 'airgap': -1},  # laminar flow
			'stator_rotor_ff': {'gap_area': 1, 'airgap': -1, 'statorade': 1},
			'stator_rotor_rad': {'gap_area': 1, 'emissivity': 1},
			'stator_shell_rad': {'side_area': 1, 'emissivity': 1},
			'rotor_shell': {'rotor_volume': 1, 'length': -2},
			'inner_shell': {'side_area': 1},
			'inner_stator': {'side_area': 1},
			'shell_air': {'side_area': 1},
			'rotor_air': {'gap_area': 1, 'rim_exposure': 1},
			'inner_air': {'side_area': 1, 'vented': 1},
		}

		scalings, attrs = Conductivity.expand(scalings, attrs)

		# this scaling with side exposure is only intended to block free stream velocity
		scalings['inner_air_v']['side_exposure'] = 1
		scalings['rotor_air_v']['side_exposure'] = 1
		scalings['shell_air_v']['side_exposure'] = 1

		return ShelledConductivity.init(
			geometry=geometry, context=['geometry'], scaling=scalings,
			statorade=statorade,
			side_exposure=side_exposure,  # weighting factor; how much of the motor is exposed to free stream velocity
			rim_exposure=rim_exposure,
			vented=vented,
			potted=potted,
			emissivity=emissivity,

		).from_dimensionless(attrs)


def shelled_thermal(mass, **kwargs):
	return Thermal(
		capacity=generic_capacity(mass=mass),
		conductivity=ShelledConductivity.from_mixed(geometry=mass.geometry, **kwargs)
	)


class OpenConductivity(Conductivity):
	@staticmethod
	def from_foo(geometry, linear=10, circumferential=10, h=5):
		# NOTE: triplets are constant, radial and linear velocity scaled components
		attrs = {
			'coils_stator': (20,),
			'stator_rotor': (0.5,),
			'rotor_shell': (1.6,),
			'stator_air': (h / 2, 0, h),
			'coil_air': (h / 2, 0, h),
			'shell_air': (h / 2, h / 2, h),
			'rotor_air': (h / 2, h / 2, h),
		}

		scalings = {
			'coils_stator': {'coils_contact_area': 1, 'coil_thickness': -1},
			'stator_rotor': {'gap_area': 1, 'airgap': -1},  # smaller gap means more mixing shear energy injected
			'rotor_shell': {'gap_circumference': 1, 'back_iron_thickness': 1, 'length': -1},
			'stator_air': {'coils_contact_area': 1},
			'coil_air': {'coils_contact_area': 1},
			'shell_air': {'side_area': 1},
			'rotor_air': {'gap_area': 1},
		}

		scalings, attrs = Conductivity.expand(scalings, attrs)

		return Conductivity.init(
			geometry=geometry, context=['geometry'], scaling=scalings,
			linear=linear, circumferential=circumferential,
		).from_dimensional(attrs)


def open_thermal(mass, **kwargs):
	"""setup for drone type motors in open air"""
	return Thermal(
		capacity=generic_capacity(mass=mass),
		conductivity=OpenConductivity.from_foo(geometry=mass.geometry, **kwargs)
	)
