
import numpy as np

from pypowertrain.utils import *

def loop_ratios(l, r, t):
	l, r = l + t, r + t
	s = l + r
	return l / s, r / s

def Kv_from_Kt(Kt):
	"""Kv in V/rpm"""
	return 60 / (2 * np.pi) / Kt

def Kt_from_Kv(Kv):
	"""Kv in V/rpm"""
	return 60 / (2 * np.pi) / Kv


@dataclass
class Motor(Base):
	"""PMSM motor model"""

	# electrical
	Kt: float
	R: float
	Lq: float
	Ld: float

	# geometry
	poles: int
	slots: int

	radius: float	# gap radius
	stack_height: float
	tooth_depth: float

	# weight
	# FIXME: define as fractions?
	copper_weight: float
	magnet_weight: float
	iron_weight: float
	structure_weight: float

	drag_0: float = 3.5	# Nm / radius / kg_iron
	drag_1: float = 0.005	# intercept of the above, per rad/s
	eddy_em: float = 0

	# FIXME: its a conductivity not a resistance
	thermal_resistance: float = 6  # W/K/ m gap circumference; good value for closed ebike motor. 3 for botwheel, 10 for drone motor
	coil_temperature: float = 20
	magnet_temperature: float = 20

	turns: int = 5		# in case turns are unknown, not too crazy a default?
	magnet_height: float = 3e-3
	airgap: float = 0.8e-3
	magnet_fraction: float = 0.9
	iron_fill: float = 0.5  # tooth iron/copper ratio
	H_limit: float = 500	# At / mm of magnet

	inrunner: bool = False

	# cost (pm, iron, copper, structural)?

	@property
	def circumference(self):
		return self.radius * 2 * np.pi

	@property
	def pole_pairs(self):
		return self.poles / 2

	@property
	def weight(self):
		"""Total weight in kg"""
		return self.iron_weight + self.copper_weight + self.magnet_weight + self.structure_weight
	@property
	def Kv(self):
		"""Kv in V/rpm"""
		return Kv_from_Kt(self.Kt)

	@property
	def phase_current_limit(self):
		# # FIXME: EM sat limit?
		# EM_limit = 0.3	# keep EM as some fraction of PM field strength
		# reluctance = (self.airgap + self.magnet_height) / 1.26e-6
		# FIXME: need to work airgap into this also? should be conservative this way
		return self.H_limit * 1000 * self.magnet_height / self.turns

	def hysteresis_drag(self, omega):
		"""Dimensionless coefficients to Nm"""
		d = self.drag_0 * np.sign(omega) + omega * self.drag_1
		return self.radius * self.iron_weight * d

	@property
	def resistance(self):
		"""resistance, corrected for temperature"""
		scale = 1 + (self.coil_temperature - 20) * 0.393 / 100
		return self.R * scale

	@property
	def flux(self):
		"""flux, corrected for magnet temperature"""
		# FIXME: whats the temperature relation again? lost the reference... lose like 5% at high temp?
		temp_derate = 1 - (self.magnet_temperature - 20)/100 * 0.05
		return self.Kt / self.pole_pairs / 1.5 * temp_derate

	@property
	def tooth_slot_width(self):
		tooth_width = self.circumference / self.slots * self.iron_fill
		slot_width = self.circumference / self.slots * (1-self.iron_fill)
		return tooth_width, slot_width

	def rescale(self,
		length=1,	# stack length
		radius=1,	# radius
		turns=1,	# turns
		copper=1, 	# tooth depth
		magnet=1, 	# reluctance scaling; magnet+airgap depth, at constant flux
		# frequency=1,	# pole and slot scaling; how to add without breaking plots?

		# f=None,	# flux scaling; suspect, requires iron changes
		):
		"""recompute electrical and weight based on geometric scalings

		Can we add iron/copper ratio scaling? just push current density higher at equal saturation

		References
		----------
		https://www.researchgate.net/publication/283646083_Scaling_laws_for_synchronous_permanent_magnet_machines

		"""
		# FIXME: can we add in unwrapped pole scaling?
		# compute aspect ratios
		tooth_width, slot_width = self.tooth_slot_width
		co, ew = loop_ratios(self.stack_height, tooth_width, slot_width/2)
		ti, bi = loop_ratios(self.tooth_depth, slot_width, tooth_width/2)
		ra, ha = loop_ratios(self.radius*2, self.stack_height, 1e-2)

		area = radius * length
		volume = radius * area

		Kt = volume * turns
		Rew = turns**2 / radius
		Rco = turns**2 / radius**2 * length
		R = (Rco * co + Rew * ew) / copper
		Lco = turns**2 * length
		Lew = turns**2 * radius
		# FIXME: are we understanding this right? seems Lew is tiny in paper, like 1%
		#  thats for fairly long motor with low flux fringing though
		# L = (Lco * co + Lew * ew) / magnet
		L = Lco * 1 / magnet

		m_magnet = volume * magnet
		m_iron = volume	* bi + volume * ti * copper
		m_copper = (volume * co + radius * ew) * copper
		m_structure = ha * length + ra * radius**2

		# loss_scale = bi * volume + ti * volume * copper

		return self._replace(
			poles=self.poles,
			slots=self.slots,

			Kt=self.Kt * Kt,
			R=self.R * R,
			Lq=self.Lq * L,
			Ld=self.Ld * L,
			turns=self.turns * turns,

			radius=self.radius*radius,
			stack_height=self.stack_height*length,
			tooth_depth=self.tooth_depth*copper*radius,

			airgap=self.airgap*magnet*radius,
			magnet_height=self.magnet_height*magnet*radius,

			# hysteresis_drag=self.hysteresis_drag * loss_scale,
			# eddy_pm=self.eddy_pm * loss_scale,
			# eddy_em=self.eddy_em * loss_scale,

			iron_weight=self.iron_weight * m_iron,
			copper_weight=self.copper_weight * m_copper,
			magnet_weight=self.magnet_weight * m_magnet,
			structure_weight=self.structure_weight * m_structure
		)

	def plot(self, ax=None):
		assert self.inrunner is False		# FIXME: change this
		"""graphical representation of motor proportions"""
		import matplotlib.pyplot as plt
		from matplotlib.collections import LineCollection

		def rotate(angles, geo):
			c, s = np.cos(angles), np.sin(angles)
			r = np.array([[c, s], [-s, c]])
			return np.einsum('abr, ...a -> r...b', r, geo)

		def square(s, e):
			g = np.array([s, e])
			geo = [
				[g[0, 0], g[0, 1]],
				[g[0, 0], g[1, 1]],
				[g[1, 0], g[1, 1]],
				[g[1, 0], g[0, 1]],
				[g[0, 0], g[0, 1]]]
			return np.array(geo)

		def coil(s, e, n):
			g = np.array([s, e])
			p = np.linspace(g[0, 0], g[1, 0], n+1)
			b =np.ones(n)
			geo = [[p[1:], b*g[0, 1]], [p[:-1], b*g[1,1]]]
			return np.moveaxis(np.array(geo), 2, 0)

		def circle(r, n=360):
			a = np.linspace(0, np.pi*2, n+1, endpoint=True)
			a+=a[1]/2
			return np.array([np.cos(a), np.sin(a)]) * r

		if ax is None:
			fix, ax = plt.subplots(1, 1)
		plt.sca(ax)

		radius = self.radius
		poles = self.poles
		slots = self.slots

		tooth_width, slot_width = self.tooth_slot_width

		iron_depth = tooth_width / 2
		geometric_fill_factor = 0.8
		magnet_width = self.circumference / poles * self.magnet_fraction
		turns = int(self.turns)

		magnet = square(
			[radius + self.airgap + self.magnet_height, magnet_width/2],
			[radius + self.airgap, -magnet_width/2])
		geo = rotate(np.linspace(0, np.pi*2, poles, endpoint=False), magnet)
		line_collection = LineCollection(geo, linewidths=2, colors=['r', 'b']*(poles//2))
		ax.add_collection(line_collection)
		ax.plot(*circle(radius + self.airgap + self.magnet_height, poles))
		ax.plot(*circle(radius + self.airgap + self.magnet_height + iron_depth, poles))

		tooth = square(
			[radius, tooth_width/2],
			[radius - self.tooth_depth, -tooth_width/2])
		geo = rotate(np.linspace(0, np.pi*2, slots, endpoint=False), tooth)
		line_collection = LineCollection(geo, linewidths=2)
		ax.add_collection(line_collection)
		ax.plot(*circle(radius - self.tooth_depth))
		ax.plot(*circle(radius - self.tooth_depth - iron_depth))

		tip_width = tooth_width + slot_width / 2
		tip_depth = 1e-3	# FIXME: move rotor outward!
		tip = square(
			[radius, tip_width/2],
			[radius - tip_depth, -tip_width/2])
		geo = rotate(np.linspace(0, np.pi*2, slots, endpoint=False), tip)
		line_collection = LineCollection(geo, linewidths=2)
		ax.add_collection(line_collection)

		a = np.linspace(0, np.pi * 2, slots, endpoint=False)
		q = tooth_width + slot_width * geometric_fill_factor
		gcoil = coil(
			[radius - tip_depth, q / 2],
			[radius - self.tooth_depth, -q / 2],
			turns
		)
		colors = [
			["salmon", "darkmagenta", "green"],
			["lightsalmon", "magenta", "lightgreen"]
		]
		polarity, phase, wf, balance = winding(poles, slots)
		colors = np.array(colors)[polarity, phase]
		print('balance:', balance)

		# geo = rotate(a, gcoil)
		# fgeo = rotate(a, gcoil * [1, -1])	# windings going the other way
		# geo[polarity==1] = fgeo[polarity==1]
		geo = np.where(polarity, rotate(a, gcoil).T, rotate(a, gcoil * [1, -1]).T).T
		geo = np.reshape(geo, (-1, ) + geo.shape[-2:])
		colors = np.repeat(colors, turns)
		line_collection = LineCollection(geo, linewidths=2, color=colors)
		ax.add_collection(line_collection)

		plt.xlim(-radius*1.1, +radius*1.1)
		plt.ylim(-radius*1.1, +radius*1.1)
		plt.axis('equal')
		# plt.show()


def winding(poles, slots, phases=3):
	"""assign optimal windings, given poles and slots"""
	ca = np.linspace(0, np.pi * poles, slots, endpoint=False)
	cb = np.linspace(0, np.pi*2, 2*phases, endpoint=False) #+ 1e-6 #+ np.random.uniform(0, np.pi*2)
	cb += cb[0]/2
	# compute alignment score between each pole and candidate phase
	d = ca[:, None] + cb[None, :]
	alignment = np.cos(d)
	i = np.argmax(alignment, axis=1)
	balance1 = np.mean(np.sin(d[np.arange(slots), i])) < 1e-4
	foo = np.bincount(i%phases)
	balance2 = np.all(foo*phases == slots)
	polarity, phase = np.unravel_index(i, (2, phases))
	winding_factor = np.mean(np.max(alignment, axis=1))
	return polarity, phase, winding_factor, np.logical_and(balance1, balance2)
