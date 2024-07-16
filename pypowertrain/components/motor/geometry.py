import numpy as np

from pypowertrain.utils import *


def loop_ratios(l, r, t):
	l, r = l + t, r + t
	s = l + r
	return l / s, r / s


@dataclass
class Geometry(Base):
	"""BLDC outrunner geometry logic"""
	# FIXME: should geometry itself be split into scaled attr dict?
	#  think its kindof overcomplication

	# absolute numbers
	poles: int
	slots: int

	# in meters
	gap_radius: float
	gap_length: float
	# FIXME: rather than radius, make slots and loop_length fundamental attribute
	#  gap_circumference = slots * loop_length
	#  gap_radius = gap_circumference / pi
	#  etc
	#  axial_scale an planar_scale?

	# FIXME: improve coherence; internally, prefer fractions or absolutes?

	slot_depth_fraction: float	# fraction of gap radius
	slot_width_fraction: float = 0.5  # tooth iron/copper ratio. or tooth_fraction?

	airgap: float = 0.8e-3
	coil_fill: float = 0.6	# FIXME: rename fraction?

	turns: int = 5		# in case turns are unknown, not too crazy a default?
	magnet_height: float = 3e-3
	magnet_width_fraction: float = 0.9	# fraction of back iron width filled by magnets

	structure_thickness: float = 3e-3	# outer shell and support structures
	inrunner: bool = False

	@staticmethod
	def create(
		turns=5,
		slots=None, slot_triplets=None,
		poles=None, pole_pairs=None,
		gap_radius=None, gap_diameter=None,
		gap_length=None, aspect_ratio=None,
		slot_depth=None, slot_depth_fraction=None,
		magnet_height=None, magnet_height_fraction=0.03,
		magnet_width=None, magnet_width_fraction=0.9,
		airgap=None, airgap_fraction=0.007,
		inrunner=False,
	):
		poles = poles or pole_pairs * 2
		slots = slots or slot_triplets * 3
		gap_radius = gap_radius or gap_diameter / 2
		gap_diameter = gap_radius * 2
		gap_circumference = gap_diameter * np.pi
		gap_length = gap_length or gap_diameter * aspect_ratio
		magnet_height = magnet_height or magnet_height_fraction * gap_radius
		magnet_width = magnet_width or magnet_width_fraction * gap_circumference / poles
		magnet_width_fraction = magnet_width / gap_circumference * poles
		slot_depth_fraction = slot_depth_fraction or slot_depth / gap_radius
		airgap = airgap or gap_radius * airgap_fraction

		return Geometry(
			slots=slots, poles=poles,
			gap_radius=gap_radius,
			gap_length=gap_length,
			magnet_height=magnet_height,
			slot_depth_fraction=slot_depth_fraction,
			magnet_width_fraction=magnet_width_fraction,
			airgap=airgap,
			turns=turns,
			inrunner=inrunner,
		)

	@property
	def radius(self):
		return self.gap_radius
	@property
	def length(self):
		return self.gap_length
	@property
	def aspect_ratio(self):
		return self.gap_diameter / self.gap_length

	@property
	def pole_pairs(self):
		return self.poles / 2

	@property
	def gap_diameter(self):
		return self.gap_radius * 2
	@property
	def gap_circumference(self):
		return self.gap_diameter * np.pi
	@property
	def gap_area(self):
		return self.gap_circumference * self.gap_length


	@property
	def slot_depth(self):
		return self.slot_depth_fraction * self.gap_radius
	@property
	def slot_width(self):
		return self.gap_circumference / self.slots * self.slot_width_fraction
	@property
	def tooth_width(self):
		return self.gap_circumference / self.slots * (1-self.slot_width_fraction)
	@property
	def magnet_width(self):
		return self.gap_circumference / self.poles * self.magnet_width_fraction


	@property
	def back_iron_thickness(self):
		return self.tooth_width / 2

	@property
	def outer_radius(self):
		rotor = self.airgap + self.magnet_height + self.back_iron_thickness
		sign = -1 if self.inrunner else +1
		return self.gap_radius + rotor * sign
	@property
	def inner_radius(self):
		stator = self.slot_depth + self.back_iron_thickness
		sign = -1 if self.inrunner else +1
		return self.gap_radius - stator * sign

	# these are rough geometry-estimated values for various volumes
	# the point of them isnt to be accurate; but to scale appropriately
	# specific motors can tune to their actual values with a dimensionless coefficient
	@property
	def tooth_volume(self):
		tooth = self.tooth_width
		A = tooth * self.slots * self.slot_depth
		return A * self.gap_length
	@property
	def stator_volume(self):
		b = self.back_iron_thickness * self.inner_radius * self.gap_length
		return b + self.tooth_volume
	@property
	def tooth_area(self):
		"""combined area of all teeth"""
		return self.tooth_width * self.slots * self.slot_depth
	@property
	def slot_area(self):
		"""combined area of all slots"""
		A = (self.gap_radius ** 2 - (self.gap_radius - self.slot_depth) ** 2) * np.pi
		return A - self.tooth_area
	@property
	def coil_contact_area(self):
		"""contact area of coils and stator"""
		return self.length * self.slot_depth * self.slots
	@property
	def coil_thickness(self):
		"""Average coil thickness as-wound around the tooth"""
		As = self.slot_area
		return As / self.slot_depth / 2 / self.slots
	@property
	def coil_volume(self):	# m^3
		"""Full coil volume, without any fill-factors applied"""
		As = self.slot_area
		t = self.coil_thickness
		L = (self.gap_length + t) * 2 + (self.tooth_width + t) * 2
		return L * As
	@property
	def coil_volume_fill(self):	# m^3
		"""Actual volume occupied by actual wires"""
		return self.coil_fill * self.coil_volume
	@property
	def coil_area_fill(self):	# m^3
		"""Actual area occupied by actual wires"""
		return self.slot_area * self.coil_fill
	@property
	def magnet_volume(self):
		return self.gap_area * self.magnet_height * self.magnet_width_fraction
	@property
	def back_volume(self):
		return self.back_iron_thickness * self.outer_radius * self.gap_length
	@property
	def rotor_volume(self):
		return self.back_volume + self.magnet_volume
	@property
	def structure_volume(self):
		# 2 outer plates, one interior hub plate?
		# FIXME: split this into seperate hub and outer shell logic
		return self.gap_radius**2*np.pi * self.structure_thickness * 3 + self.gap_area * self.structure_thickness
	@property
	def outer_length(self):
		overhang = self.slot_width / 2
		rest = overhang + self.airgap + self.structure_thickness
		return self.gap_length + rest * 2
	@property
	def reluctance_length(self):
		"""total amount of mu-0 reluctance in the circuit"""
		return self.airgap + self.magnet_height
	@property
	def iron_field(self):
		"""dimensionless iron flux density"""
		return self.pm_flux / self.tooth_width
	@property
	def pm_flux(self):
		"""pm flux scaling factor"""
		return self.magnet_height * self.magnet_width / self.reluctance_length
		# https://www.e-magnetica.pl/doku.php/flux_fringing
	@property
	def em_flux(self):
		"""em-flux per amp"""
		mu0 = 1e-6
		return self.turns / self.reluctance_length / mu0

	@property
	def side_area(self):
		"""side pate area"""
		return self.outer_radius ** 2 * np.pi

	def coil_ratios(self):
		"""fraction of coils in slots vs end-turns"""
		return loop_ratios(self.gap_length, self.tooth_width, self.slot_width/2)
	def flux_ratios(self):
		"""Fraction of EM-flux through airgap vs fringing at end-turns"""
		# FIXME: paper does not explain this, but should be about em flux through air gap,
		#  vs em flux through the end of the stack, arcing over the slots
		#  calibrated to match Lew in numerical example in paper
		return loop_ratios(
			self.gap_area / self.reluctance_length,
			self.tooth_area / self.slot_width / 4, 0)

	def rpm_to_electric_hz(self, rpm):
		return rpm / 60 * self.pole_pairs
	def rpm_to_electric_radians(self, rpm):
		return self.rpm_to_electric_hz(rpm) * 2 * np.pi

	# FIXME: turn these into independent setters?
	def rescale(self,
		length_scale=1,	# stack length
		radius_scale=1,	# radius
		turns_scale=1,	# turns
		slot_depth_scale=1, 	# tooth depth
		slot_width_scale=1, 	# tooth width
		reluctance_scale=1, 	# reluctance scaling; magnet+airgap depth, at constant flux
		turns=None,
		slot_depth=None,
		gap_radius=None,
		gap_length=None,
	):
		# FIXME: add slot/pole scaling? need to add them to electric scaling laws too
		return self.replace_norescale(
			gap_radius=gap_radius or self.gap_radius*radius_scale,
			gap_length=gap_length or self.gap_length*length_scale,
			airgap=self.airgap*radius_scale*reluctance_scale,
			magnet_height=self.magnet_height*radius_scale*reluctance_scale,
			turns=turns or self.turns*turns_scale,
			slot_depth_fraction=slot_depth/self.gap_radius if slot_depth else self.slot_depth_fraction*slot_depth_scale,
			slot_width_fraction=self.slot_width_fraction * slot_width_scale,
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
			show = True
			fix, ax = plt.subplots(1, 1)
		else:
			show = False
		plt.sca(ax)

		radius = self.gap_radius
		poles = self.poles
		slots = self.slots

		tooth_width, slot_width = self.tooth_width, self.slot_width
		tooth_depth = self.slot_depth
		iron_depth = self.back_iron_thickness
		geometric_fill_factor = 0.8
		magnet_width = self.magnet_width
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
			[radius - tooth_depth, -tooth_width/2])
		geo = rotate(np.linspace(0, np.pi*2, slots, endpoint=False), tooth)
		line_collection = LineCollection(geo, linewidths=2)
		ax.add_collection(line_collection)
		ax.plot(*circle(radius - tooth_depth))
		ax.plot(*circle(radius - tooth_depth - iron_depth))

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
			[radius - tooth_depth, -q / 2],
			turns
		)
		colors = [
			["salmon", "darkmagenta", "green"],
			["lightsalmon", "magenta", "lightgreen"]
		]
		polarity, phase, wf, balance = winding(poles, slots)
		colors = np.array(colors)[polarity, phase]

		geo = np.where(polarity, rotate(a, gcoil).T, rotate(a, gcoil * [1, -1]).T).T
		geo = np.reshape(geo, (-1, ) + geo.shape[-2:])
		colors = np.repeat(colors, turns)
		line_collection = LineCollection(geo, linewidths=2, color=colors)
		ax.add_collection(line_collection)

		plt.xlim(-radius*1.1, +radius*1.1)
		plt.ylim(-radius*1.1, +radius*1.1)
		plt.axis('equal')
		if show:
			plt.show()


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
