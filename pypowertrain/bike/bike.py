
from pypowertrain.system import *
from pypowertrain.utils import *

import scipy.optimize

root = lambda root, x0: scipy.optimize.root_scalar(root, x0=x0).root
root_b = lambda root, b: scipy.optimize.root_scalar(root, bracket=b, method='bisect').root


@dataclass
class BikeLoad(Load):
	CdA: float
	Cr: float
	nominal_kmh: float
	wheel_diameter: float
	structure_weight: float
	rider_weight: float = 80

	cog_height: float = 0.8
	cog_rear: float = 0.5
	cog_front: float = 0.5

	Cf: float = 0.6		# wet road

	front: bool = False		# only do identical motors for now
	rear: bool = True

	@property
	def n_motors(self):
		return (1 if self.front else 0) + (1 if self.rear else 0)

	@property
	def wheelbase(self):
		return self.cog_rear + self.cog_front

	@property
	def wheel_radius(self):
		return self.wheel_diameter / 2
	@property
	def wheel_circumference(self):
		return self.wheel_diameter * np.pi


	@property
	def weight(self):
		return self.rider_weight + self.structure_weight
	# @property
	# def inertia(self):
	# 	return self.weight

	def kph_to_rpm(self, kph):
		return kph * 1000 / 60 / self.wheel_circumference
	def rpm_to_kph(self, rpm):
		return rpm / 1000 * 60 * self.wheel_circumference



	def dash_tab(self):
		return dbc.Tab(label='Load', children=[
			html.Label('Weight (Kg)'),
			dcc.Slider(0, 100, 10, value=self.rider_weight, id='weight-slider'),
			html.Label('CdA'),
			dcc.Slider(0, 0.8, 0.05, value=self.CdA, id='cda-slider'),
			html.Label('Cr'),
			dcc.Slider(0, 0.01, 0.001, value=self.Cr, id='cr-slider'),
			html.Label('Wheel (inch)'),
			dcc.Slider(16, 28, 1, value=self.wheel_diameter/0.0256, id='wheel-slider'),

			html.Label('cog height (m)'),
			dcc.Slider(0, 1, 0.1, value=self.cog_height, id='cog-height-slider'),
			html.Label('cog rear (m)'),
			dcc.Slider(0, 1, 0.1, value=self.cog_rear, id='cog-rear-slider'),
			html.Label('cog front (m)'),
			dcc.Slider(0, 1, 0.1, value=self.cog_front, id='cog-front-slider'),
			html.Label('Cf'),
			dcc.Slider(0.1, 1.2, 0.1, value=self.Cf, id='cf-slider'),

			dcc.Checklist(['Front'], value=['Front'], id='front-check'),
			dcc.Checklist(['Rear'], value=['Rear'], id='rear-check'),
		])

	def dash_callback(self):
		@callback(
			Output('load', 'data'),

			Input('weight-slider', 'value'),
			Input('cda-slider', 'value'),
			Input('cr-slider', 'value'),
			Input('wheel-slider', 'value'),

			Input('cog-height-slider', 'value'),
			Input('cog-rear-slider', 'value'),
			Input('cog-front-slider', 'value'),
			Input('cf-slider', 'value'),

			Input('front-check', 'value'),
			Input('rear-check', 'value'),
		)
		def compute_handler_system(
				weight, CdA, Cr, wheel,
				cog_height, cog_rear, cog_front, cf,
				front, rear,
		):
			return pickle_encode(self.replace(
				CdA=CdA,
				Cr=Cr,
				rider_weight=weight,
				wheel_diameter=wheel*0.0256,
				cog_height=cog_height,
				cog_rear=cog_rear,
				cog_front=cog_front,
				Cf=cf,
				front='Front' in front,
				rear='Rear' in rear,
			))


@dataclass
class BikeSystem(System):

	@property
	def nominal_kinetic(self):
		# fixme rotational component!
		mps = self.load.nominal_kmh / 3.6
		return (1/2) * self.weight * mps**2

	def aero_drag(self, mps):
		rho = 1.225
		return (1 / 2) * rho * self.load.CdA * mps * np.abs(mps)

	def gravity_drag(self, grade):
		return self.downforce * np.sin(np.arctan(grade))

	def rolling_drag(self, mps):
		return self.load.Cr * self.downforce * np.sign(mps)

	def drag(self, rpm, grade=0):
		mps = self.load.rpm_to_kph(rpm) / 3.6
		return self.rolling_drag(mps) + self.aero_drag(mps) - self.gravity_drag(grade)

	@property
	def downforce(self):
		return self.weight * 9.81
	@property
	def rear_downforce(self):
		return self.downforce * self.load.cog_front / self.load.wheelbase
	@property
	def front_downforce(self):
		return self.downforce * self.load.cog_rear / self.load.wheelbase
	def shift(self, t_force):
		"""calc weight shift on each wheel as function of total traction force"""
		return (t_force * self.load.cog_height) / self.load.wheelbase
	def shifted_rear_downforce(self, t_force):
		return self.rear_downforce + self.shift(t_force)
	def shifted_front_downforce(self, t_force):
		return self.front_downforce - self.shift(t_force)

	@cached_property
	def stoppie(self):
		return root(self.shifted_rear_downforce, -self.downforce/2)
	@cached_property
	def wheelie(self):
		return root(self.shifted_front_downforce, self.downforce/2)

	@cached_property
	def traction_efficiency(self):
		q = self.downforce * 1.01
		F = np.linspace(-q, q, 100, endpoint=True)	# force applied at each wheel
		r = F * self.load.rear
		f = F * self.load.front
		return F, abs_traction(self, r, f) / (r+f)

	def nm_to_g(self, nm_per_motor):
		# fixme need to add rotational inertia to load
		return nm_per_motor * self.load.n_motors / self.load.wheel_radius / self.weight / 9.81

	def acceleration(self, rpm, torque):
		"""acceleration in G"""
		net_torque_per_motor = torque - self.drag(rpm) * self.load.wheel_radius / self.load.n_motors
		x, y = self.traction_efficiency
		eff = np.interp(net_torque_per_motor / self.load.wheel_radius, x, y)
		return self.nm_to_g(net_torque_per_motor * eff)

	def acceleration_lines(self):
		return '{:0.2f}G', np.linspace(-1, +1, 21, endpoint=True)
	@property
	def weight(self):
		return self.battery.weight + self.actuator.weight * self.load.n_motors + self.load.weight

	def temperatures(self, rpm, copper_loss, iron_loss, dt, key='coils'):
		# for a bike, rpm and forward free stream velocity are coupled
		mps = self.load.rpm_to_kph(rpm) / 3.6
		return self.actuator.temperatures(mps, rpm, copper_loss, iron_loss, dt, key)

	def x_axis(self, rpm=None):
		return 'kmh', self.load.rpm_to_kph(rpm)
	# FIXME: what is the most natural y axis? not sure Nm is intrinsically meaningful but somehow grown attached to it
	# def y_axis(self, nm=None):
	# 	"""Acceleration of the bike"""
	# 	return 'G', self.nm_to_g(nm)


def bike_stats(bike):
	print('total drag', bike.replace().system_drag(bike.nominal_kmh))
	print('rolling drag', bike.replace(CdA=0).system_drag(bike.nominal_kmh))
	print('aero drag', bike.replace(Cr=0).system_drag(bike.nominal_kmh))


def abs_traction(bike: BikeSystem, rearforce=None, frontforce=None):
	"""calculate traction forces with abs,
	taking into account motor limits, friction limits, and weight shift effects"""
	if frontforce is None: frontforce = rearforce
	if rearforce is None: rearforce = frontforce
	rearforce, frontforce = np.broadcast_arrays(rearforce, frontforce)

	def clip_mag(a, alim):
		"""cap magnitude"""
		alim = np.maximum(alim, 0)
		return np.clip(a, -alim, +alim)

	def virtual_abs(rf, ff):
		"""solve for total traction over both wheels"""
		rear_t_force = lambda t_force: clip_mag(rf, bike.shifted_rear_downforce(t_force) * bike.load.Cf)
		front_t_force = lambda t_force: clip_mag(ff, bike.shifted_front_downforce(t_force) * bike.load.Cf)
		balance = lambda t_force: rear_t_force(t_force) + front_t_force(t_force) - t_force
		return root(balance, rf+ff)
	force = [virtual_abs(r, f) for r, f in zip(rearforce, frontforce)]
	return np.clip(force, bike.stoppie, bike.wheelie)
