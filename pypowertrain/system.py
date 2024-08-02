import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.actuator import Actuator
from pypowertrain.components.battery import Battery



from dash import html, dcc, callback, Output, Input
import dash_bootstrap_components as dbc


@dataclass
class Load(Base):

	def load(self, rpm):
		raise NotImplementedError
	@property
	def weight(self):
		raise NotImplementedError
	def dash_tab(self):
		raise NotImplementedError
	def dash_callback(self):
		raise NotImplementedError


@dataclass
class DummyLoad(Load):

	drag: float = 2e-3
	inertia: float = 1.0

	def load(self, rpm):
		# by default, lets have only a light aero drag on the output shaft
		f = rpm * self.drag
		return f * np.abs(f)
	@property
	def weight(self):
		return 0

	def dash_tab(self):
		return dbc.Tab(label='Load', children=[
			html.Label('Inertia (Kg m^2)'),
			dcc.Slider(0, 10, 1, value=self.inertia, id='inertia-slider'),
			html.Label('Drag (Nm / (rad/s))'),
			dcc.Slider(0, 0.01, 0.001, value=self.drag, id='drag-slider'),
		])

	def dash_callback(self):
		@callback(
			Output('load', 'data'),

			Input('inertia-slider', 'value'),
			Input('drag-slider', 'value'),
		)
		def compute_handler_system(
				inertia, drag,
		):
			return pickle_encode(self.replace(
				drag=drag, inertia=inertia
			))


@dataclass
class System(Base):
	"""Simple battery-controller-motor system"""
	battery: Battery
	actuator: Actuator
	load: Load = DummyLoad()

	@property
	def weight(self):
		return self.battery.weight + self.actuator.weight + self.load.weight

	@property
	def inertia(self):
		# FIXME: move to actuator?
		# gear-reflected inertia
		actuator_inertia = self.actuator.motor.mass.rotor_inertia * self.actuator.gearing.ratio ** 2
		return actuator_inertia + self.load.inertia


	def acceleration(self, rpm, torque):
		net = torque - self.load.load(rpm)
		return net / self.inertia
	def acceleration_lines(self):
		return '{:0.2f} rad/s^2', [-10, 0, +10]

	def x_axis(self, rpm=0):
		return 'rpm', rpm
	def y_axis(self, nm=0):
		return 'Nm', nm
	def x_axis_forward(self, rpm):
		return rpm
	def y_axis_forward(self, nm):
		return nm
	def x_axis_inverse(self, rpm):
		return rpm
	def y_axis_inverse(self, nm):
		return nm

	def temperatures(self, rpm, copper_loss, iron_loss, dt, key='coils'):
		mps = rpm * 0	# by default, no link between rpm and free stream velocity
		return self.actuator.temperatures(mps, rpm, copper_loss, iron_loss, dt, key)


def system_limits(
	system: System,
	trange, rpm,
	gridsize=500
):
	"""
	Find powertrain operating points,
	that attain a given torque, with the least energy consumption,
	where possible, given the various physical constraints along the powertrain

	Parameters
	----------
	system: System
	trange: output torque range
	rpm: output rpm range
	gridsize: number of Id points to search over

	Returns
	-------
	List of graphs

	References
	-------
	https://nl.mathworks.com/help/mcb/gs/pmsm-constraint-curves-and-their-application.html#PMSMConstraintCurvesAndTheirApplicationExample-6
	"""
	actuator = system.actuator
	battery = system.battery
	motor = actuator.motor
	controller = actuator.controller

	if controller.field_weakening:
		arange = np.linspace(-actuator.phase_current_limit, +actuator.phase_current_limit, gridsize+1)
	else:
		arange = np.array([0.])	# always pursue Id=0

	# map output ranges to motor ranges
	# FIXME: gearing efficiency makes torque function of rpm, through sign alone.
	#  need to move this into process loop below to make that work
	#  alternatively; dont paramerize in terms of output torque? stick with EM torque?
	rpm, trange = actuator.gearing.backward(rpm, trange)

	salience = motor.electrical.salience # -(motor.Lq - motor.Ld)
	Id, em_torque = np.meshgrid(arange, trange)
	Iq = em_torque / (3/2) / motor.geometry.pole_pairs / (motor.flux + Id * salience)
	# FIXME: encode saturation relation; torque goes down with excessive (pm_flux - id)**2+iq**2
	#  that is, we might use FW at high torque to combat saturation

	# should ripple count towards phase current limits? i guess so conservatively.
	Is = Id**2 + Iq**2 + actuator.ripple_current**2

	bus_resistance = actuator.bus.resistance + battery.resistance
	Rt = actuator.phase_resistance


	Vlim = lambda omega: (Id*Rt - omega*motor.electrical.Lq*Iq)**2 + (Iq*Rt + omega*motor.electrical.Ld*Id + omega*motor.flux)**2
	def process_omega(omega_axle_hz):
		"""construct and intersect amp and V limits for a given omega"""
		omega_elec_hz = omega_axle_hz * motor.geometry.pole_pairs
		omega_axle_rad = omega_axle_hz * 2 * np.pi
		omega_elec_rad = omega_elec_hz * 2 * np.pi

		# FIXME: work Id-FW-dependence into iron drag? Id division should be about equal to demag current limit
		#  effect seems quite minimal in practice; like 3% kph continuous rating
		drag_torque = np.sign(omega_axle_hz) * motor.iron_drag(omega_axle_hz) #* (1+Id/300)**2
		copper_loss = Is*Rt
		iron_loss = omega_axle_rad * drag_torque + omega_elec_rad**2*Is*0
		dissipation = copper_loss + iron_loss
		mechanical_torque = em_torque - drag_torque
		mechanical_power = mechanical_torque * omega_axle_rad
		bus_power = dissipation + mechanical_power

		bus_current = bus_power / battery.voltage	# FIXME: solve bus voltage sag properly; get wonky results right now for high resistsances
		effective_bus_voltage = battery.voltage - bus_current * bus_resistance
		effective_voltage = actuator.effective_voltage(effective_bus_voltage)

		mask = np.abs(omega_elec_hz) < controller.freq_limit
		mask = np.logical_and(mask, Vlim(omega_elec_rad) < effective_voltage ** 2)
		mask = np.logical_and(mask, Is < actuator.phase_current_limit**2)
		# cap bus power
		mask = np.logical_and(mask, bus_power < battery.peak_discharge_power)
		mask = np.logical_and(mask, bus_power > -battery.peak_charge_power)
		# mech power here not appropriate; wrong sign of dissipation.
		# yet we are missing something here no? caps will dissipate
		# when switching large current zero bus regen braking
		mask = np.logical_and(mask, np.abs(bus_power) < actuator.power_limit)

		# of all valid Id options for a given em_torque, we pick the one most favorable to battery
		ma = np.ma.array(bus_power, mask=1-mask)
		# this is a reduction over Id
		mask = np.ma.min(ma, axis=1).mask
		idx = np.ma.argmin(ma, axis=1)
		i = np.arange(len(trange))

		# map to output torque
		_, mechanical_torque = actuator.gearing.forward(0, mechanical_torque)

		return [
			np.ma.array(o[i, idx], mask=mask).filled(np.nan)
			for o in [copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, mechanical_torque]
		]
	# good old for loop rather than vectorize over rpm; memory use might explode otherwise
	outputs = np.array([process_omega(o) for o in rpm/60])
	return np.moveaxis(outputs, [1, 2, 0], [0, 1, 2])


def round(x, digits):
	"""round to a number of significant digits"""
	shift = 10**int(np.log10(x) - digits + 1)
	return np.ceil(x / shift) * shift


def system_detect_limits(system, fw=1.3, frac=0.95, padding=1.1):
	"""auto-detect some reasonably tight limits on the actuator system"""
	max_torque = system.actuator.peak_torque * 1.1
	max_rpm = system.actuator.motor.electrical.Kv * system.battery.voltage * fw * system.actuator.n_series
	trange = np.linspace(-max_torque, +max_torque, 20, endpoint=True)
	rpm = np.linspace(0, max_rpm, 20, endpoint=False)
	_, _, _, _, _, _, torque = system_limits(system, trange, rpm)
	max_torque = np.max(np.abs(np.nan_to_num(torque)))
	max_rpm = rpm[::-1][np.argmin(np.mean(np.isnan(torque), axis=0)[::-1] > frac)]
	max_rpm = system.x_axis_inverse(round(system.x_axis_forward(max_rpm) * padding, 2))
	max_torque = system.y_axis_inverse(round(system.y_axis_forward(max_torque) * padding, 2))
	return max_rpm, max_torque


def system_plot(
	system: System,
	n_rpm=200,
	n_torque=500,
	max_rpm=None,
	max_torque=None,
	targets=None,
):
	"""mpl plots of system limits and properties"""
	import matplotlib.pyplot as plt

	# setup calculation grids
	if max_torque is None:
		max_rpm, max_torque = system_detect_limits(system)

	rpm_range = np.linspace(0, max_rpm, n_rpm+1, endpoint=True)
	torque_range = np.linspace(-max_torque, +max_torque, n_torque + 1, endpoint=True)

	# eval the system performance graphs
	copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = system_limits(system, torque_range, rpm_range)
	dissipation = copper_loss + iron_loss
	efficiency = 1 - dissipation / np.maximum(np.abs(mechanical_power), np.abs(bus_power))

	acceleration = system.acceleration(system.x_axis_forward(rpm_range), torque)

	# construct thermal curves
	thermal_specs = [
		{'color': 'yellow', 'dt': 5000, 'dT': 60},
		{'color': 'orange', 'dt': 60, 'dT': 60},
		{'color': 'red', 'dt': 2, 'dT': 40},
	]
	thermals = {
		s['color']: system.temperatures(
			rpm_range, copper_loss, iron_loss, dt=s['dt']) - s['dT']
		for s in thermal_specs
	}

	x, X = system.x_axis(rpm_range)
	y, Y = system.y_axis(torque_range)

	def imshow(im, **kwargs):
		a = plt.pcolormesh(X, Y, im, mouseover=True, **kwargs)
		plt.colorbar()
		return a
	def contour(im, **kwargs):
		return plt.contour(X, Y, im, **kwargs)

	def default_annotate():
		# delineate FW-region
		contour(Id, levels=[-1], colors='white')
		# plot x axis
		plt.plot(X, X * 0, c='gray')
		# plot thermal limits
		for c, m in thermals.items():
			contour(m, levels=[0], colors=c)

		# plot acceleration load lines
		amin, amax = np.nan_to_num(acceleration).min(), np.nan_to_num(acceleration).max()
		aname, lines = system.acceleration_lines()
		for a in lines:
			if np.all(a > amin+1e-3) and np.all(a < amax-1e-3):
				contour(acceleration, levels=[a], colors='black')
		contour(acceleration, levels=[amin + 5e-4], colors='gray')
		contour(acceleration, levels=[amax - 5e-4], colors='gray')

		contour(efficiency, levels=[0.9], colors='green')

		if targets is not None:
			t_torque, t_rpm, t_dissipation, t_weight = [np.array(t) for t in targets]
			plt.scatter(system.x_axis(t_rpm)[1], system.y_axis(t_torque)[1])

		plt.xlabel(x)
		plt.xticks(X[::len(X)//10])
		plt.ylabel(y)
		plt.yticks(Y[::len(Y)//10])

	# plt.figure()
	# imshow(efficiency, cmap='nipy_spectral', clim=(0, 1))
	# plt.title('Efficiency')
	# default_annotate()
	#
	# plt.figure()
	# current_range = np.abs(np.nan_to_num(Id)).max()
	# imshow(Id, cmap='bwr', clim=(-current_range, current_range))
	# plt.title('Id')
	# default_annotate()
	#
	# # bus power levels
	# plt.figure()
	# blim = np.abs(np.nan_to_num(bus_power)).max()
	# imshow(bus_power, cmap='bwr', clim=(-blim, blim))
	# plt.title('Bus power')
	# default_annotate()
	#
	# # dissipation
	# plt.figure()
	# imshow(dissipation, cmap='cool', clim=(0, np.nan_to_num(dissipation, nan=0).max()))
	# plt.title('Dissipation')
	# default_annotate()

	# dissipation
	plt.figure()
	alim = np.abs(np.nan_to_num(acceleration)).max()
	imshow(acceleration, cmap='bwr', clim=(-alim, alim))
	plt.title('Acceleration')
	default_annotate()

	plt.show()


def sample_graph(graph, sample_point):
	"""find i,j in graph closest to sampling point for each column"""
	delta = np.nan_to_num(graph, nan=-1e9) - sample_point
	i = np.argmin(delta**2, axis=0)
	j = np.arange(len(i))
	return i, j


def graph_sampler(graph, sample=0, s=0, e=None):
	"""find crossing point of graph in each column"""
	if e is None: e = graph.shape[1]+1
	delta = graph[:,s:e] - sample
	delta = np.nan_to_num(delta, nan=1e9)
	p, n = np.argsort(delta**2, axis=0)[:2]
	j = np.arange(len(p))
	dp, dn = delta[p, j], delta[n, j]
	wn = dp / (dp - dn)
	wn = np.clip(wn, 0, 1)
	wp = 1 - wn
	def inner(data):
		return data[:,s:e][p, j] * wp + data[:,s:e][n, j] * wn
	return inner


def integrate_traject(v, a, d=0, reverse=False):
	"""calculate braking/accel distance and dissipation

	velocity, acceleration and dissipation sample points

	could be either in radial or linear units
	"""
	v, a, d = np.broadcast_arrays(v, a, d)
	if reverse:
		v = v[::-1]
		a = a[::-1]
		d = d[::-1]
	dv = np.diff(v)
	x = 0
	e = 0
	t = 0
	for i in range(len(dv)):
		dt = dv[i] / a[i]
		t += dt
		x += v[i] * dt
		e += d[i] * dt
	return x, e, t


def integrate_step(rpm, accel, size):
	"""calculate time taken to perform a step function
	of `size` units, from standstill to standstill,
	given an acceleration graph
	"""
	f = graph_sampler(accel, 300)(accel)
	r = graph_sampler(accel, -300)(accel)
	radps = rpm/60*2*np.pi

	def step(i):
		x1, _, t1 = integrate_traject(radps[:i], f[:i])
		x2, _, t2 = integrate_traject(radps[:i], r[:i], reverse=True)
		return x1+x2, t1+t2
	for i in range(len(rpm)):
		angle, time = step(i)
		if angle > size:
			return time, rpm[i]
