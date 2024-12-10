import matplotlib.pyplot as plt
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

	Note that the logic implemented here is FOC and BLDC specific

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
	# fiXME: really want a way to visualize limits.
	#  idea; gather all limit factors into a normalized list of float arrays; stuff that needs to be < 1
	#  actual operating condition selects indices with all conditions applied
	#  limit curve graph selects indices in leave-one-out manner
	#  where we can observe limit curves as values surpassing 1
	#  nope.. kinda fails for voltage limit already. its 1 in entire FW region. dropping volt limit will produce large jumps
	#  seems like both sides of the FW region would require special treatment already.

	def masking(minimizer, mask):
		# now, we pick the optimal operating conditions, of those left in the mask
		# of all valid options for a given torque, we pick the one most favorable
		ma = np.ma.array(minimizer, mask=1 - mask)
		mask = np.ma.min(ma, axis=1).mask
		idx = np.ma.argmin(ma, axis=1)
		i = np.arange(len(idx))
		masker = lambda o : np.ma.array(o[i, idx], mask=mask).filled(np.nan)
		return i, idx, masker

	actuator = system.actuator
	battery = system.battery
	motor = actuator.motor
	controller = actuator.controller

	if controller.field_weakening:
		# FIXME: are we at all interested in positive Id? prob not, PM saturation limits sensible Id
		arange = np.linspace(-actuator.phase_current_limit, +actuator.phase_current_limit*0.0, gridsize+1)
	else:
		# FIXME: can we easily simulate what a non-FOC controller would do in practice?
		#  does it simply compute an out of bound voltage vector, and then clip it?
		arange = np.array([0.])	# always pursue Id=0; any nonzero Id implicitly considered an invalid state

	# map output ranges to motor ranges
	# FIXME: gearing efficiency makes torque function of rpm, through sign alone.
	#  need to move this into process loop below to make that work
	#  alternatively; dont paramerize in terms of output torque? stick with EM torque?
	rpm, trange = actuator.gearing.backward(rpm, trange)

	salience = motor.electrical.salience * motor.geometry.pole_pairs
	Kt_dq = motor.Kt
	Id, em_torque = np.meshgrid(arange, trange)
	Iq = em_torque / (Kt_dq + Id * salience)

	# apply inverse saturation curve to Iq
	# FIXME: this makes sense for low rpm non-salient motors operating near Id=0;
	#  but in the general case it might be more complex
	#  saturation models are already wildly underconstrained by empirical data as is though,
	#  little point in making things more complicated
	Iq = Iq * motor.electrical.saturation_factor(Iq)

	# FIXME: should ripple count towards phase current limits? i guess so conservatively.
	#  otoh motor current limits are given in terms of pure phase current, already factoring in ripple
	I_squared = Id**2 + Iq**2 #+ actuator.ripple_current**2

	bus_resistance = actuator.bus.resistance + battery.resistance
	R_dq = actuator.phase_resistance		# resistance in dq frame of motor and controller combined
	Ke_dq = Kt_dq / motor.geometry.pole_pairs	# V / (elec_rad / s)
	L_d, L_q = motor.electrical.Ld, motor.electrical.Lq

	# formulate voltage relations of the motor to solve for states within voltage limits
	# FIXME: move into electrical class? R_dq depends on controller tho
	Vq_bal = lambda omega: Iq * R_dq + omega * L_d * Id + omega * Ke_dq
	Vd_bal = lambda omega: Id * R_dq - omega * L_q * Iq
	# magnitude of voltage vector required in dq frame to reach a given current state
	V_dq = lambda omega: np.sqrt(Vq_bal(omega) ** 2 + Vd_bal(omega) ** 2)

	# these are rewritings of the above; solving Id for a given Iq at voltage equilibrium
	Id_bal = lambda omega: omega * L_q * Iq / R_dq
	Vq_bal_2 = lambda omega: Iq * R_dq + omega * L_d * Id_bal(omega) + omega * Ke_dq

	# build up boolean validity mask terms valid over all rpm
	smask = 1
	# controller phase current limit
	smask = np.logical_and(smask, I_squared < actuator.controller.phase_current_limit ** 2)
	# demagnetization limit
	smask = np.logical_and(smask, motor.demagnetiztion_factor(Iq, Id) < 1)


	def process_frequency(omega_axle_hz):
		"""construct and intersect all operating point limits and solve for remaining optimum, for given frequency"""
		omega_elec_hz = omega_axle_hz * motor.geometry.pole_pairs
		omega_axle_rad = omega_axle_hz * 2 * np.pi
		omega_elec_rad = omega_elec_hz * 2 * np.pi

		# FIXME: work Id-FW-dependence into iron drag? Id division should be about equal to demag current limit
		#  effect seems quite minimal in practice; like 3% kph continuous rating. not nothing tho
		drag_torque = np.sign(omega_axle_hz) * motor.iron_drag(omega_axle_hz) #* (1+Id/300)**2
		# NOTE: both I and R are already in the q-d frame; dont need another constant like 3/2 here.
		copper_loss = I_squared * R_dq	# this 'just works' given our chosen coordinate frame
		iron_loss = omega_axle_rad * drag_torque + omega_elec_rad**2*I_squared*0 # keep this rotor-eddy term in here for broadcasting
		dissipation = copper_loss + iron_loss
		mechanical_torque = em_torque - drag_torque
		mechanical_power = mechanical_torque * omega_axle_rad
		bus_power = dissipation + mechanical_power

		bus_current = bus_power / battery.voltage
		# compute bus voltage sag/rise
		# FIXME: solve bus voltage sag properly; get wonky results right now for high bus resistances
		effective_bus_voltage = battery.voltage - bus_current * bus_resistance
		# how much DC we really feel at the bus after clipping and series.
		# excessive bus voltage is clipped to controller voltage rather than making it explode
		voltage_effective = np.minimum(effective_bus_voltage, controller.bus_voltage_limit) * actuator.n_series  # FIXME: conservative; pass in from battery?
		# how much of a voltage vector wed want versus how much DC voltage we have to work with
		v_ratio = V_dq(omega_elec_rad) / voltage_effective

		# calculate ripple losses
		# https://www.portescap.com/en/newsroom/whitepapers/2021/10/understanding-the-effect-of-pwm-when-controlling-a-brushless-dc-motor
		# note: this is for a switching pattern with zero state (3-level)
		D = v_ratio
		ripple_factor = voltage_effective / (controller.ripple_freq * motor.electrical.L)
		ripple_delta = D * (1 - D) * ripple_factor  # peak to peak variation
		ripple_loss = ripple_delta**2 / 12 * R_dq	# to rms equivalent requires factor 12

		# adjust power terms for ripple losses
		# FIXME: solve this dependency better? bus power should feed back into the above; but lets assume ripple too small to impact bus behavior for now
		copper_loss += ripple_loss
		dissipation += ripple_loss
		bus_power += ripple_loss

		# map to output torque
		_, mechanical_torque = actuator.gearing.forward(0, mechanical_torque)


		# apply controller frequency limit
		mask = np.logical_and(smask, np.abs(omega_elec_hz) < controller.freq_limit)
		# apply voltage limit
		mask = np.logical_and(mask, v_ratio < controller.modulation_factor)
		# cap bus power
		mask = np.logical_and(mask, bus_power < battery.peak_discharge_power)
		mask = np.logical_and(mask, bus_power > -battery.peak_charge_power)
		# mech power here not appropriate; wrong sign of dissipation.
		# yet we are missing something here no? caps will dissipate
		# when switching large current zero bus regen braking
		mask = np.logical_and(mask, np.abs(bus_power) < actuator.power_limit)

		# construct masking function
		i, j, masker = masking(minimizer=bus_power, mask=mask)

		return [
			masker(o)
			for o in [copper_loss, ripple_loss, iron_loss, bus_power, mechanical_power, Iq, Id, Vq_bal_2(omega_elec_rad), mechanical_torque]
		] + [
			# these two measure distance to cone and distance from Iq=0
			# FIXME: clean this up to be more readable
			#  generalize into mechanism for returning all limit plots?
			v_ratio[i, j],
			v_ratio[i, np.argmin(np.abs(arange))]-controller.modulation_factor
		]
	# good old for loop rather than vectorize over rpm; memory use might explode otherwise
	outputs = [process_frequency(o) for o in rpm/60]
	# change to [n_graphs, torque, rpm]
	names = (
		'copper_loss',
		'ripple_loss',
		'iron_loss',
		'bus_power',
		'mechanical_power', 'Iq', 'Id', 'Vq_bal', 'mechanical_torque', 'v_ratio', 'v_ratio_2')
	return dict(zip(names, np.moveaxis(outputs, [1, 2, 0], [0, 1, 2]).astype(np.float32)))




def round(x, digits):
	"""round to a number of significant digits"""
	shift = 10**int(np.log10(x) - digits + 1)
	return np.ceil(x / shift) * shift


def system_detect_limits(system, fw=1.5, frac=0.98, padding=1.2):
	"""auto-detect some reasonably tight limits on the actuator system"""
	max_torque = system.actuator.peak_torque * 1.2
	max_rpm = system.actuator.motor.electrical.Kv * system.battery.voltage * fw * system.actuator.n_series
	trange = np.linspace(-max_torque, +max_torque, 64, endpoint=True)
	rpm = np.linspace(0, max_rpm, 64, endpoint=False)
	graphs = system_limits(system, trange, rpm)
	torque = graphs['mechanical_torque']
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
	color='power',
	annotations='tdeos',
	rpm_negative=False,
	torque_negative=True,
):
	"""mpl plots of system limits and properties"""
	import matplotlib.pyplot as plt
	from matplotlib.lines import Line2D

	# setup calculation grids
	if max_torque is None or max_rpm is None:
		_max_rpm, _max_torque = system_detect_limits(system)
		max_rpm = max_rpm or _max_rpm
		max_torque = max_torque or _max_torque

	rpm_range = np.linspace(-max_rpm*rpm_negative, max_rpm, n_rpm+1, endpoint=True)
	torque_range = np.linspace(-max_torque*torque_negative, +max_torque, n_torque + 1, endpoint=True)

	# eval the system performance graphs
	graphs = system_limits(system, torque_range, rpm_range)

	copper_loss = graphs['copper_loss']
	iron_loss = graphs['iron_loss']
	bus_power = graphs['bus_power']
	mechanical_power = graphs['mechanical_power']
	Iq = graphs['Iq']
	Id = graphs['Id']
	vbal = graphs['Vq_bal']
	torque = graphs['mechanical_torque']

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
		(s['color'], s['dt']): system.temperatures(
			rpm_range, copper_loss, iron_loss, dt=s['dt']) - s['dT']
		for s in thermal_specs
	}

	x_label, x_range = system.x_axis(rpm_range)
	y_label, y_range = system.y_axis(torque_range)

	def imshow(im, **kwargs):
		a = plt.pcolormesh(x_range, y_range, im, mouseover=True, **kwargs)
		plt.colorbar()
		return a

	def contour(im, **kwargs):
		return plt.contour(x_range, y_range, im, **kwargs)

	label_handles = []
	def add_label(label, color):
		line = Line2D([0], [0], label=label, color=color)
		label_handles.append(line)

	def default_annotate():
		if 'd' in annotations:
			# delineate FW-region
			contour(Id, levels=[-0.5], colors='white')
			add_label(color='white', label='Field weakening')
		if 't' in annotations:
			# plot thermal limits
			for (c, dt), m in thermals.items():
				contour(m, levels=[0], colors=c)
				add_label(color=c, label=f'Thermal {dt}s')
		if 'a' in annotations:
			# # plot acceleration load lines
			amin, amax = np.nan_to_num(acceleration).min(), np.nan_to_num(acceleration).max()
			aname, lines = system.acceleration_lines()
			for a in lines:
				if np.all(a > amin+1e-3) and np.all(a < amax-1e-3):
					contour(acceleration, levels=[a], colors='black', linewidths=1.5)
			contour(acceleration, levels=[amin + 5e-4], colors='black', linewidths=2)
			contour(acceleration, levels=[amax - 5e-4], colors='black', linewidths=2)
			add_label(color='black', label='Acceleration')
		if 'e' in annotations:
			contour(efficiency, levels=[0.9], colors='green')
			add_label(color='green', label='90% Efficiency')
		if 'o' in annotations:
			# open loop torque curve
			contour(vbal, levels=[0], colors='brown')
			add_label(color='brown', label='Open Loop')
		if 's' in annotations:
			# saturation limit
			p = np.abs(Iq) - system.actuator.motor.electrical.saturation
			contour(p, levels=[0], colors='gray')
			add_label(color='gray', label='Saturation')


		if targets is not None:
			t_torque, t_rpm, t_dissipation, t_weight = [np.array(t) for t in targets]
			plt.scatter(system.x_axis_forward(t_rpm), system.y_axis_forward(t_torque))

		if torque_negative:
			# plot x axis
			plt.plot(x_range, x_range * 0, c='black',linewidth=0.5)
		if rpm_negative:
			# plot y axis
			plt.plot(y_range*0, y_range, c='black',linewidth=0.5)

		plt.legend(handles=label_handles, loc='upper right')
		plt.xlabel(x_label)
		plt.xticks(x_range[::len(x_range)//10])
		plt.ylabel(y_label)
		plt.yticks(y_range[::len(y_range)//10])

	plt.figure()
	if color == 'efficiency':
		imshow(efficiency, cmap='nipy_spectral', clim=(0, 1))
		plt.title('Efficiency')
	if color == 'Id':
		current_range = np.abs(np.nan_to_num(Id)).max()
		imshow(Id, cmap='bwr', clim=(-current_range, current_range))
		plt.title('Id')
	if color == 'Iq':
		current_range = np.abs(np.nan_to_num(Iq)).max()
		imshow(Iq, cmap='bwr', clim=(-current_range, current_range))
		plt.title('Iq')
	# if color == 'I':
	# 	current_range = np.abs(np.nan_to_num(Iq)).max()
	# 	imshow(Iq, cmap='bwr', clim=(-current_range, current_range))
	# 	plt.title('Iq')
	if color == 'power':
		blim = np.abs(np.nan_to_num(bus_power)).max()
		imshow(bus_power, cmap='bwr', clim=(-blim, blim))
		plt.title('Bus power')
	if color == 'dissipation':
		imshow(dissipation, cmap='cool', clim=(0, np.nan_to_num(dissipation, nan=0).max()))
		plt.title('Dissipation')
	if color == 'acceleration':
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
