
import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.actuator import Actuator
from pypowertrain.components.battery import Battery


@dataclasses.dataclass(frozen=True)
class System(Base):
	"""simply battery-controller-motor system"""
	battery: Battery
	actuator: Actuator

	@property
	def weight(self):
		return self.battery.weight + self.actuator.weight

	def x_axis(self, rpm=0):
		return 'rpm', rpm
	def y_axis(self, nm=0):
		return 'Nm', nm



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
		# bit of a mystery why not to include fw as a controller feature, but soit
		arange = np.array([0.])

	# map output ranges to motor ranges
	# FIXME: gearing efficiency makes torque function of rpm, through sign alone.
	#  need to move this into process loop below to make that work
	#  alternatively; dont paramerize in terms of output torque? stick with EM torque?
	rpm, trange = actuator.gearing.backward(rpm, trange)

	salience = -(motor.Lq - motor.Ld)
	Id, em_torque = np.meshgrid(arange, trange)
	Iq = em_torque / (3/2) / motor.pole_pairs / (motor.flux + Id * salience)

	# should ripple count towards phase current limits? i guess so conservatively.
	Is = Id**2 + Iq**2 + actuator.ripple_current**2

	bus_resistance = actuator.bus.resistance + battery.resistance
	Rt = actuator.phase_resistance


	Vlim = lambda omega: (Id*Rt - omega*motor.Lq*Iq)**2 + (Iq*Rt + omega*motor.Ld*Id + omega*motor.flux)**2
	# Vlim = lambda omega: (- omega*motor.Lq*Iq)**2 + ( + omega*motor.Ld*Id + omega*motor.flux)**2
	def process_omega(omega_axle_hz):
		"""construct and intersect amp and V limits for a given omega"""
		omega_elec_hz = omega_axle_hz * motor.pole_pairs
		omega_axle_rad = omega_axle_hz * 2 * np.pi
		omega_elec_rad = omega_elec_hz * 2 * np.pi

		drag_torque = motor.hysteresis_drag(omega_elec_rad)
		dissipation = Is*Rt + omega_axle_rad * drag_torque + omega_elec_rad**2*Is*motor.eddy_em
		mechanical_torque = em_torque - drag_torque
		mechanical_power = mechanical_torque * omega_axle_rad
		bus_power = dissipation + mechanical_power

		bus_current = bus_power / battery.voltage	# FIXME: solve bus voltage sag properly; get errors right now for high resistsances
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

		# of all valid Id options for a given em_torque, we pick the one most favorable to bus power
		ma = np.ma.array(bus_power, mask=1-mask)
		# this is a reduction over Id
		mask = np.ma.min(ma, axis=1).mask
		idx = np.ma.argmin(ma, axis=1)
		i = np.arange(len(trange))

		# map to output torque
		_, mechanical_torque = actuator.gearing.forward(0, mechanical_torque)

		return [
			np.ma.array(o[i, idx], mask=mask).filled(np.nan)
			for o in [dissipation, bus_power, mechanical_power, Iq, Id, mechanical_torque]
		]
	# good old for loop; memory use might explode otherwise
	outputs = np.array([process_omega(o) for o in rpm/60])
	return np.moveaxis(outputs, [1, 2, 0], [0, 1, 2])




def round(x, digits):
	"""round to a number of significant digits"""
	shift = 10**int(np.log10(x) - digits + 1)
	return np.ceil(x / shift) * shift


def system_detect_limits(system, fw=3, frac=0.95, padding=1.1):
	"""auto-detect some reasonably tight limits on the actuator system"""
	max_torque = system.actuator.peak_torque * 1.1
	max_rpm = system.actuator.motor.Kv * system.battery.voltage * fw
	trange = np.linspace(-max_torque, +max_torque, 50, endpoint=True)
	rpm = np.linspace(0, max_rpm, 50, endpoint=False)
	dissipation, bus_power, mechanical_power, Iq, Id, torque = system_limits(system, trange, rpm)
	max_torque = np.max(np.abs(np.nan_to_num(torque)))
	max_rpm = rpm[::-1][np.argmin(np.mean(np.isnan(torque), axis=0)[::-1] > frac)]
	max_rpm = round((max_rpm) * padding, 2)
	max_torque = round((max_torque) * padding, 2)
	return max_rpm, max_torque


def system_plot(
	system: System,
	n_rpm=200,
	n_torque=500,
	max_rpm=None,
	max_torque=None,
	targets=None,
):

	import matplotlib.pyplot as plt

	# setup calculation grids
	if max_torque is None:
		max_rpm, max_torque = system_detect_limits(system)

	rpm_range = np.linspace(0, max_rpm, n_rpm+1, endpoint=True)
	torque_range = np.linspace(-max_torque, +max_torque, n_torque + 1, endpoint=True)

	# eval the system performance graphs
	dissipation, bus_power, mechanical_power, Iq, Id, torque = system_limits(system, torque_range, rpm_range)

	# construct thermal curves
	heat_loss = system.actuator.heat_loss(rpm_range)
	# at infinite time horizon, capacity does not matter
	heat_inf_80c = dissipation - heat_loss
	CE = system.actuator.heat_capacity_stator(dT=40)
	dt = 60 # s
	heat_1min_40c = (dissipation - heat_loss) - CE / dt
	# heat_1min_40c = dissipation - 70	# overlay with rated power
	# in 2sec timeframe, we count copper coil mass capacity alone
	CE = system.actuator.heat_capacity_copper(dT=40)
	dt = 2
	heat_2s_40c = (dissipation - heat_loss) - CE / dt
	thermals = {
		'yellow': heat_inf_80c,
		'orange': heat_1min_40c,
		'red': heat_2s_40c,
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
		# plot zero torque line
		plt.plot(X, X * 0, c='gray')
		# plot thermal limits
		for c, m in thermals.items():
			contour(m, levels=[0], colors=c)

		if targets is not None:
			t_torque, t_rpm, t_dissipation, t_weight = np.array(targets)
			plt.scatter(t_rpm, t_torque)

		plt.xlabel(x)
		plt.xticks(X[::len(X)//10])
		plt.ylabel(y)
		plt.yticks(Y[::len(Y)//10])

	plt.figure()
	efficiency = np.where(
		torque >= 0,
		np.abs(mechanical_power) / np.abs(bus_power),
		np.abs(bus_power) / np.abs(mechanical_power)
	)
	imshow(efficiency, cmap='nipy_spectral', clim=(0, 1))
	plt.title('Efficiency')
	default_annotate()

	plt.figure()
	current_range = np.abs(np.nan_to_num(Id)).max()
	imshow(Id, cmap='bwr', clim=(-current_range, current_range))
	plt.title('Id')
	default_annotate()

	# bus power levels
	plt.figure()
	blim = np.abs(np.nan_to_num(bus_power)).max()
	imshow(bus_power, cmap='bwr', clim=(-blim, blim))
	plt.title('Bus power')
	default_annotate()

	# dissipation
	plt.figure()
	imshow(dissipation, cmap='cool', clim=(0, np.nan_to_num(dissipation, nan=0).max()))
	plt.title('Dissipation')
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
