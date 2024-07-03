import numpy as np

from pypowertrain.system import *
from pypowertrain.utils import *


@dataclasses.dataclass(frozen=True)
class Bike(Base):
	front: Actuator
	rear: Actuator

	battery: Battery

	CdA: float
	Cr: float
	nominal_kmh: float
	wheel_diameter: float
	structure_weight: float
	rider_weight: float = 80

	cog_height: float = 0.8
	cog_rear: float = 0.5
	cog_front: float = 0.5

	Cf: float = 0.8

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
	def nominal_kinetic(self):
		mps = self.nominal_kmh / 3.6
		return (1/2) * self.weight * mps**2
	@property
	def n_motors(self):
		return (0 if self.front is None else 1) + (0 if self.rear is None else 1)

	def kph_to_rpm(self, kph):
		return kph * 1000 / 60 / self.wheel_circumference
	def rpm_to_kph(self, kph):
		return kph / 1000 * 60 * self.wheel_circumference

	@property
	def downforce(self):
		return self.weight * 9.81

	def aero_drag(self, mps):
		rho = 1.225
		return (1 / 2) * rho * self.CdA * mps ** 2

	def gravity_drag(self, grade):
		return self.downforce * np.sin(np.arctan(grade))

	def rolling_drag(self):
		return self.Cr * self.downforce

	def system_drag_force(self, kph, grade=0):
		"""power requirement of bike system"""
		mps = kph / 3.6
		force = self.rolling_drag() + self.aero_drag(mps) - self.gravity_drag(grade)
		return force

	def system_drag(self, kph, grade=0):
		"""power requirement of bike system"""
		mps = kph / 3.6
		return self.system_drag_force(kph, grade) * mps

	@property
	def electrical_weight(self):
		return (self.battery.weight
				+ (0 if self.rear is None else self.rear.weight)
				+ (0 if self.front is None else self.front.weight))

	@property
	def weight(self):
		return self.rider_weight + self.structure_weight + self.electrical_weight

	def plot(self):
		"""plot wheels in relation to COM"""

	def rear_downforce(self):
		return self.downforce * self.cog_front / self.wheelbase
	def front_downforce(self):
		return self.downforce * self.cog_rear / self.wheelbase


@dataclass
class BikeSystem(System):
	bike: Bike = None
	# def speed_forward(self, rpm):
	# 	return self.bike.rpm_to_kph(rpm)
	# def speed_backward(self, speed):
	# 	return self.bike.kph_to_rpm(speed)
	# def force_forward(self, nm):
	# 	return nm / self.bike.wheel_radius / self.bike.weight / 9.81
	# def force_backward(self, nm):
	# 	return nm * self.bike.wheel_radius * self.bike.weight * 9.81

	def x_axis(self, rpm=None):
		return 'kmh', self.bike.rpm_to_kph(rpm)
	def y_axis(self, nm=None):
		return 'G', nm / self.bike.wheel_radius / self.bike.weight / 9.81


def bike_stats(bike):
	print('total drag', bike.replace().system_drag(bike.nominal_kmh))
	print('rolling drag', bike.replace(CdA=0).system_drag(bike.nominal_kmh))
	print('aero drag', bike.replace(Cr=0).system_drag(bike.nominal_kmh))


def bike_plot(
	bike: Bike,
	torque_range=None,
	gridsize=500,
	targets=None
):
	"""plot bike performance"""

	import matplotlib.pyplot as plt
	# setup calculation grids
	torque_range = torque_range or bike.rear.peak_torque * 1.1
	trange = np.linspace(-torque_range, +torque_range, gridsize+1, endpoint=True)

	kph = np.linspace(0, bike.nominal_kmh*2, 200+1, endpoint=True)
	rpm = bike.kph_to_rpm(kph)
	mps = kph / 3.6

	# FIXME: just using rear now? work out multi-motor system;
	system = BikeSystem(actuator=bike.rear, battery=bike.battery, bike=bike)
	# system_plot(system, targets=targets)
	#
	# return
	dissipation, bus_power, mechanical_power, Iq, Id, torque = system_limits(system, trange, rpm)


	# vehicle model

	winds = [-10, 0, +10]
	# winds = [0]
	# bumps = [-.05, 0, +.05]
	bumps = [0]

	# bumps = [0]
	# 34% grade is world record; 17% is dutch record
	# FIXME: dont need averaging here; makes bumps useless; need to fold in dissipation
	# net force per wheel
	wheel_force = (
		torque / bike.wheel_radius -
		# bike.system_drag(kph, grade)
		np.mean([bike.system_drag_force(kph+w, 0) for w in winds for b in bumps], axis=0) / bike.n_motors
		# for grade in [0]#np.linspace(-0.34, 0.34, 5)
	)
	grades = [bike.gravity_drag(g) for g in np.linspace(-0.34, 0.34, 5)]


	# x axis idx of nominal kph
	idx = np.argmin((kph - bike.nominal_kmh) ** 2)

	sample = graph_sampler(wheel_force, 1e6, 0, idx + 1)
	abs_force = abs_traction(bike, sample(wheel_force), 0 if bike.front is None else None)
	dm, de, ts = integrate_traject(mps[:idx + 1], abs_force/bike.weight, sample(dissipation))
	print(f'{(bike.nominal_kmh/3.6) / ts / 9.81} G average accel')
	print(f'{ts} s to nominal')
	print(f'{de/bike.rear.heat_capacity_copper(dT=1)} dT(C) to nominal')

	sample = graph_sampler(wheel_force, -1e6, 0, idx + 1)
	abs_force = abs_traction(bike, sample(wheel_force), 0 if bike.front is None else None)
	dm, de, ts = integrate_traject(mps[:idx + 1], abs_force/bike.weight, sample(dissipation), reverse=True)
	print(f'{(bike.nominal_kmh/3.6) / ts / 9.81} G average decel')
	print(f'{ts}s to full stop')
	print(f'{de/bike.rear.heat_capacity_copper(dT=1)} dT(C) to full stop')


	# sample equilibrium load line at rated speed
	sampler = graph_sampler(wheel_force, 0, idx, idx + 1)
	print(f'system_power: {sampler(mechanical_power) * bike.n_motors} W')
	copper_loss = sampler(Iq**2+Id**2) * bike.rear.phase_resistance
	print('copper', copper_loss*bike.n_motors)
	print('dissipation', sampler(dissipation) * bike.n_motors)

	power_use = sampler(bus_power) * bike.n_motors
	print(f'bus power at nominal: {power_use} W')
	print('runtime (h): ', bike.battery.capacity / power_use)
	print('range (km): ', bike.battery.capacity / power_use * bike.nominal_kmh)

	# construct thermal curves
	heat_loss = bike.rear.heat_loss(rpm)
	# at infinite time horizon, capacity does not matter
	heat_inf_80c = dissipation - heat_loss
	CE = bike.rear.heat_capacity(dT=40)
	# let loss count 50% in 1min timeframe?
	dt = 60 # s
	heat_1min_40c = (dissipation - heat_loss / 2) * dt - CE
	# in 2sec timeframe, we count copper coil mass capacity alone
	CE = bike.rear.heat_capacity_copper(dT=40)
	dt = 2
	heat_2s_40c = dissipation * dt - CE

	thermals = {
		'yellow': heat_inf_80c,
		'orange': heat_1min_40c,
		'red': heat_2s_40c,
	}

	def imshow(im, **kwargs):
		a = plt.pcolormesh(kph, trange, im, **kwargs)
		plt.colorbar()
		return a
	def contour(im, **kwargs):
		return plt.contour(kph, trange, im, **kwargs)

	def default_annotate():
		# delineate FW-region
		contour(Id, levels=[-1], colors='white')

		plt.plot(kph, kph * 0, c='gray')
		plt.plot(np.ones_like(trange) * bike.nominal_kmh, trange, c='gray')

		for ll in grades:
			contour(wheel_force*bike.n_motors + ll, levels=[0], colors='black')
		# plot thermal limits
		for c, m in thermals.items():
			contour(m, levels=[0], colors=c)

		if targets is not None:
			t_torque, t_rpm, t_dissipation, t_weight = np.array(targets)
			plt.scatter(bike.rpm_to_kph(t_rpm), t_torque)

		plt.xticks(kph[::20])
		plt.yticks([-torque_range, 0, torque_range])
		plt.xlabel('kmh')
		plt.ylabel('Nm')

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

	plt.figure()
	g = wheel_force * bike.n_motors / bike.downforce
	imshow(g, cmap='bwr', clim=(-bike.Cf, bike.Cf))
	# contour(g, levels=np.linspace(-0.8, 0.8, 9, endpoint=True))
	plt.title('Acceleration (G)')
	default_annotate()


	plt.show()


def abs_traction(bike, rearforce=None, frontforce=None):
	"""calculate braking/accel forces with abs,
	taking into account motor limits, friction limits, and weight shift effects"""
	if frontforce is None: frontforce = rearforce
	if rearforce is None: rearforce = frontforce
	rearforce, frontforce = np.broadcast_arrays(rearforce, frontforce)

	def shift(force):
		"""calc weight shift on each wheel as function of total braking force"""
		return (force * bike.cog_height) / bike.wheelbase

	rear_df = lambda force: bike.rear_downforce() + shift(force)
	front_df = lambda force: bike.front_downforce() - shift(force)

	def amin(a, alim):
		"""cap magnitude"""
		alim = np.maximum(alim, 0)
		return np.clip(a, -alim, +alim)

	import scipy.optimize
	root = lambda root, x0: scipy.optimize.root_scalar(root, x0=x0).root

	def virtual_abs(rf, ff):
		rearforce = lambda force: amin(rf, rear_df(force) * bike.Cf)
		frontforce = lambda force: amin(ff, front_df(force) * bike.Cf)
		balance = lambda force: rearforce(force) + frontforce(force) - force
		return root(balance, bike.downforce*bike.Cf)
	force = [virtual_abs(r, f) for r, f in zip(
		rearforce, frontforce)]

	stoppie = root(rear_df, bike.downforce/2)
	wheelie = root(front_df, bike.downforce/2)
	return np.clip(force, stoppie, wheelie)


