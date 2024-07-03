import numpy as np

from pypowertrain.utils import *

from pypowertrain.components.bus import Bus
from pypowertrain.components.controller import Controller
from pypowertrain.components.motor import Motor
from pypowertrain.components.gearing import Gearing, define_direct


@dataclass
class Actuator(Base):
	motor: Motor
	controller: Controller

	gearing: Gearing = define_direct()
	bus: Bus = Bus(length=1, area=1e-3**2)	# 1m 1mm copper

	n_series: int = 1
	# n_parallel: int = 1

	@property
	def weight(self):
		return self.motor.weight + self.controller.weight * self.n_series + self.bus.weight

	@property
	def peak_torque(self):
		A = self.phase_current_limit
		salience = (self.motor.Ld - self.motor.Lq)
		t = A * self.motor.Kt + A * A / 2 * np.abs(salience) * (3/2) * self.motor.pole_pairs
		_, t = self.gearing.forward(1, t)
		return t
	@property
	def phase_current_limit(self):
		return np.minimum(self.controller.phase_current_limit, self.motor.phase_current_limit)

	@property
	def power_limit(self):
		return self.controller.power_limit * self.n_series

	def effective_voltage(self, v_bus):
		"""map voltages to effective FOC available voltage"""
		v_bus = np.minimum(v_bus, self.controller.bus_voltage_limit)
		return v_bus * self.controller.modulation_factor * self.n_series


	@property
	def ripple_current(self):
		voltage = self.controller.bus_voltage_limit * self.n_series	# FIXME: conservative; pass in from battery?
		ripple = lambda L: voltage / (2*self.controller.ripple_freq*L) / np.sqrt(2) / np.sqrt(3)
		return np.sqrt(ripple(self.motor.Lq)**2 + ripple(self.motor.Ld)**2)

	@property
	def phase_resistance(self):
		return self.motor.resistance + self.controller.resistance * self.n_series

	def heat_loss(self, rpm):
		mps = rpm / 60 * self.motor.circumference
		dT = 60#self.motor.magnet_temperature - 20
		W = self.motor.thermal_resistance * dT * self.motor.circumference

		# empirical scaling law of convective transfer between 2-20m/s
		convection_scale = lambda v: 10.45 - v + 10 * np.sqrt(v)
		# we assume 250w at 8m/s as an empirical value 200 convective and 50 radiative?
		convection_loss = lambda v: convection_scale(v) / convection_scale(8) * 0.8
		radiation_loss = 0.2
		return (convection_loss(mps) + radiation_loss) * W

	def heat_capacity(self, dT=40):
		copper_cp = 0.376e3
		return (self.motor.weight) * copper_cp * dT
	def heat_capacity_stator(self, dT=40):
		copper_cp = 0.376e3
		return (self.motor.copper_weight + self.motor.iron_weight) * copper_cp * dT
	def heat_capacity_copper(self, dT=40):
		copper_cp = 0.376e3
		return self.motor.copper_weight * copper_cp * dT

	def plot(self, ax=None):
		"""graphical representation of motor and controller in relation"""
		import matplotlib.pyplot as plt
		from matplotlib.collections import LineCollection

		# FIXME: move to plotting util module
		def rotate(angles, geo):
			c, s = np.cos(angles), np.sin(angles)
			r = np.array([[c, s], [-s, c]])
			return np.einsum('abr, pa -> rpb', r, geo)

		def square(s, e):
			g = np.array([s, e])
			geo = [
				[g[0, 0], g[0, 1]],
				[g[0, 0], g[1, 1]],
				[g[1, 0], g[1, 1]],
				[g[1, 0], g[0, 1]],
				[g[0, 0], g[0, 1]]]
			return np.array(geo)

		def circle(r, n=360):
			a = np.linspace(0, np.pi*2, n, endpoint=True)
			a+=a[1]/2
			return np.array([np.cos(a), np.sin(a)]) * r

		if ax is None:
			fig, ax = plt.subplots(1, 1)

		self.motor.plot(ax=ax)

		def controller(w, h, r):
			shaft_radius = r
			geo = square(
				[shaft_radius, h/2],
				[shaft_radius+w, -h/2])
			geo = rotate(np.linspace(0, np.pi*2, self.n_series, endpoint=False), geo)
			line_collection = LineCollection(geo, linewidths=2)
			ax.add_collection(line_collection)
			plt.plot(*circle(r))

		controller(self.controller.width, self.controller.length, 12e-3)

		return fig, ax
