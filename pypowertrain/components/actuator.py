import numpy as np

from pypowertrain.utils import *

from pypowertrain.components.bus import Bus
from pypowertrain.components.controller import Controller
from pypowertrain.components.motor.motor import Motor
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
		return self.motor.mass.total + self.controller.weight * self.n_series + self.bus.weight

	@property
	def peak_torque(self):
		A = self.phase_current_limit
		# salience = (self.motor.Ld - self.motor.Lq)
		salience = self.motor.electrical.salience
		t = A * self.motor.Kt + A * A / 2 * np.abs(salience) * (3/2) * self.motor.geometry.pole_pairs
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
		return np.sqrt(ripple(self.motor.electrical.Lq)**2 + ripple(self.motor.electrical.Ld)**2)

	@property
	def phase_resistance(self):
		return self.motor.resistance + self.controller.resistance * self.n_series

	def temperatures(self, mps, rpm, copper_loss, iron_loss, dt, key='coils'):
		"""build motor temp graphs from heat source/sink graphs"""
		circumferential = rpm / 60 * self.motor.geometry.gap_circumference
		def process_speed(i):
			thermal = self.motor.thermal.replace(
				conductivity__linear=np.abs(mps[i]),
				conductivity__circumferential=np.abs(circumferential[i])
			)
			# solve impulse responses
			d_iron = thermal.solve({'stator': 1}, dt=dt)[key]
			d_copper = thermal.solve({'coils': 1}, dt=dt)[key]
			return d_iron * iron_loss[:, i] + d_copper * copper_loss[:, i]
		return np.array([process_speed(i) for i in range(len(mps))]).T

	def plot(self, ax=None):
		"""graphical representation of motor and controller in relation to one another"""
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
			show=True
		else:
			show=False

		self.motor.geometry.plot(ax=ax)

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

		if show:
			plt.show()
		return fig, ax
