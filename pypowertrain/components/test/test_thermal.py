import numpy as np

from pypowertrain.components.thermal import *
from pypowertrain.components.motor.thermal import shelled_thermal


def test_couette():
	"""laminar couette air drag really should not be the issue, even for tiny gap"""
	def radiative_k(ta, tb, eps = 1.0):
		"""radiative conductivity, linearlized around a pair of temperatures in C"""
		ta, tb = ta + 273, tb + 273
		s = 5.6703e-8
		return s * eps * (ta ** 4 - tb ** 4) / (ta - tb)

	def couette_k(a, l):
		k = 2.88e-2	#	@ 60C
		return k * a / l

	nu = 2e-5
	l, d = 3e-2, 0.2
	v = 36/2
	a = l * d*np.pi
	gl = 0.5e-3
	f = a * nu / gl
	print(f*d/2*v)
	e = 12e-6	# alu expension coef; worst case; mostly dealing with steel-vs-steel in reality
	dt = 40
	print(e*dt * d/2)	# even then, just 1/20 of a mm
	k=2.88e-2
	print(k*a/gl)
	q = radiative_k(80, 50)
	print(q * a)
	sa = d**2/4*np.pi*2
	print(radiative_k(60, 30) * sa)


def test_fit():
	# Fit thermal model to grin data
	# https://electricbike-blog.com/2015/12/18/ferrofluids-join-the-ebike-motor-cooling-revolution/
	# lots of links here in background
	# https://ebikes.ca/product-info/grin-products/statorade.html
	from scipy.optimize import differential_evolution
	def replace_conductivity(a, b, c):
		thermal = all_axle(h=c)
		conductivity = dict(thermal.conductivity)
		conductivity[('stator', 'rotor')] = a
		conductivity[('rotor', 'shell')] = b
		return thermal.replace(
			conductivity=conductivity
		)

	def solve():
		def opt(args):
			a, aa, b, c = args

			qq = {'coils': 68 - 22, 'rotor': 33 - 22, 'shell': 30 - 22}
			res = replace_conductivity(a, b, c).solve(
				{'coils': 95}, dt=5000
			)
			q1 = np.linalg.norm([res[k] - v for k, v in qq.items()])

			qq = {'coils': 44 - 18, 'rotor': 34 - 18, 'shell': 28 - 18}
			res = replace_conductivity(aa, b, c).solve(
				{'coils': 95}, dt=5000
			)
			q2 = np.linalg.norm([res[k] - v for k, v in qq.items()])
			return q1 + q2

		res = differential_evolution(opt, [[0.1, 10], [0.1, 10], [3, 40], [1, 20]], polish=True)
		return res.x

	print(solve())


def test_fit_old():
	# Fit thermal model to grin data
	# https://electricbike-blog.com/2015/12/18/ferrofluids-join-the-ebike-motor-cooling-revolution/
	# lots of links here in background
	# https://ebikes.ca/product-info/grin-products/statorade.html
	from scipy.optimize import differential_evolution

	def replace_conductivity(a, b, c):
		thermal = all_axle(h=c)
		conductivity = dict(thermal.conductivity)
		conductivity[('stator', 'rotor')] = a
		conductivity[('rotor', 'shell')] = b
		return thermal.replace(
			conductivity=conductivity
		)

	def solve():
		def opt(args):
			a, aa, b, c = args

			qq = {'coils': 68 - 22, 'rotor': 33 - 22, 'shell': 30 - 22}
			res = replace_conductivity(a, b, c).solve(
				{'coils': 95}, dt=5000
			)
			q1 = np.linalg.norm([res[k] - v for k, v in qq.items()])

			qq = {'coils': 44 - 18, 'rotor': 34 - 18, 'shell': 28 - 18}
			res = replace_conductivity(aa, b, c).solve(
				{'coils': 95}, dt=5000
			)
			q2 = np.linalg.norm([res[k] - v for k, v in qq.items()])
			return q1 + q2

		res = differential_evolution(opt, [[0.1, 10], [0.1, 10], [3, 40], [1, 20]], polish=True)
		return res.x

	print(solve())


def test_fitted():
	from pypowertrain.library import grin
	motor = grin.all_axle()
	import matplotlib.pyplot as plt
	t = np.linspace(0, 5000, 100)
	fig, ax = plt.subplots(3)
	thermals = all_axle(motor, statorade=True)
	K = ShellConductivity(motor.mass)
	res = [list(thermals.solve({'coils': 95}, dt=dt).values()) for dt in t]
	ax[1].plot(t, np.array(res) + 18)
	thermals = all_axle(motor, statorade=False)
	res = [list(thermals.solve({'coils': 95}, dt=dt).values()) for dt in t]
	ax[0].plot(t, np.array(res) + 22)
	thermals = all_axle(motor, statorade=False, vented=True)
	res = [list(thermals.solve({'coils': 95}, dt=dt).values()) for dt in t]
	ax[2].plot(t, np.array(res) + 22)
	plt.figlegend(thermals.keys)
	plt.show()


