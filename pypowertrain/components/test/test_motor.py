import numpy as np

from pypowertrain.components.motor import *
from pypowertrain.library import grin


def test_scaling():
	import matplotlib.pyplot as plt
	grin.all_axle().rescale(
		turns=4/5,
		copper=0.8,
		magnet=1.5,
		radius=0.5,
	).plot()
	plt.show()


def test_winding():
	"""Think there are some bugs hiding in this logic;
	but its only used for plotting at the moment anyway"""
	P = np.arange(2, 40) * 2
	S = np.arange(2, 26) * 3

	def score(p, s):
		_, _, wf, b = winding(p, s)
		b = b * 1.0
		if p >= s * 1.9 or p<=s / 1.9: b = np.float_(0)
		return wf * (b/b)
	wf = [[score(p, s) for s in S] for p in P]
	import matplotlib.pyplot as plt
	plt.pcolor(S, P, wf)
	plt.colorbar()
	plt.xlabel('slots')
	plt.ylabel('poles')
	plt.show()