from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry, winding


def test_geometry():
	geo = Geometry.create(
		poles=46,
		slots=42,
		turns=5,

		gap_diameter=199e-3,
		gap_length=27e-3,
		slot_depth_fraction=0.16,
		magnet_height=3e-3,
		airgap=0.7e-3,
	)
	# geo.replace(slot_width_fraction=0.4).plot()
	print(geo)

	print(list(expand_paths('turns', geo)))
	print(geo.flux_ratios())
	print(geo.pm_flux_scale)
	print(geo.em_flux_scale*(150/1.0))
	# at 150A/nm, we are still under 1/3 of pm flux
	# pythagoras summing q and d fluxes total is only 5% more, at 150nm
	print(geo.iron_field_scale * 1.39) # *1.39 for typical closed loop neo strength; indeed around 2T
	# geo = geo.rescale(frequency_scale=2)
	# geo.plot()


def test_L():
	"""copy parameters from paper; check that we get Lew=0.01"""
	geo = Geometry(
		poles=4,
		slots=12,
		turns=19,

		gap_radius=530-3 / 2,
		gap_length=120e-3,
		slot_depth_fraction=0.5,
		slot_width_fraction=0.75,
		magnet_height=250e-3/10,
		airgap=250e-3/40,
	)
	print(23.5e-3/((1.9+0.62)/2)/geo.flux_ratios()[1])
	print(geo.flux_ratios()[1])
	print()

	geo = geo.replace(
		gap_radius=636e-3/2,
		gap_length=168e-3,
		turns=17
	)
	print(10.0e-3/((0.95+0.31)/2)/geo.flux_ratios()[1])
	print(geo.flux_ratios()[1])


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

