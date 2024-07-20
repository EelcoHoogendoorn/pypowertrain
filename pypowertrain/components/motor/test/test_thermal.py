import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry
from pypowertrain.components.motor.mass import Mass
from pypowertrain.components.motor.thermal import *


def fixture(**kwargs):
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

	mass = Mass.from_absolute(
		geometry=geo,
		total=4.0
	)

	K = ShelledConductivity.from_mixed(
		geometry=geo,
		**kwargs
	)

	return Thermal(
		capacity=generic_capacity(mass=mass),
		conductivity=K)


def test_fixture():
	thermal = fixture(statorade=True)
	geo = thermal.capacity.mass.geometry
	print()
	print(geo.coils_contact_area / geo.coil_thickness)
	print(geo.side_area*2)
	k = thermal.conductivity
	print()
	for kv in (k.get_attrs().items()):
		print(kv)
	print()
	for kv in (k.attrs.items()):
		print(kv)


def test_utils():
	thermal = fixture()
	# temps = thermal.solve({'coils': 5000}, dt=2)
	# print(temps)

	temps = thermal.solve({'coils': 200}, dt=100)
	print(temps)
	# FIXME: rescaling appears broken? but replace does work?
	print(thermal.capacity.mass.geometry.radius)
	print(thermal.conductivity.geometry.radius)
	# print(list(thermal.conductivity.values()))
	thermal = thermal.replace(__geometry__radius_scale=1.2)
	# thermal = thermal.replace(__geometry__radius_scale=0.5)
	print(thermal.capacity.mass.geometry.radius)
	print(thermal.conductivity.geometry.radius)
	# print(list(thermal.conductivity.values()))
	temps = thermal.solve({'coils': 200}, dt=100)
	print(temps)

	print(thermal.conductivity.get_attrs())
	print(thermal.conductivity.attrs)


def test_plot_rpm():
	import matplotlib.pyplot as plt
	dt = 1e4
	kph = np.linspace(0, 50, 100)
	mps = kph / 3.6
	fig, ax = plt.subplots(3)
	thermal = fixture(statorade=False)
	W = 1
	key = 'coils'
	# key = 'shell'		# plot shell to ambient
	source = {key: W}
	K = [thermal.replace(linear=v, circumferential=v/5*2).solve(source, dt=dt)[key] for v in mps]
	ax[0].plot(kph, W/np.array(K))
	thermal = fixture(statorade=True)
	K = [thermal.replace(linear=v, circumferential=v/5*2).solve(source, dt=dt)[key] for v in mps]
	ax[0].plot(kph, W/np.array(K))
	plt.show()


def test_plot_temporal():
	# https://electricbike-blog.com/2015/12/18/ferrofluids-join-the-ebike-motor-cooling-revolution/
	import matplotlib.pyplot as plt
	t = np.linspace(0, 5000, 100)
	fig, ax = plt.subplots(3)
	l, c = 10, 3
	# l, c = 0, 5
	thermal = fixture(statorade=False).replace(conductivity__linear=l, conductivity__circumferential=c)
	res = [list(thermal.solve({'coils': 95}, dt=dt).values()) for dt in t]
	ax[0].plot(t, np.array(res) + 18)
	thermal = fixture(statorade=True).replace(conductivity__linear=l, conductivity__circumferential=c)
	res = [list(thermal.solve({'coils': 95}, dt=dt).values()) for dt in t]
	ax[1].plot(t, np.array(res) + 22)
	thermal = fixture(vented=True).replace(conductivity__linear=l, conductivity__circumferential=c)
	res = [list(thermal.solve({'coils': 95}, dt=dt).values()) for dt in t]
	ax[2].plot(t, np.array(res) + 22)
	plt.figlegend(thermal.keys)
	plt.show()


