from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry
from pypowertrain.components.motor.electrical import Electrical

def fixture():
	geo = Geometry.create(
		poles=46,
		slots=42,
		turns=5,

		gap_diameter=199e-3,
		gap_length=27e-3,
		slot_depth_fraction=0.16,
		magnet_height=3e-3,
		airgap=0.8e-3,
	)

	elec = Electrical.from_absolute(
		geometry=geo,
		Kt_dq=1.0,
		R_ll=0.1,
		L_ll=0.2e-3,
		d0=0.45, d1=5e-4,
		saturation_nm=100,
	)
	return elec


def test_electrical():
	"""test basic nondimensionalization and scaling"""
	elec = fixture()
	print('Kt', elec.Kt)
	print('R', elec.R)
	print('L', elec.L)
	print('d', elec.iron_drag(200))
	elec = elec.replace(geometry__frequency_scale=2)
	print('Kt', elec.Kt)
	print('R', elec.R)
	print('L', elec.L)
	print('d', elec.iron_drag(200))

	# print('R', elec.replace(geometry__slot_depth_scale=0.5).R)


def test_saturation():
	"""visualize saturation curve"""
	elec = fixture()
	print(elec.Kt)
	print(elec.geometry.gap_length)
	# elec = elec.replace(geometry__length_scale=1/0.84)

	print(elec.Kt)
	print(elec.geometry.gap_length)
	print(elec.saturation)
	amps = np.linspace(0, 300)
	# return
	import matplotlib.pyplot as plt
	plt.plot(amps, amps * elec.Kt / elec.saturation_factor(amps))
	plt.show()
