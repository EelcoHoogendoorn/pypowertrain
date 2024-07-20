from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Geometry
from pypowertrain.components.motor.electrical import Electrical


def test_electrical():
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
		Kt=1.0,
		phase_to_phase_R=0.1,
		phase_to_phase_L=0.2e-3,
		d0=0.45, d1=5e-4,
	)
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
