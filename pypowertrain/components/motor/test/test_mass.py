from pypowertrain.utils import *
from pypowertrain.components.motor.geometry import Outrunner
from pypowertrain.components.motor.mass import Mass


def fixture():
	geo = Outrunner.create(
		poles=46,
		slots=42,
		turns=5,

		gap_radius=199e-3 / 2,
		gap_length=27e-3,
		slot_depth_fraction=0.16,
		magnet_height=3e-3,
		airgap=0.8e-3,
	)

	mass = Mass.from_absolute(
		geometry=geo,
		total=4.0
	)
	return mass


def test_mass():
	mass = fixture()
	print()
	print(mass.total)
	print(mass.coils)
	print(mass.rotor_inertia)
	mass = mass.replace(geometry__slot_depth_scale=0.5)
	print(mass.coils)
	print(mass.rotor_inertia)
	print(mass.total)
	mass = mass.replace(geometry__frequency_scale=0.5)
	print(mass.total)


def test_util():
	mass = fixture()
	print(list(expand_paths('turns', mass)))
	print(list(expand_paths('.turns', mass)))
	print(list(expand_paths('geometry.turns', mass)))
	print(list(expand_paths('.geometry.turns', mass)))
	print(list(expand_paths('.geometry.slot_depth_scale', mass)))
