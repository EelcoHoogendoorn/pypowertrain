from pypowertrain.components.motor.geometry import Geometry
from pypowertrain.components.motor.electrical import Electrical
from pypowertrain.components.motor.motor import Motor


def test_motor():
	geo = Geometry(
		poles=46,
		slots=42,
		turns=5,

		gap_radius=199e-3 / 2,
		gap_length=27e-3,
		slot_depth_fraction=0.16,
		magnet_height=3e-3,
		airgap=0.8e-3,
	)
	# geo.replace(slot_width_fraction=0.4).plot()
	print(geo)
	electrical = Electrical.from_absolute(
		geometry=geo,
		Kt=1.0,
		phase_to_phase_R=200e-3,
		phase_to_phase_L=400e-6,
		d0=0.45,
		d1=0.0005,
		salience=0,
	)

	motor = Motor(electrical=electrical)
	print(motor.geometry.gap_length)
	print(motor.electrical.Lq)
	print(motor.electrical.R)
	motor = motor.replace(__geometry__gap_length=30e-3)
	print(motor.electrical.geometry.gap_length)
	print(motor.electrical.Lq)
	print(motor.electrical.R)
