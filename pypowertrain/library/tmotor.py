from pypowertrain.components.motor import Electrical, Mass, Motor, shelled_thermal
from pypowertrain.components.motor.geometry import Outrunner


def tmotor_u8():
	"""https://store.tmotor.com/product/u8-v2-u-efficiency-kv150.html"""
	geometry = Outrunner.create(
		pole_pairs=21,
		slot_triplets=12,
		turns=5,

		gap_radius=35e-3,
		gap_length=12e-3,
		slot_depth_fraction=0.20,
		slot_width_fraction=0.5,

		magnet_height=1.5e-3,
		airgap=0.5e-3,
		structure_thickness=1e-3,
	)

	electrical = Electrical.from_absolute(
		geometry=geometry,
		Kv_ll=150,
		R_ll=43e-3*2,
		L_ll=20e-6*2,
	)

	mass = Mass.from_absolute(geometry=geometry, total=0.273)
	# print(mass.get_attrs())
	return Motor(
		electrical=electrical,
		thermal=shelled_thermal(mass, statorade=0.0, vented=1.0),
		mass=mass,
	)
