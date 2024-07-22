from pypowertrain.library import cubemars, grin, odrive, moteus


def test_compare():
	"""
	Motors are all quite close in their dimensionless attributes,
	which validates our scaling laws

	R_co values in particular should be quite close;
	insofar as they differ, our estimate of the slot area or copper fill factor must be off

	L values should be harder to pin down; they depend on iron saturation or fringing effects,
	which might vary quite a bit between motors and which is not covered in our modelling

	dimensionless Kt values especially may vary; the magnetic flux scaling we estimate
	is only valid as-linearized around the measured motor, and highly dependent on iron saturation,
	amongst other things. Putting more magnet volume on a motor typically does not do much
	to increase measured torque, but it will drop our dimensionless Kt which is normalized to magnet volume,
	amongst other things.

	"""
	print()
	print(grin.all_axle(turns=5).electrical.attrs)
	print(grin.all_axle(turns=8).electrical.attrs)
	print(odrive.botwheel().electrical.attrs)
	print(odrive.M8325s_100KV().electrical.attrs)
	print(moteus.mj5208().electrical.attrs)
	print(cubemars.RO100().electrical.attrs)

