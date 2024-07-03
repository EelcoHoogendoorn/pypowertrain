from pypowertrain.library import grin


def test_drive_actuator():
	actuator = grin.actuator(turns=5)
	print(actuator.weight)
	print(actuator.peak_torque)
	actuator = actuator.replace(motor__turns=4/5)
	print(actuator.peak_torque)
	actuator.plot()
	import matplotlib.pyplot as plt
	plt.show()
