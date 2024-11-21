
pypowertrain
=========

[Get started with a runnable notebook!](https://mybinder.org/v2/gh/EelcoHoogendoorn/pypowertrain/HEAD?labpath=getting_started.ipynb)

Summary
-------
This package implements functionality for the simulation and optimization of electric powertrains with field-oriented controllers. It seeks to be a repository of well-characterized components, making their performance easy to evaluate in combination.

```python
def test_example(kmh=40, inch=20):
	"""Simple minimal example"""
	bike = BikeSystem(
		actuator=Actuator(
			motor=grin.all_axle(turns=8),
			controller=odrive.pro(),
		),
		battery=define_battery(v=58, wh=500),
		load=BikeLoad(
			CdA=0.7 * 0.8,
			Cr=0.004,
			structure_weight=40,
			rider_weight=80,
			nominal_kmh=kmh,
			wheel_diameter=inch * 25.4e-3,
			front=True, rear=True,
		),
	)

	system_plot(bike, annotations='tdeosa')
```
<img src="./grin_with_odrive.png" width="400" height="300" />

This package has functionality for [optimizing](./pypowertrain/test/test_optimize.py) over all components of a powertrain, such as motor geometry, battery configuration or controller parameters, to attain certain design constraints, while minimizing objectives such as weight or cost. Currently, the only physical systems implemented are vehicles, but it is designed to be easy to extend to robotic actuators, quadcopters, wind turbines, or you name it.

The approach to motor design optimization relies on applying simple scaling laws to existing, empirically verified components. Doing ab-initio calculations has its use; but it is hard to make it capture all real world relevant effect and manufacturing constraints. 

The simplest example of a scaling law is rewinding with a different number of turns, but a number of dependable scaling laws are known. With only tweaks to a few such parameters, you can cover a very large part of design space, starting from a few empirically well characterized components. The current implementation of motor dimensional scaling uses [this paper](https://www.researchgate.net/publication/283646083_Scaling_laws_for_synchronous_permanent_magnet_machines) as a starting point to scale the electrical parameters with the geometrical parameters. These scaling laws are exact. Several more approximate scaling laws are implemented in pypowertrain, notably slot depth scaling and magnet thickness scaling; plus we also apply scaling laws to the magnetic limits, not just the electrical ones. More user awareness as to the limitations of the approximations therein is advised, should you seek to scale those parameters.

Note that pypowertrain makes heavy use of nondimensional defaults for many parameters. In fact there isnt a single motor in the library for which all required parameters are actually available. Good examples of this are iron loss coefficients and core saturation limits, which are rarely given explicitly by manufacturers. Rather than assuming the motors for which no such limit is stated do not have one, we elect to give a default value in nondimensional units. This tends to give ballpark reasonable results, but be advised it is not a substitute for an actual measurement. Also note that the thermal models specified in the library are of highly variable quality. The grin motor thermal models are the highest quality, since this is the only manufacturer that has published detailed data on thermals.

This library is inspired by grins' [motor simulator](https://ebikes.ca/tools/simulator.html), with the main difference being that this is a modular programmable open source library, with somewhat more general aims. For equal inputs, our results are [almost-but-not-quite](./pypowertrain/bike/test/test_grin.py) the same those of the above simulator. If you can help us narrow down the gap, contributions are welcome! 

What would also be highly appreciated are other empirical test cases of whole powertrains with known components, that the results of pypowertrain can be compared to.