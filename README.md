
pypowertrain
=========

Summary
-------
This package implements functionality for the simulation and optimization of electric powertrains with field-oriented controllers. It seeks to be a repository of well-characterized components, making them easy to evaluate.



This package has functionality for optimizing over components of a powertrain, such as motor geometry, to attain certain design constraints, while minimizing objectives such as weight or cost. Currently the only physical loads implemented are vehicles, but it is designed to be easy to extend to robotic actuators, quadcopters, wind turbines, or you name it.

The approach to design optimization in pypowertrain relies on applying simple scaling laws to existing, empirically verified components. Doing ab-initio calculations has its use; but it does not answer question like 'can the manufacturer actually wind these wires in that tooth geometry' and 'is the saturation going to cause audible harmonics that cross into the annoying-region'; and so on. 

There are some simple scaling laws that can inform us of what ought to be possible. For instance, we can scale up a motor by placing two on the same axle and phase wire; and we know that this is a conservative estimate of the resulting scaled motor, since in practice we would cancel out the coil overhang. Decreases in tooth depth are similarly predictable, as are changes of the number of turns. With only tweaks to a few such parameters, you can cover a very large part of design space, starting from a few empirically well characterized components. The current implementation to such motor dimensional scaling uses [this paper]() as a starting point.

This library is inspired by grins' [motor simulator](https://ebikes.ca/tools/simulator.html), with differences being that this is a modular programmable open source library, with somewhat more general aims. For equal inputs, our results are almost-but-not-quite the same those of the above simulator. If you can help us narrow down the gap, contributions are welcome. 

What would also be highly appreciated are empirical test cases of whole powertrains with known components, that the resuts of pypowertrain can be compared to.