mc_panda_lirmm
========

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/mc_panda_lirmm/workflows/CI%20of%20mc_panda_lirmm/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_panda_lirmm/actions?query=workflow%3A%22CI+of+mc_panda_lirmm%22)

This package extends the [mc_panda] robot modules and provides modules specific to the panda robots in `LIRMM`. Notably these include the supports on which the robots are attached.

Naming convention
===

`mc_panda` provides the following modules:
- `PandaDefault`
- `PandaHand`
- `PandaFoot`
- `PandaPump`

In addition this package provides modules for the robots `Panda2`, `Panda5` and `Panda7` (each corresponding to one of the panda robots at `LIRMM`). Robot modules are named `<RobotName>LIRMM<End Effector>`.
Since the robots' estimated force sensor is left handed, and most simulators do not support this, for each robot module two variants are further provided:
- `<RobotName>LIRMM<End Effector>` : flips the sensor measurement along one axis such that it becomes right handed. This is intended to be used on the real robot.
- `<RobotName>LIRMM<End Effector>Simulation` : keeps the force sensor measurement as-is. This is intended to be used in simulators where the force sensor measurement is already right-handed.

For example you can use `Panda7LIRMMDefault`, `Panda7LIRMMHand`, `Panda7LIRMMFoot`, `Panda7LIRMMPump`, and their corresponding simulation variants `Panda7LIRMMDefaultSimulation`, `...`

Panda2
===
Panda2 model
![panda2](doc/panda2.png)

Panda2 convex
![panda2 convex](doc/panda2_convex.png)

Panda2 basic convex shapes
![panda2 convex shapes](doc/panda2_shapes_convex.png)

Panda5
===

Panda5 model
![panda5](doc/panda5.png)

Panda5 convex
![panda5 convex](doc/panda5_convex.png)

Panda5 basic convex shapes
![panda5 convex shapes](doc/panda5_shapes_convex.png)

Panda7
===

Panda7 model
![panda7](doc/panda7.png)

Panda7 convex
![panda7 convex](doc/panda7_convex.png)

Panda7 basic convex shapes
![panda7 convex shapes](doc/panda7_shapes_convex.png)

Dependencies
------------

This package requires:
- [mc_panda]
