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

In addition this package provides modules for the robots `Panda2`, `Panda6` and `Panda7` (each corresponding to one of the panda robots at `LIRMM`). Robot modules are named `<RobotName>LIRMM<End Effector>`.

For example you can use `Panda7LIRMMDefault`, `Panda7LIRMMHand`, `Panda7LIRMMFoot`, `Panda7LIRMMPump`.

Panda2
===


IP: 192.168.1.2

Panda2 model
![panda2](doc/panda2.png)

Panda2 convex
![panda2 convex](doc/panda2_convex.png)

Panda2 basic convex shapes
![panda2 convex shapes](doc/panda2_shapes_convex.png)

Panda6
===

IP: 172.16.0.6

panda6 model
![panda6](doc/panda6.png)

panda6 convex
![panda6 convex](doc/panda6_convex.png)

panda6 basic convex shapes
![panda6 convex shapes](doc/panda6_shapes_convex.png)

Panda7
===

IP: 172.16.1.7

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
