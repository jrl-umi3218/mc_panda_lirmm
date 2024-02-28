mc_panda_lirmm
========

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/mc_panda_lirmm/workflows/CI%20of%20mc_panda_lirmm/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_panda_lirmm/actions?query=workflow%3A%22CI+of+mc_panda_lirmm%22)

This package extends the [mc_panda] robot modules and provides modules specific to the panda robots in LIRMM. Notably these include the supports on which the robots are attached.
The following robots are available:
- `Panda2LIRMM/Panda5LIRMM/Panda7LIRMM`: This corresponds to `PandaDefault` with the supports on which the robots are mounted (table, metal stand). This is intended to be used on the real robot (flipped force sensor)
- `Panda2LIRMM::Simulation/Panda5LIRMM::Simulation/Panda7LIRMM::Simulation`: This corresponds to `PandaDefault` with the supports on which the robots are mounted (table, metal stand)

Dependencies
------------

This package requires:
- [mc_panda]
