# imetro_roboplan

Utilities based on the [RoboPlan](https://github.com/open-planning/roboplan) motion planning library for [iMETRO](https://github.com/NASA-JSC-Robotics/imetro).

## Getting Started

This repo uses [Pixi](https://pixi.prefix.dev/latest/) to manage the environment, but can be brought into other projects as a regular set of ROS packages.

To get started,

1. Install Pixi: <https://pixi.prefix.dev/latest/installation/>

2. `pixi run build`

Then, you can open up a `pixi shell` and do all your development and testing there!

## Packages Description

* [imetro_roboplan](src/imetro_roboplan): RoboPlan motion planning server.
* [imetro_roboplan_msgs](src/imetro_roboplan_msgs): Service definitions for the planning server.
* [imetro_roboplan_behaviors](src/imetro_roboplan_behaviors): [imetro_behavior](https://github.com/NASA-JSC-Robotics/imetro_behavior) nodes and subtrees that use the planning server.
