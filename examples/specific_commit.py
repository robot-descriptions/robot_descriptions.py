#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""
We can load specific commit to pin the version of any robot description using
the `commit` keyword argument. In this example, we load a Universal Robots UR5e
from a specific commit of the MuJoCo Menagerie repository that distributes it.
This example requires MuJoCo, which is installed by `pip install mujoco`.

See also:
    https://github.com/robot-descriptions/robot_descriptions.py/discussions/86
"""

from robot_descriptions.loaders.mujoco import load_robot_description

if __name__ == "__main__":
    model = load_robot_description(
        "ur5e_mj_description",
        commit="cc3868ec21bae324e6dd3b01ea185b6e6f0fa172",
    )
    print(f"UR5e successfully loaded in {model=}")
