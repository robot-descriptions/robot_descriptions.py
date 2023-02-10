#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Robonaut 2 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "nasa-urdf-robots",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "r2_description")

URDF_PATH: str = _path.join(
    PACKAGE_PATH, "robots", "r2c_sim_full_control.urdf.urdf"
)

# Description-specific paths

URDF_PATH_B: str = _path.join(PACKAGE_PATH, "robots", "r2b.urdf")
URDF_PATH_C1: str = _path.join(PACKAGE_PATH, "robots", "r2c1.urdf")
URDF_PATH_C5: str = _path.join(PACKAGE_PATH, "robots", "r2c5.urdf")
URDF_PATH_C6: str = _path.join(PACKAGE_PATH, "robots", "r2c6.urdf")

URDF_PATH_B_CONTROL: str = _path.join(
    PACKAGE_PATH, "robots", "r2b_control.urdf"
)
URDF_PATH_C1_CONTROL: str = _path.join(
    PACKAGE_PATH, "robots", "r2c1_control.urdf"
)
URDF_PATH_C5_CONTROL: str = _path.join(
    PACKAGE_PATH, "robots", "r2c5_control.urdf"
)
URDF_PATH_C6_CONTROL: str = _path.join(
    PACKAGE_PATH, "robots", "r2c6_control.urdf"
)

URDF_PATH_B_DYNAMICS: str = _path.join(
    PACKAGE_PATH, "robots", "r2b_dynamics.urdf"
)
URDF_PATH_C1_DYNAMICS: str = _path.join(
    PACKAGE_PATH, "robots", "r2c1_dynamics.urdf"
)
URDF_PATH_C5_DYNAMICS: str = _path.join(
    PACKAGE_PATH, "robots", "r2c5_dynamics.urdf"
)
URDF_PATH_C6_DYNAMICS: str = _path.join(
    PACKAGE_PATH, "robots", "r2c6_dynamics.urdf"
)

URDF_PATH_C_SIM_FULL_CONTROL: str = _path.join(
    PACKAGE_PATH, "robots", "r2c_sim_full_control.urdf"
)
URDF_PATH_C_SIM_FULL_DYNAMICS: str = _path.join(
    PACKAGE_PATH, "robots", "r2c_sim_full_dynamics.urdf"
)
URDF_PATH_C_SIM_LEGS_CONTROL: str = _path.join(
    PACKAGE_PATH, "robots", "r2c_sim_legs_control.urdf"
)
URDF_PATH_C_SIM_LEGS_DYNAMICS: str = _path.join(
    PACKAGE_PATH, "robots", "r2c_sim_legs_dynamics.urdf"
)
URDF_PATH_C_SIM_UPPERBODY_CONTROL: str = _path.join(
    PACKAGE_PATH, "robots", "r2c_sim_upperbody_control.urdf"
)
URDF_PATH_C_SIM_UPPERBODY_DYNAMICS: str = _path.join(
    PACKAGE_PATH, "robots", "r2c_sim_upperbody_dynamics.urdf"
)
