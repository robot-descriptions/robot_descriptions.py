#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Kinova Gen3 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "ros2_kortex",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "kortex_description")

XACRO_PATH: str = _path.join(PACKAGE_PATH, "robots", "gen3.xacro")

XACRO_ARGS = {
    "dof": "7",
}

SRDF_PATH_7DOF_ROBOTIQ_2F_85: str = _path.join(
    REPOSITORY_PATH,
    "kortex_moveit_config",
    "kinova_gen3_7dof_robotiq_2f_85_moveit_config",
    "config",
    "gen3.srdf",
)

SRDF_PATH_6DOF_ROBOTIQ_2F_85: str = _path.join(
    REPOSITORY_PATH,
    "kortex_moveit_config",
    "kinova_gen3_6dof_robotiq_2f_85_moveit_config",
    "config",
    "gen3.srdf",
)
