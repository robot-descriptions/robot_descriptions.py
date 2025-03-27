#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Ability Hand MJCF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "ability-hand-api",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "python", "ah_simulators", "mujoco_xml"
)

MJCF_PATH: str = _path.join(PACKAGE_PATH, "scene.xml")

# Description-specific paths

MJCF_PATH_LEFT_LARGE: str = _path.join(
    PACKAGE_PATH, "hands", "abh_left_large.xml"
)

MJCF_PATH_LEFT_SMALL: str = _path.join(
    PACKAGE_PATH, "hands", "abh_left_small.xml"
)

MJCF_PATH_RIGHT_LARGE: str = _path.join(
    PACKAGE_PATH, "hands", "abh_right_large.xml"
)

MJCF_PATH_RIGHT_SMALL: str = _path.join(
    PACKAGE_PATH, "hands", "abh_right_small.xml"
)
