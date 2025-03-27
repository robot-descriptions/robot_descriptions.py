#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Ability Hand description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "ability-hand-api",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH,
    "URDF",
)

URDF_PATH: str = _path.join(PACKAGE_PATH, "ability_hand_left_large.urdf")

# Description-specific paths

URDF_PATH_LEFT_LARGE: str = _path.join(
    PACKAGE_PATH, "ability_hand_left_large.urdf"
)

URDF_PATH_LEFT_SMALL: str = _path.join(
    PACKAGE_PATH, "ability_hand_left_small.urdf"
)

URDF_PATH_RIGHT_LARGE: str = _path.join(
    PACKAGE_PATH, "ability_hand_right_large.urdf"
)

URDF_PATH_RIGHT_SMALL: str = _path.join(
    PACKAGE_PATH, "ability_hand_right_small.urdf"
)
