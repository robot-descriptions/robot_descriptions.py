#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Allegro Hand description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "drake",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "manipulation", "models", "allegro_hand_description"
)

URDF_PATH: str = _path.join(
    PACKAGE_PATH, "urdf", "allegro_hand_description_left.urdf"
)

# Description-specific paths

URDF_PATH_LEFT: str = _path.join(
    PACKAGE_PATH, "urdf", "allegro_hand_description_left.urdf"
)

URDF_PATH_RIGHT: str = _path.join(
    PACKAGE_PATH, "urdf", "allegro_hand_description_right.urdf"
)
