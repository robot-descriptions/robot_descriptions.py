#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""iiwa 14 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "drake",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "manipulation", "models", "iiwa_description"
)

URDF_PATH: str = _path.join(
    PACKAGE_PATH, "urdf", "iiwa14_primitive_collision.urdf"
)

# Description-specific paths

URDF_PATH_NO_COLLISION: str = _path.join(
    PACKAGE_PATH, "urdf", "iiwa14_no_collision.urdf"
)

URDF_PATH_POLYTOPE_COLLISION: str = _path.join(
    PACKAGE_PATH, "urdf", "iiwa14_polytope_collision.urdf"
)

URDF_PATH_PRIMITIVE_COLLISION: str = _path.join(
    PACKAGE_PATH, "urdf", "iiwa14_primitive_collision.urdf"
)
