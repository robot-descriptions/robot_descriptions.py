#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Simple Humanoid description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "simple_humanoid_description",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH)

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "simple_humanoid.urdf")

SRDF_PATH: str = _path.join(PACKAGE_PATH, "srdf", "simple_humanoid.srdf")

URDF_PATH_CLASSICAL: str = _path.join(
    PACKAGE_PATH, "urdf", "simple_humanoid_classical.urdf"
)

SRDF_PATH_CLASSICAL: str = _path.join(
    PACKAGE_PATH, "srdf", "simple_humanoid_classical.srdf"
)
