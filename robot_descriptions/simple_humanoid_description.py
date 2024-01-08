#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

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

URDF_PATH_CLASSICAL: str = _path.join(
    PACKAGE_PATH, "urdf", "simple_humanoid_classical.urdf"
)
