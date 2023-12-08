#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

"""TALOS description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "talos-data",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH)

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "talos_full.urdf")

# Description-specific paths

URDF_PATH_V2: str = _path.join(PACKAGE_PATH, "urdf", "talos_full_v2.urdf")

URDF_PATH_REDUCED: str = _path.join(PACKAGE_PATH, "urdf", "talos_reduced.urdf")

URDF_PATH_REDUCED_V2: str = _path.join(
    PACKAGE_PATH, "urdf", "talos_reduced_v2.urdf"
)
