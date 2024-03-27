#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Atlas DRC (v3) description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "drake",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "examples", "atlas")

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "atlas_convex_hull.urdf")

# Description-specific paths

URDF_PATH_MINIMUM_CONTACT: str = _path.join(
    PACKAGE_PATH, "urdf", "atlas_minimum_contact.urdf"
)
