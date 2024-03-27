#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Valkyrie description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "nasa-urdf-robots",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "val_description")

URDF_PATH: str = _path.join(
    PACKAGE_PATH, "model", "robots", "valkyrie_sim.urdf"
)

# Description-specific paths

URDF_PATH_A: str = _path.join(
    PACKAGE_PATH, "model", "robots", "valkyrie_A.urdf"
)

URDF_PATH_B: str = _path.join(
    PACKAGE_PATH, "model", "robots", "valkyrie_B.urdf"
)

URDF_PATH_C: str = _path.join(
    PACKAGE_PATH, "model", "robots", "valkyrie_C.urdf"
)
