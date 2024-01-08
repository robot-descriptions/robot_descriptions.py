#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Atlas v4 description."""

from os import getenv as _getenv
from os import path as _path
from typing import List

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "roboschool",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "roboschool", "models_robot", "atlas_description"
)

MESH_PATHS: List[str] = [
    _path.join(PACKAGE_PATH, "meshes_v3"),
    _path.join(PACKAGE_PATH, "meshes_unplugged"),
    _path.join(
        REPOSITORY_PATH,
        "roboschool",
        "models_robot",
        "multisense_sl_description",
        "meshes",
    ),
]

URDF_PATH: str = _path.join(
    PACKAGE_PATH, "urdf", "atlas_v4_with_multisense.urdf"
)
