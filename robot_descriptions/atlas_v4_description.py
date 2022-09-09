#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Atlas v4 description.
"""

from os import path as _path
from typing import List

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache("roboschool")

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
