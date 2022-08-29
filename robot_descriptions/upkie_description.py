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
Upkie description.
"""

__version__ = "1.1.0"

from os import path as _path

from .git import git_clone_description as _git_clone_description

MESHES_PATH: str = ""
PATH: str = ""
URDF_PATH: str = ""

__working_dir__ = _git_clone_description(
    "https://github.com/tasts-robots/upkie_description.git"
)

if __working_dir__ is not None:
    MESHES_PATH = _path.join(__working_dir__, "meshes")
    PATH = str(__working_dir__)
    URDF_PATH = _path.join(__working_dir__, "urdf", "upkie.urdf")
else:  # __working_dir__ is None
    raise ImportError("Git repository for the robot description is empty")
