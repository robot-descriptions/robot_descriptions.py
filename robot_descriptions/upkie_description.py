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

import os as _os

from .git import git_clone_to_descriptions_dir as _git_clone_to_descriptions_dir

__version__ = "0.1.0"


__repo = git_clone_to_descriptions_dir(
    "https://github.com/tasts-robots/upkie_description.git"
)

MESHES_PATH = None
if __repo.working_dir is not None:
    MESHES_PATH = os.path.join(__repo.working_dir, "meshes")

URDF_PATH = None
if __repo.working_dir is not None:
    URDF_PATH = os.path.join(__repo.working_dir, "urdf", "upkie.urdf")


def get_repository():

__all__ = [
    "MESHES_PATH",
    "URDF_PATH",
]
