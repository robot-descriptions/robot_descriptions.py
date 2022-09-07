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
Show a robot descriptions specified from the command line, using yourdfpy.

Note:
    See ``robot_descriptions/_command_line.py`` for a more advanced
    implementation, including the ability to set the robot configuration or
    show collision meshes.

This example requires `yourdfpy` which is an optional dependency. It can be
installed separately (``pip install yourdfpy``), or when robot descriptions are
installed via ``pip install robot_descriptions[cli]``.
"""

import argparse
from importlib import import_module  # type: ignore

try:
    import yourdfpy  # pylint: disable=import-error
except ImportError as e:
    raise ImportError(
        "yourdfpy not found, try ``pip install yourdfpy``"
    ) from e


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        module = import_module(f"robot_descriptions.{args.name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{args.name}_description")

    robot = yourdfpy.URDF.load(module.URDF_PATH)
    robot.show()
