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

This example requires MuJoCo, which is installed by ``pip install mujoco``.
"""

import argparse
from importlib import import_module  # type: ignore

try:
    import mujoco
except ImportError as e:
    raise ImportError("MuJoCo not found, try ``pip install mujoco``") from e

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        module = import_module(f"robot_descriptions.{args.name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{args.name}_description")

    if not hasattr(module, "MJCF_PATH"):
        raise ValueError(f"{args.name} is not an MJCF description")

    model = mujoco.MjModel.from_xml_path(module.MJCF_PATH)
    data = mujoco.MjData(model)

    print(f"Robot successfully loaded with model={model} and data={data}")
