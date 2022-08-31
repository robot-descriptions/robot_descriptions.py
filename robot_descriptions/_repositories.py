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
Git utility functions to clone model repositories.
"""

from dataclasses import dataclass


@dataclass
class Repository:

    """
    Remote git repository.

    Attributes:
        cache_path: Path to clone the repository to in the local cache.
        commit: Commit ID or tag to checkout after cloning.
        url: URL to the remote git repository.
    """

    cache_path: str
    commit: str
    url: str


REPOSITORIES = {
    "baxter_common": Repository(
        url="https://github.com/RethinkRobotics/baxter_common.git",
        commit="v1.2.0",
        cache_path="RethinkRobotics/baxter_common",
    ),
    "cassie_description": Repository(
        url="https://github.com/UMich-BipedLab/cassie_description.git",
        commit="96323f3d0cc2cb7101cb92d7fcf4650abdcb2e81",
        cache_path="UMich-BipedLab/cassie_description",
    ),
    "drake": Repository(
        url="https://github.com/RobotLocomotion/drake.git",
        commit="v1.7.0",
        cache_path="RobotLocomotion/drake",
    ),
    "eDO_description": Repository(
        url="https://github.com/Comau/eDO_description.git",
        commit="17b3f92f834746106d6a4befaab8eeab3ac248e6",
        cache_path="Comau/eDO_description",
    ),
    "example-robot-data": Repository(
        url="https://github.com/Gepetto/example-robot-data.git",
        commit="v4.0.1",
        cache_path="Gepetto/example-robot-data",
    ),
    "gym-pybullet-drones": Repository(
        url="https://github.com/utiasDSL/gym-pybullet-drones.git",
        commit="v1.0.0",
        cache_path="utiasDSL/gym-pybullet-drones",
    ),
    "jvrc_description": Repository(
        url="https://github.com/stephane-caron/jvrc_description.git",
        commit="v1.1.0",
        cache_path="stephane-caron/jvrc_description",
    ),
    "upkie_description": Repository(
        url="https://github.com/tasts-robots/upkie_description.git",
        commit="v1.1.0",
        cache_path="tasts-robots/upkie_description",
    ),
}
