#!/usr/bin/env python
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


import unittest

import pybullet
from robot_descriptions.loaders.idyntree import (
    load_robot_description as load_idyntree,
)
from robot_descriptions.loaders.mujoco import (
    load_robot_description as load_mujoco,
)
from robot_descriptions.loaders.pinocchio import (
    load_robot_description as load_pinocchio,
)
from robot_descriptions.loaders.pybullet import (
    load_robot_description as load_pybullet,
)
from robot_descriptions.loaders.robomeshcat import (
    load_robot_description as load_robomeshcat,
)
from robot_descriptions.loaders.yourdfpy import (
    load_robot_description as load_yourdfpy,
)


class TestLoaders(unittest.TestCase):

    """
    Test loaders.
    """

    upkie_description_commit = "98502d5b175c3d6b60b3cf475b7eeef9fd290c43"

    def test_mujoco(self):
        self.assertIsNotNone(load_mujoco("cassie_mj_description"))

    def test_pinocchio(self):
        self.assertIsNotNone(
            load_pinocchio(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )

    def test_pybullet(self):
        pybullet.connect(pybullet.DIRECT)
        self.assertIsNotNone(
            load_pybullet(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )
        pybullet.disconnect()

    def test_robomeshcat(self):
        self.assertIsNotNone(
            load_robomeshcat(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )

    def test_yourdfpy(self):
        self.assertIsNotNone(
            load_yourdfpy(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )

    def test_idyntree(self):
        self.assertIsNotNone(
            load_idyntree(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )
