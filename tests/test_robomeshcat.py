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

try:
    from robot_descriptions.loaders.robomeshcat import load_robot_description

    class TestRoboMeshCat(unittest.TestCase):

        """
        Check that all descriptions are loaded properly in RoboMeshCat.
        """

        # def test_load_robot_description(self):
        #     """
        #     Load a robot description with RoboMeshCat.

        #     Since RoboMeshCat relies on Pinocchio, we only test it on a couple
        #     of robot descriptions.
        #     """
        #     mini_cheetah = load_robot_description("mini_cheetah_description")
        #     self.assertIsNotNone(mini_cheetah)

        def test_description_with_cylinders(self):
            """
            Load a robot description with cylinder shapes.
            """
            upkie = load_robot_description("upkie_description")
            self.assertIsNotNone(upkie)

        # def test_collada_description(self):
        #     """
        #     Test a robot description with Collada meshes.
        #     """
        #     r2 = load_robot_description("r2_description")
        #     self.assertIsNotNone(r2)


except ImportError:

    pass
