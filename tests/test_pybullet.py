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

from robot_descriptions._descriptions import DESCRIPTIONS
from robot_descriptions.loaders.pybullet import load_robot_description


class TestPyBullet(unittest.TestCase):

    """
    Check that all descriptions are loaded properly in PyBullet.
    """

    def setUp(self):
        """
        Start PyBullet simulation.
        """
        pybullet.connect(pybullet.DIRECT)

    def tearDown(self):
        """
        Stop PyBullet simulation.
        """
        pybullet.disconnect()

    def test_urdf_descriptions(self):
        """
        Check all URDF descriptions.
        """
        for name, desc in DESCRIPTIONS.items():
            if desc.has_urdf:
                print(name)
                load_robot_description(name)


if __name__ == "__main__":
    unittest.main()
