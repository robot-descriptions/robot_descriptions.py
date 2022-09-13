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

import logging
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

    def test_value_error_when_no_urdf(self):
        """
        Test exception raised when a description has no URDF_PATH.
        """
        with self.assertRaises(ValueError):
            load_robot_description("_empty_description")

    @staticmethod
    def get_test_for_description(description: str):
        """
        Get test function for a given description.

        Args:
            description: Name of the description.

        Returns:
            Test function for that description.
        """

        def test(self):
            logging.debug(f"Loading {description} in PyBullet...")
            load_robot_description(description)

        return test


for name, description in DESCRIPTIONS.items():
    if description.has_urdf:
        setattr(
            TestPyBullet,
            f"test_{name}",
            TestPyBullet.get_test_for_description(name),
        )
