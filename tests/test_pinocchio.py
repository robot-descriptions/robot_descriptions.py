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

from robot_descriptions._descriptions import DESCRIPTIONS

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description

    class TestPinocchio(unittest.TestCase):

        """
        Check that all descriptions are loaded properly in Pinocchio.
        """

        def setUp(self):
            logging.basicConfig()
            logging.getLogger().setLevel(logging.INFO)

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
                logging.info(f"Loading {description} in Pinocchio...")
                load_robot_description(description)

            return test

    # Add a test function for each URDF description
    for name, description in DESCRIPTIONS.items():
        if description.has_urdf:
            setattr(
                TestPinocchio,
                f"test_{name}",
                TestPinocchio.get_test_for_description(name),
            )

except ImportError:

    pass
