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

import os
import unittest
from importlib import import_module  # type: ignore

from robot_descriptions._description_names import DESCRIPTION_NAMES


class TestDescriptions(unittest.TestCase):

    """
    Test fixture for all robot descriptions.
    """

    def test_all_descriptions(self):
        """
        Check all robot-description submodules.
        """
        for name in DESCRIPTION_NAMES:
            description = import_module(f"robot_descriptions.{name}")
            self.assertNotEqual(
                description.PATH, "", f"Empty PATH in {description}"
            )
            self.assertTrue(
                os.path.exists(description.PATH),
                f"Path {description.PATH} does not exist in {description}",
            )
            self.assertNotEqual(
                description.URDF_PATH, "", f"Empty URDF_PATH in {description}"
            )
            self.assertTrue(
                os.path.exists(description.URDF_PATH),
                f"URDF path {description.URDF_PATH} does not exist "
                f"in {description}",
            )
            if description.MESHES_PATH != "":
                self.assertTrue(
                    os.path.exists(description.MESHES_PATH),
                    f"Meshes path {description.MESHES_PATH} does not exist "
                    f"in {description}",
                )


if __name__ == "__main__":
    unittest.main()
