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

from robot_descriptions._descriptions import DESCRIPTIONS


class TestDescriptions(unittest.TestCase):

    """
    Test fixture for all robot descriptions.
    """

    def test_all_descriptions(self):
        """
        Check all robot-description submodules.
        """
        for name, desc in DESCRIPTIONS.items():
            description = import_module(f"robot_descriptions.{name}")
            self.assertTrue(
                os.path.exists(description.REPOSITORY_PATH),
                f"Path {description.REPOSITORY_PATH} "
                f"does not exist in {description}",
            )
            self.assertTrue(
                os.path.exists(description.PACKAGE_PATH),
                f"Path {description.PACKAGE_PATH} "
                f"does not exist in {description}",
            )
            if desc.has_mjcf:
                self.assertTrue(hasattr(description, "MJCF_PATH"))
                self.assertTrue(
                    os.path.exists(description.MJCF_PATH),
                    f"MJCF path {description.MJCF_PATH} does not exist "
                    f"in {description}",
                )
            if desc.has_urdf:
                self.assertTrue(hasattr(description, "URDF_PATH"))
                self.assertTrue(
                    os.path.exists(description.URDF_PATH),
                    f"URDF path {description.URDF_PATH} does not exist "
                    f"in {description}",
                )
