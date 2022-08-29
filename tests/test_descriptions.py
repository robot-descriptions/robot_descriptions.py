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

from robot_descriptions import (
    edo_description,
    kinova_description,
    panda_description,
    upkie_description,
    ur3_description,
    ur5_description,
)


class TestDescriptions(unittest.TestCase):

    """
    Test fixture for all robot descriptions.
    """

    def test_upkie_description(self):
        """
        Check all imported submodules.
        """
        descriptions = [
            edo_description,
            kinova_description,
            panda_description,
            upkie_description,
            ur3_description,
            ur5_description,
        ]
        for description in descriptions:
            self.assertNotEqual(description.PATH, "")
            self.assertNotEqual(description.URDF_PATH, "")


if __name__ == "__main__":
    unittest.main()
