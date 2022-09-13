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
from robot_descriptions.loaders.pinocchio import load_robot_description


class TestPinocchio(unittest.TestCase):

    """
    Check that all descriptions are loaded properly in Pinocchio.

    Note:
        Some descriptions won't pass until
        https://github.com/stack-of-tasks/pinocchio/pull/1742 hits release.
    """

    def setUp(self):
        logging.basicConfig()
        logging.getLogger().setLevel(logging.DEBUG)

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
            logging.debug(f"Loading {description} in Pinocchio...")
            load_robot_description(description)

        return test


USE_RELATIVE_PATHS = {
    "cf2_description",
    "laikago_description",
    "mini_cheetah_description",
    "minitaur_description",
    "pr2_description",
}

VALUE_ERROR_ISSUE = {
    "anymal_b_description",
    "bolt_description",
    "finger_edu_description",
    "solo_description",
}

MALLOC_ISSUE = {
    "talos_description",
}

for name, description in DESCRIPTIONS.items():
    if name in USE_RELATIVE_PATHS:
        # Those descriptions won't pass until the following fix hits release:
        # Issue: https://github.com/stack-of-tasks/pinocchio/issues/1741
        # PR: https://github.com/stack-of-tasks/pinocchio/pull/1742
        continue
    if name in VALUE_ERROR_ISSUE:
        # ValueError: Argument geometry_model should be a GeometryModel
        # This issue seems to be fixed in shortcuts.py of pin-2.6.9
        # TODO(scaron): I'm not sure why these descriptions fail to load in the
        # CI while they load fine on my machine locally (py38, pin-2.6.4)
        continue
    if name in MALLOC_ISSUE:
        # malloc(): invalid size (unsorted)
        # TODO(scaron): I'm not sure why these descriptions fail to load in the
        # CI while they load fine on my machine locally (py38, pin-2.6.4)
        continue
    if description.has_urdf:
        setattr(
            TestPinocchio,
            f"test_{name}",
            TestPinocchio.get_test_for_description(name),
        )
