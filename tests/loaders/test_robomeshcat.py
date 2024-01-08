#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import unittest

from robot_descriptions._descriptions import DESCRIPTIONS
from robot_descriptions.loaders.robomeshcat import load_robot_description


class TestRoboMeshCat(unittest.TestCase):

    """
    Check that all descriptions are loaded properly in RoboMeshCat.
    """

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
            load_robot_description(description)

        return test


# Add a test function for each URDF description
for name, description in DESCRIPTIONS.items():
    if description.has_urdf:
        setattr(
            TestRoboMeshCat,
            f"test_{name}",
            TestRoboMeshCat.get_test_for_description(name),
        )
