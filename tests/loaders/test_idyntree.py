#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Giulio Romualdi

import unittest

from robot_descriptions._descriptions import DESCRIPTIONS
from robot_descriptions.loaders.idyntree import (
    load_robot_description as load_idyntree,
)


class TestiDynTree(unittest.TestCase):
    """
    Check that all descriptions are loaded properly in iDynTree.
    """

    def test_idyntree(self):
        self.assertIsNotNone(
            load_idyntree(
                "upkie_description",
                commit="98502d5b175c3d6b60b3cf475b7eeef9fd290c43",
            )
        )

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
            load_idyntree(description)

        return test


# Add a test function for each URDF description
for name, description in DESCRIPTIONS.items():
    if description.has_urdf:
        setattr(
            TestiDynTree,
            f"test_{name}",
            TestiDynTree.get_test_for_description(name),
        )
