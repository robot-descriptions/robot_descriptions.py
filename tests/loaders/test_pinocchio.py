#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import unittest

from robot_descriptions._descriptions import DESCRIPTIONS
from robot_descriptions.loaders.pinocchio import (
    load_robot_description as load_pinocchio,
)


class TestPinocchio(unittest.TestCase):
    """
    Check that all descriptions are loaded properly in Pinocchio.
    """

    def test_pinocchio(self):
        self.assertIsNotNone(
            load_pinocchio(
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
            load_pinocchio(description)

        return test


# Add a test function for each description
for name, description in DESCRIPTIONS.items():
    if name == "a1_mj_description":
        # See https://github.com/stack-of-tasks/pinocchio/issues/2613
        continue
    if name == "aloha_mj_description":
        # See https://github.com/stack-of-tasks/pinocchio/issues/2610
        continue
    if name == "talos_mj_description":
        # See https://github.com/stack-of-tasks/pinocchio/issues/2612
        continue
    setattr(
        TestPinocchio,
        f"test_{name}",
        TestPinocchio.get_test_for_description(name),
    )
