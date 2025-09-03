#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import unittest

from robot_descriptions._descriptions import DESCRIPTIONS
from robot_descriptions.loaders.mujoco import (
    load_robot_description as load_mujoco,
)


class TestMuJoCo(unittest.TestCase):
    """
    Check that all MJCF descriptions are loaded properly in MuJoCo.
    """

    def test_mujoco(self):
        self.assertIsNotNone(load_mujoco("cassie_mj_description"))

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
            load_mujoco(description)

        return test


for name, description in DESCRIPTIONS.items():
    if description.has_mjcf:
        setattr(
            TestMuJoCo,
            f"test_{name}",
            TestMuJoCo.get_test_for_description(name),
        )
