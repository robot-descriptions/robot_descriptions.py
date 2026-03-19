#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import types
import unittest
from unittest.mock import patch

from robot_descriptions._descriptions import DESCRIPTIONS
from robot_descriptions.loaders.pinocchio import load_robot_description


class TestPinocchio(unittest.TestCase):

    """
    Check that all descriptions are loaded properly in Pinocchio.
    """

    @patch("robot_descriptions.loaders.pinocchio.pin.RobotWrapper.BuildFromURDF")
    @patch("robot_descriptions.loaders.pinocchio.get_package_dirs")
    @patch("robot_descriptions.loaders.pinocchio.get_urdf_path")
    @patch("robot_descriptions.loaders.pinocchio.import_module")
    def test_forwards_xacro_args_for_urdf(
        self,
        import_module_mock,
        get_urdf_path_mock,
        get_package_dirs_mock,
        _build_from_urdf_mock,
    ):
        module = types.SimpleNamespace(
            URDF_PATH="/tmp/test.urdf",
            PACKAGE_PATH="/tmp/pkg",
        )
        import_module_mock.return_value = module
        get_urdf_path_mock.return_value = module.URDF_PATH
        get_package_dirs_mock.return_value = [module.PACKAGE_PATH]

        load_robot_description("test_description", xacro_args={"hand": "false"})

        get_urdf_path_mock.assert_called_once_with(
            module, xacro_args={"hand": "false"}
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
