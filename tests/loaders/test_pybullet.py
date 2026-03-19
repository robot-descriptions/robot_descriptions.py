#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import types
import unittest
from unittest.mock import patch

import pybullet

from robot_descriptions._descriptions import DESCRIPTIONS

from robot_descriptions.loaders.pybullet import load_robot_description


class TestPyBullet(unittest.TestCase):

    """
    Check that all descriptions are loaded properly in PyBullet.
    """

    def setUp(self):
        """
        Start PyBullet simulation.
        """
        pybullet.connect(pybullet.DIRECT)

    def tearDown(self):
        """
        Stop PyBullet simulation.
        """
        pybullet.disconnect()

    def test_value_error_when_no_urdf(self):
        """
        Test exception raised when a description has no URDF_PATH.
        """
        with self.assertRaises(ValueError):
            load_robot_description("_empty_description")

    @patch("robot_descriptions.loaders.pybullet.pybullet.loadURDF")
    @patch("robot_descriptions.loaders.pybullet.pybullet.setAdditionalSearchPath")
    @patch("robot_descriptions.loaders.pybullet.get_urdf_path")
    @patch("robot_descriptions.loaders.pybullet.import_module")
    def test_forwards_xacro_args(
        self,
        import_module_mock,
        get_urdf_path_mock,
        _set_search_path_mock,
        _load_urdf_mock,
    ):
        module = types.SimpleNamespace(
            URDF_PATH="/tmp/test.urdf",
            PACKAGE_PATH="/tmp/pkg",
        )
        import_module_mock.return_value = module
        get_urdf_path_mock.return_value = module.URDF_PATH

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


for name, description in DESCRIPTIONS.items():
    if description.has_urdf:
        setattr(
            TestPyBullet,
            f"test_{name}",
            TestPyBullet.get_test_for_description(name),
        )
