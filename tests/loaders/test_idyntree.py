#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Giulio Romualdi

import types
import unittest
from unittest.mock import patch

from robot_descriptions._descriptions import DESCRIPTIONS

from robot_descriptions.loaders.idyntree import load_robot_description

class TestiDynTree(unittest.TestCase):

    """
    Check that all descriptions are loaded properly in iDynTree.
    """

    @patch("robot_descriptions.loaders.idyntree.idyn.ModelLoader")
    @patch("robot_descriptions.loaders.idyntree.get_urdf_path")
    @patch("robot_descriptions.loaders.idyntree.import_module")
    def test_forwards_xacro_args(
        self,
        import_module_mock,
        get_urdf_path_mock,
        model_loader_cls_mock,
    ):
        module = types.SimpleNamespace(
            URDF_PATH="/tmp/test.urdf",
            PACKAGE_PATH="/tmp/pkg",
            REPOSITORY_PATH="/tmp/repo",
        )
        import_module_mock.return_value = module
        get_urdf_path_mock.return_value = module.URDF_PATH
        model_loader = model_loader_cls_mock.return_value
        model_loader.loadModelFromFile.return_value = True
        model_loader.model.return_value.copy.return_value = object()

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
            TestiDynTree,
            f"test_{name}",
            TestiDynTree.get_test_for_description(name),
        )
