#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import os
import unittest
from importlib import import_module  # type: ignore

import git

from robot_descriptions._descriptions import DESCRIPTIONS


class TestDescriptions(unittest.TestCase):
    """Test fixture for all robot descriptions."""

    def test_all_descriptions(self):
        """Check all robot-description submodules."""
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

    def test_invalid_description_commit(self):
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = "foobar"
        with self.assertRaises(git.exc.GitCommandError):
            import_module("robot_descriptions.sigmaban_description")
