#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import unittest

import pybullet

from robot_descriptions.loaders.idyntree import (
    load_robot_description as load_idyntree,
)
from robot_descriptions.loaders.mujoco import (
    load_robot_description as load_mujoco,
)
from robot_descriptions.loaders.pinocchio import (
    load_robot_description as load_pinocchio,
)
from robot_descriptions.loaders.pybullet import (
    load_robot_description as load_pybullet,
)
from robot_descriptions.loaders.robomeshcat import (
    load_robot_description as load_robomeshcat,
)
from robot_descriptions.loaders.yourdfpy import (
    load_robot_description as load_yourdfpy,
)


class TestLoaders(unittest.TestCase):
    """
    Test loaders.
    """

    upkie_description_commit = "98502d5b175c3d6b60b3cf475b7eeef9fd290c43"

    def test_mujoco(self):
        self.assertIsNotNone(load_mujoco("cassie_mj_description"))

    def test_pinocchio(self):
        self.assertIsNotNone(
            load_pinocchio(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )

    def test_pybullet(self):
        pybullet.connect(pybullet.DIRECT)
        self.assertIsNotNone(
            load_pybullet(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )
        pybullet.disconnect()

    def test_robomeshcat(self):
        self.assertIsNotNone(
            load_robomeshcat(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )

    def test_yourdfpy(self):
        self.assertIsNotNone(
            load_yourdfpy(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )

    def test_idyntree(self):
        self.assertIsNotNone(
            load_idyntree(
                "upkie_description",
                commit=self.upkie_description_commit,
            )
        )

    def test_cache_path_package_name(self):
        """Check a description with package:// URIs and a custom commit ID."""
        load_pinocchio(
            "draco3_description",
            commit="5afd19733d7b3e9f1135ba93e0aad90ed1a24cc7",
        )

    def test_load_with_commit_then_without(self):
        # https://github.com/robot-descriptions/robot_descriptions.py/issues/67
        load_pinocchio(
            "draco3_description",
            commit="5afd19733d7b3e9f1135ba93e0aad90ed1a24cc7",
        )
        load_pinocchio("baxter_description")
