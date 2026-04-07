#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Inria

import os
import shutil
import tempfile
import types
import unittest
from unittest.mock import patch

from robot_descriptions._xacro import get_urdf_path


class TestXacro(unittest.TestCase):
    """Unit tests for xacro path resolution."""

    def setUp(self):
        self.cache_dir = tempfile.mkdtemp(prefix="robot_descriptions_cache_")
        os.environ["ROBOT_DESCRIPTIONS_CACHE"] = self.cache_dir

    def tearDown(self):
        os.environ.pop("ROBOT_DESCRIPTIONS_CACHE", None)
        shutil.rmtree(self.cache_dir, ignore_errors=True)

    def test_urdf_passthrough(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            URDF_PATH="/tmp/test.urdf",
        )
        self.assertEqual(get_urdf_path(module), "/tmp/test.urdf")

    def test_rejects_xacro_args_for_plain_urdf(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            URDF_PATH="/tmp/test.urdf",
        )
        with self.assertRaises(ValueError):
            get_urdf_path(module, xacro_args={})

    def test_requires_xacrodoc(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            XACRO_PATH="/tmp/test.xacro",
            XACRO_ARGS={},
        )
        with patch(
            "robot_descriptions._xacro.import_module",
            side_effect=ModuleNotFoundError,
        ):
            with self.assertRaises(ModuleNotFoundError):
                get_urdf_path(module)

    def test_xacro_generated_once_then_reused(self):
        input_dir = tempfile.mkdtemp(prefix="robot_descriptions_xacro_")
        self.addCleanup(lambda: shutil.rmtree(input_dir, ignore_errors=True))
        xacro_path = os.path.join(input_dir, "robot.urdf.xacro")
        with open(xacro_path, "w", encoding="utf-8") as xacro_file:
            xacro_file.write("<robot name='test'/>")

        calls = {"from_file": 0}

        class FakeDoc:
            class _Dom:
                @staticmethod
                def getElementsByTagName(_name: str):
                    return []

            dom = _Dom()

            def to_urdf_file(self, path: str) -> None:
                with open(path, "w", encoding="utf-8") as urdf_file:
                    urdf_file.write("<robot name='generated'/>")

        class FakeXacroDoc:
            @staticmethod
            def from_file(path: str, subargs):
                calls["from_file"] += 1
                self.assertEqual(path, xacro_path)
                self.assertEqual(subargs, {"prefix": "test_"})
                return FakeDoc()

        fake_xacrodoc = types.SimpleNamespace(
            __version__="0.0.0-test",
            XacroDoc=FakeXacroDoc,
        )

        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            XACRO_PATH=xacro_path,
            XACRO_ARGS={"prefix": "test_"},
            REPOSITORY_PATH="/tmp/repo",
            PACKAGE_PATH="/tmp/pkg",
        )

        with patch(
            "robot_descriptions._xacro.import_module",
            return_value=fake_xacrodoc,
        ):
            first_path = get_urdf_path(module)
            second_path = get_urdf_path(module)

        self.assertEqual(first_path, second_path)
        self.assertTrue(os.path.exists(first_path))
        self.assertEqual(calls["from_file"], 1)

    def test_xacro_args_override_module_args(self):
        input_dir = tempfile.mkdtemp(prefix="robot_descriptions_xacro_")
        self.addCleanup(lambda: shutil.rmtree(input_dir, ignore_errors=True))
        xacro_path = os.path.join(input_dir, "robot.urdf.xacro")
        with open(xacro_path, "w", encoding="utf-8") as xacro_file:
            xacro_file.write("<robot name='test'/>")

        calls = {"subargs": []}

        class FakeDoc:
            class _Dom:
                @staticmethod
                def getElementsByTagName(_name: str):
                    return []

            dom = _Dom()

            def to_urdf_file(self, path: str) -> None:
                with open(path, "w", encoding="utf-8") as urdf_file:
                    urdf_file.write("<robot name='generated'/>")

        class FakeXacroDoc:
            @staticmethod
            def from_file(path: str, subargs):
                self.assertEqual(path, xacro_path)
                calls["subargs"].append(subargs)
                return FakeDoc()

        fake_xacrodoc = types.SimpleNamespace(
            __version__="0.0.0-test",
            XacroDoc=FakeXacroDoc,
        )

        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            XACRO_PATH=xacro_path,
            XACRO_ARGS={"prefix": "test_", "hand": "true"},
            REPOSITORY_PATH="/tmp/repo",
            PACKAGE_PATH="/tmp/pkg",
        )

        with patch(
            "robot_descriptions._xacro.import_module",
            return_value=fake_xacrodoc,
        ):
            first_path = get_urdf_path(
                module,
                xacro_args={"hand": "false"},
            )
            second_path = get_urdf_path(
                module,
                xacro_args={"hand": "false"},
            )
            third_path = get_urdf_path(
                module,
                xacro_args={"hand": "true"},
            )

        self.assertEqual(
            calls["subargs"],
            [
                {"prefix": "test_", "hand": "false"},
                {"prefix": "test_", "hand": "true"},
            ],
        )
        self.assertEqual(first_path, second_path)
        self.assertNotEqual(first_path, third_path)
