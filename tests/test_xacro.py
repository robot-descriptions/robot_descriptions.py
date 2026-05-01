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

from robot_descriptions._xacro import (
    get_description_path,
    get_mjcf_path,
    get_srdf_path,
    get_urdf_path,
)


class _FakeDoc:
    class _Dom:
        @staticmethod
        def getElementsByTagName(_name: str):
            return []

    dom = _Dom()

    def to_urdf_file(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as out_file:
            out_file.write("<robot name='generated'/>")


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

    def test_srdf_passthrough(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            SRDF_PATH="/tmp/test.srdf",
        )
        self.assertEqual(get_srdf_path(module), "/tmp/test.srdf")

    def test_mjcf_passthrough(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            MJCF_PATH="/tmp/test.xml",
        )
        self.assertEqual(get_mjcf_path(module), "/tmp/test.xml")

    def test_description_path_uses_requested_format(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            URDF_PATH="/tmp/test.urdf",
            MJCF_PATH="/tmp/test.xml",
            SRDF_PATH="/tmp/test.srdf",
        )
        self.assertEqual(
            get_description_path(module, "mjcf"),
            "/tmp/test.xml",
        )

    def test_description_path_rejects_unknown_format(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
        )
        with self.assertRaises(ValueError):
            get_description_path(module, "unknown")

    def test_rejects_xacro_args_for_plain_urdf(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            URDF_PATH="/tmp/test.urdf",
        )
        with self.assertRaises(ValueError):
            get_urdf_path(module, xacro_args={})

    def test_rejects_xacro_args_for_plain_srdf(self):
        module = types.SimpleNamespace(
            __name__="robot_descriptions.test_description",
            SRDF_PATH="/tmp/test.srdf",
        )
        with self.assertRaises(ValueError):
            get_srdf_path(module, xacro_args={})

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

        class FakeXacroDoc:
            @staticmethod
            def from_file(path: str, subargs):
                calls["from_file"] += 1
                self.assertEqual(path, xacro_path)
                self.assertEqual(subargs, {"prefix": "test_"})
                return _FakeDoc()

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

        class FakeXacroDoc:
            @staticmethod
            def from_file(path: str, subargs):
                self.assertEqual(path, xacro_path)
                calls["subargs"].append(subargs)
                return _FakeDoc()

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