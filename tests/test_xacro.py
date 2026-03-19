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
    _convert_filenames_to_package_uris,
    _to_package_uri,
    get_urdf_path,
)


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
            rootdir = None

            @staticmethod
            def to_urdf_string(use_protocols: bool, pretty: bool) -> str:
                self.assertTrue(use_protocols)
                self.assertTrue(pretty)
                return "<robot name='generated'/>"

        class FakeXacroDoc:
            @staticmethod
            def from_file(path: str, subargs, resolve_packages=False):
                calls["from_file"] += 1
                self.assertEqual(path, xacro_path)
                self.assertEqual(subargs, {"prefix": "test_"})
                self.assertFalse(resolve_packages)
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

    def test_to_package_uri_default_root(self):
        repo_dir = "/tmp/franka_description"
        package_dir = os.path.join(repo_dir, "robots", "fr3")

        default_cases = [
            (
                f"file://{os.path.join(repo_dir, 'meshes', 'visual', 'link.stl')}",
                "package://franka_description/meshes/visual/link.stl",
            ),
            (
                f"file://{os.path.join(package_dir, 'mesh.stl')}",
                "package://franka_description/mesh.stl",
            ),
            (
                "package://already_ok/meshes/keep.stl",
                "package://already_ok/meshes/keep.stl",
            ),
            ("file:///opt/external/mesh.stl", "file:///opt/external/mesh.stl"),
        ]

        for filename, expected in default_cases:
            with self.subTest(filename=filename):
                self.assertEqual(
                    _to_package_uri(
                        filename,
                        package_name="franka_description",
                        repo_path=repo_dir,
                        package_path=package_dir,
                        package_uri_root=package_dir,
                    ),
                    expected,
                )

        self.assertEqual(
            _to_package_uri(
                f"file://{os.path.join(package_dir, 'mesh.stl')}",
                package_name="franka_description",
                repo_path=repo_dir,
                package_path=package_dir,
                package_uri_root=package_dir,
            ),
            "package://franka_description/mesh.stl",
        )

    def test_package_uri_root_preserves_nested_package_path(self):
        repo_dir = "/tmp/stretch_urdf"
        package_dir = os.path.join(repo_dir, "stretch_urdf", "SE3")
        self.assertEqual(
            _to_package_uri(
                f"file://{os.path.join(package_dir, 'meshes', 'base_link.STL')}",
                package_name="stretch_urdf",
                repo_path=repo_dir,
                package_path=package_dir,
                package_uri_root=repo_dir,
            ),
            "package://stretch_urdf/stretch_urdf/SE3/meshes/base_link.STL",
        )

    def test_generated_urdf_filenames_are_normalized(self):
        repo_dir = "/tmp/stretch_urdf"
        package_dir = os.path.join(repo_dir, "stretch_urdf", "SE3")
        module = types.SimpleNamespace(
            __name__="robot_descriptions.stretch_se3_description",
            REPOSITORY_PATH=repo_dir,
            PACKAGE_PATH=package_dir,
            PACKAGE_URI_ROOT=repo_dir,
        )
        urdf_text = _convert_filenames_to_package_uris(
            module,
            "\n".join(
                [
                    "<robot name='generated'>",
                    "<mesh filename='"
                    f"file://{os.path.join(repo_dir, 'meshes', 'visual', 'link.stl')}"
                    "'/>",
                    "<mesh filename='"
                    f"file://{os.path.join(package_dir, 'meshes', 'base_link.STL')}"
                    "'/>",
                    "<mesh filename='package://already_ok/meshes/keep.stl'/>",
                    "<mesh filename='file:///opt/external/mesh.stl'/>",
                    "</robot>",
                ]
            ),
        )

        self.assertIn(
            "filename='package://stretch_urdf/meshes/visual/link.stl'",
            urdf_text,
        )
        self.assertIn(
            "filename='package://stretch_urdf/stretch_urdf/SE3/meshes/base_link.STL'",
            urdf_text,
        )
        self.assertIn(
            "package://already_ok/meshes/keep.stl",
            urdf_text,
        )
        self.assertIn("file:///opt/external/mesh.stl", urdf_text)
