#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import os
import shutil
import tempfile
import types
import unittest
from unittest.mock import patch

from robot_descriptions._descriptions import DESCRIPTION_FORMATS
from robot_descriptions._xacro import (
    _DESCRIPTION_FORMAT_ATTRS,
    _convert_filenames_to_package_uris,
    _to_package_uri,
    get_description_path,
    get_mjcf_path,
    get_srdf_path,
    get_urdf_path,
)


class _FakeDoc:
    rootdir = None

    @staticmethod
    def to_urdf_string(use_protocols: bool, pretty: bool) -> str:
        return "<robot name='generated'/>"


class TestXacro(unittest.TestCase):
    """Unit tests for xacro path resolution."""

    def test_description_format_attrs_match_description_formats(self):
        self.assertEqual(
            set(_DESCRIPTION_FORMAT_ATTRS.keys()),
            set(DESCRIPTION_FORMATS),
        )

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
            def from_file(path: str, subargs, resolve_packages=False):
                calls["from_file"] += 1
                self.assertEqual(path, xacro_path)
                self.assertEqual(subargs, {"prefix": "test_"})
                self.assertFalse(resolve_packages)
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
            def from_file(path: str, subargs, resolve_packages=False):
                self.assertEqual(path, xacro_path)
                self.assertFalse(resolve_packages)
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

    def test_to_package_uri(self):
        package_dir = "/tmp/ros2_kortex/kortex_description"
        mesh = os.path.join(package_dir, "meshes", "base_link.STL")
        outside = os.path.join("/tmp/ros2_kortex", "other", "thing.stl")

        cases = [
            (
                f"file://{mesh}",
                "package://kortex_description/meshes/base_link.STL",
            ),
            # Files outside the package are left untouched.
            (f"file://{outside}", f"file://{outside}"),
            (
                "package://already_ok/meshes/keep.stl",
                "package://already_ok/meshes/keep.stl",
            ),
            ("file:///opt/external/mesh.stl", "file:///opt/external/mesh.stl"),
        ]

        for filename, expected in cases:
            with self.subTest(filename=filename):
                self.assertEqual(
                    _to_package_uri(
                        filename,
                        package_name="kortex_description",
                        package_path=package_dir,
                    ),
                    expected,
                )

    def test_to_package_uri_names_package_not_repository(self):
        # The URI names the package directory (e.g. "SE3"), not the
        # repository clone that happens to contain it.
        package_dir = "/tmp/stretch_urdf/stretch_urdf/SE3"
        base_link = os.path.join(package_dir, "meshes", "base_link.STL")
        self.assertEqual(
            _to_package_uri(
                f"file://{base_link}",
                package_name="SE3",
                package_path=package_dir,
            ),
            "package://SE3/meshes/base_link.STL",
        )

    def test_generated_urdf_filenames_are_normalized(self):
        repo_dir = "/tmp/stretch_urdf"
        package_dir = os.path.join(repo_dir, "stretch_urdf", "SE3")
        base_link = os.path.join(package_dir, "meshes", "base_link.STL")
        module = types.SimpleNamespace(
            __name__="robot_descriptions.stretch_se3_description",
            REPOSITORY_PATH=repo_dir,
            PACKAGE_PATH=package_dir,
        )
        urdf_text = _convert_filenames_to_package_uris(
            module,
            "\n".join(
                [
                    "<robot name='generated'>",
                    f"<mesh filename='file://{base_link}'/>",
                    "<mesh filename='package://already_ok/meshes/keep.stl'/>",
                    "<mesh filename='file:///opt/external/mesh.stl'/>",
                    "</robot>",
                ]
            ),
        )

        self.assertIn(
            "filename='package://SE3/meshes/base_link.STL'",
            urdf_text,
        )
        self.assertIn(
            "package://already_ok/meshes/keep.stl",
            urdf_text,
        )
        self.assertIn("file:///opt/external/mesh.stl", urdf_text)
