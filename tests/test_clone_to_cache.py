#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import os
import tempfile
import unittest

import git

from robot_descriptions._cache import clone_to_cache, clear_cache
from robot_descriptions._repositories import REPOSITORIES


class TestCloneToCache(unittest.TestCase):
    def test_clone_to_cache_found(self):
        """
        Test clone_to_cache on a valid repository.
        """
        working_dir_1 = clone_to_cache("upkie_description")
        working_dir_2 = clone_to_cache("upkie_description")
        self.assertEqual(working_dir_1, working_dir_2)

    def test_clone_to_cache_not_found(self):
        """
        Test clone_to_cache on an invalid repository.
        """
        with self.assertRaises(ImportError):
            clone_to_cache("foo")

    def test_clone_to_cache_right_commit(self):
        """
        Check that clone_to_cache clones the repository at the right commit.
        """
        description_name = "simple_humanoid_description"
        repo = git.Repo(clone_to_cache(description_name))
        commit = str(list(repo.iter_commits(max_count=1))[0])
        self.assertEqual(commit, REPOSITORIES[description_name].commit)

    def test_cache_creation(self):
        """
        Check that clone_to_cache creates directory if needed.
        """
        description_name = "simple_humanoid_description"
        repo = REPOSITORIES[description_name]

        with tempfile.TemporaryDirectory() as tmp_dir:
            os.environ["ROBOT_DESCRIPTIONS_CACHE"] = tmp_dir
            clone_to_cache(description_name)
            self.assertTrue(
                os.path.exists(os.path.join(tmp_dir, repo.cache_path))
            )
            del os.environ["ROBOT_DESCRIPTIONS_CACHE"]

    def test_clone_invalid_commit(self):
        """Invalid commit raises an exception."""
        with self.assertRaises(git.exc.GitCommandError):
            clone_to_cache("simple_humanoid_description", commit="foobar")

    def test_clone_valid_commit(self):
        description_name = "simple_humanoid_description"
        commit = "0e488ee4708155a71b2a92d05305a9186b543593"
        repository_path = clone_to_cache(description_name, commit)
        self.assertTrue(commit in repository_path)

    def test_clone_with_commit_then_without(self):
        clear_cache()
        clone_to_cache(
            "upkie_description",
            commit="a2c820054a7572603875def478f21376165c125e",
        )
        clone_to_cache("draco3_description")
