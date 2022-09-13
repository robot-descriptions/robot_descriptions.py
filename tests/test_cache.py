#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import tempfile
import unittest

import git

from robot_descriptions._cache import (
    CloneProgressBar,
    clone_to_cache,
    clone_to_directory,
)
from robot_descriptions._repositories import REPOSITORIES


class TestCache(unittest.TestCase):

    """
    Test fixture for git-related functions.
    """

    def test_clone_to_directory(self):
        """
        Check cloning on a dummy repository.
        """
        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_name = "test_repo"
            repo_path = os.path.join(tmp_dir, repo_name)
            clone_dir = os.path.join(tmp_dir, "clone")
            empty_file = os.path.join(repo_path, "foo.bar")
            repo = git.Repo.init(repo_path)
            open(empty_file, "wb").close()
            repo.index.add([empty_file])
            repo.index.commit("Test initial commit")

            # Clone the repository for the first time
            clone_1 = clone_to_directory(repo_path, clone_dir)
            self.assertTrue(clone_1.common_dir.startswith(tmp_dir))
            self.assertTrue(clone_1.working_dir.endswith("clone"))

            # Cloning again, should recover the existing repo
            clone_2 = clone_to_directory(repo_path, clone_dir)
            self.assertEqual(
                str(clone_1.active_branch), str(clone_2.active_branch)
            )
            self.assertTrue(clone_1.common_dir, clone_2.common_dir)
            self.assertTrue(clone_1.working_dir, clone_2.working_dir)

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

    def test_cache_creation(self):
        """
        Check that clone_to_cache creates directory if needed.
        """
        description = "upkie_description"
        repo = REPOSITORIES[description]
        with tempfile.TemporaryDirectory() as tmp_dir:
            os.environ["ROBOT_DESCRIPTIONS_CACHE"] = tmp_dir
            clone_to_cache(description)
            self.assertTrue(
                os.path.exists(os.path.join(tmp_dir, repo.cache_path))
            )
            del os.environ["ROBOT_DESCRIPTIONS_CACHE"]

    def test_progress_bar(self):
        """
        Check progress bar.
        """
        bar = CloneProgressBar()
        bar.update(0, 42, 42)
        self.assertEqual(bar.progress.n, 42)
        self.assertEqual(bar.progress.total, 42)
