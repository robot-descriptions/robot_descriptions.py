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

from robot_descriptions.git import clone_to_directory


class TestGit(unittest.TestCase):

    """
    Test fixture for git-related functions.
    """

    def setUp(self):
        """
        Initialize a dummy git repository.
        """
        tmp_dir = tempfile.TemporaryDirectory()
        repo_name = "test_repo"
        repo_path = os.path.join(tmp_dir.name, f"{repo_name}.git")
        descriptions_dir = os.path.join(tmp_dir.name, "descriptions")
        empty_file = os.path.join(repo_path, "foo.bar")
        repo = git.Repo.init(repo_path)
        open(empty_file, "wb").close()
        repo.index.add([empty_file])
        repo.index.commit("Test initial commit")

        self.descriptions_dir = descriptions_dir
        self.repo_name = repo_name
        self.repo_path = repo_path
        self.tmp_dir = tmp_dir

    def tearDown(self):
        """
        Clean up temporary directory.
        """
        self.tmp_dir.cleanup()

    def test_git_clone_description(self):
        """
        Check cloning on a dummy repository.
        """
        # Clone the repository for the first time
        clone_1 = git_clone_description(
            self.repo_path,
            descriptions_dir=self.descriptions_dir,
        )
        self.assertTrue(str(clone_1.active_branch) in ["main", "master"])
        self.assertTrue(clone_1.common_dir.startswith(self.tmp_dir.name))
        self.assertTrue(clone_1.working_dir.endswith(self.repo_name))

        # Cloning again should recover the existing repo
        clone_2 = git_clone_description(
            self.repo_path,
            descriptions_dir=self.descriptions_dir,
        )
        self.assertEqual(
            str(clone_1.active_branch), str(clone_2.active_branch)
        )
        self.assertTrue(clone_1.common_dir, clone_2.common_dir)
        self.assertTrue(clone_1.working_dir, clone_2.working_dir)


if __name__ == "__main__":
    unittest.main()
