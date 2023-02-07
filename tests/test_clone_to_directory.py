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

from robot_descriptions._cache import clone_to_directory
from robot_descriptions._repositories import REPOSITORIES


class TestCloneToDirectory(unittest.TestCase):
    def test_clone_to_directory(self):
        """Check cloning on a dummy repository."""
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

    def test_clone_to_directory_newer_commit(self):
        """Repository is pulled when a newer commit is configured."""
        description_name = "simple_humanoid_description"
        repo_params = REPOSITORIES[description_name]

        def get_commit(repo):
            return repo.git.rev_parse("HEAD")

        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_dir = os.path.join(tmp_dir, "test")
            repo = clone_to_directory(
                repo_params.url, repo_dir, commit=repo_params.commit
            )
            self.assertFalse(repo.bare)
            self.assertEqual(get_commit(repo), repo_params.commit)
            repo.git.reset("HEAD~1", "--hard")
            self.assertNotEqual(get_commit(repo), repo_params.commit)
            repo_bis = clone_to_directory(
                repo_params.url, repo_dir, commit=repo_params.commit
            )
            self.assertEqual(get_commit(repo), repo_params.commit)
            self.assertEqual(get_commit(repo_bis), repo_params.commit)

    def test_clone_to_invalid_directory(self):
        """Cloning to an empty (invalid) git repo recreates it."""
        description_name = "simple_humanoid_description"
        repo_params = REPOSITORIES[description_name]
        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_dir = os.path.join(tmp_dir, "test")
            os.mkdir(repo_dir)
            self.assertEqual(len(os.listdir(repo_dir)), 0)
            clone_to_directory(repo_params.url, repo_dir)
            self.assertGreater(len(os.listdir(repo_dir)), 0)
