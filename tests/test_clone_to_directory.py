#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import os
import tempfile
import unittest
from unittest.mock import ANY, MagicMock, patch

import git

from robot_descriptions._cache import clone_to_directory
from robot_descriptions._repositories import REPOSITORIES


class TestCloneToDirectory(unittest.TestCase):
    def test_fresh_clone_with_tag_uses_tag_refspec(self):
        """Fresh clones of a tag use a tag refspec and checkout by tag name."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_url = "https://example.com/repo.git"
            clone_dir = os.path.join(tmp_dir, "clone")
            target_tag = "v1.2.3"
            mock_repo = MagicMock()
            mock_origin = MagicMock()
            mock_repo.create_remote.return_value = mock_origin

            with patch("robot_descriptions._cache.Repo") as MockRepo:
                MockRepo.init.return_value = mock_repo
                result = clone_to_directory(repo_url, clone_dir, commit=target_tag)

            mock_repo.create_remote.assert_called_once_with("origin", repo_url)
            mock_origin.fetch.assert_called_once_with(
                f"refs/tags/{target_tag}:refs/tags/{target_tag}",
                depth=1,
                progress=ANY,
            )
            mock_repo.git.checkout.assert_called_once_with(target_tag)
            self.assertIs(result, mock_repo)

    def test_non_tag_revision_falls_back_to_direct_fetch(self):
        """If tag refspec is missing remotely, fallback fetches the revision directly."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_url = "https://example.com/repo.git"
            clone_dir = os.path.join(tmp_dir, "clone")
            revision = "release-1.0.0"
            mock_repo = MagicMock()
            mock_origin = MagicMock()
            mock_repo.create_remote.return_value = mock_origin
            mock_origin.fetch.side_effect = [
                git.exc.GitCommandError(
                    "git fetch",
                    128,
                    stderr=(
                        "fatal: couldn't find remote ref "
                        f"refs/tags/{revision}"
                    ),
                ),
                None,
            ]

            with patch("robot_descriptions._cache.Repo") as MockRepo:
                MockRepo.init.return_value = mock_repo
                result = clone_to_directory(repo_url, clone_dir, commit=revision)

            self.assertEqual(mock_origin.fetch.call_count, 2)
            mock_origin.fetch.assert_any_call(
                f"refs/tags/{revision}:refs/tags/{revision}",
                depth=1,
                progress=ANY,
            )
            mock_origin.fetch.assert_any_call(revision, depth=1, progress=ANY)
            mock_repo.git.checkout.assert_called_once_with(revision)
            self.assertIs(result, mock_repo)

    def test_fresh_clone_with_commit_uses_shallow_fetch(self):
        """Fresh clones of a specific commit use shallow fetch."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_url = "https://example.com/repo.git"
            clone_dir = os.path.join(tmp_dir, "clone")
            target_commit = "deadbeef" * 5  # 40-char hex SHA
            mock_repo = MagicMock()
            mock_origin = MagicMock()
            mock_repo.create_remote.return_value = mock_origin

            with patch("robot_descriptions._cache.Repo") as MockRepo:
                MockRepo.init.return_value = mock_repo
                result = clone_to_directory(repo_url, clone_dir, commit=target_commit)

            mock_repo.create_remote.assert_called_once_with("origin", repo_url)
            mock_origin.fetch.assert_called_once_with(
                target_commit, depth=1, progress=ANY  # SHA passed directly
            )
            mock_repo.git.checkout.assert_called_once_with(target_commit)
            self.assertIs(result, mock_repo)

    def test_existing_repo_fetches_missing_commit(self):
        """If checkout fails, the commit is fetched shallowly before retrying."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_url = "https://example.com/repo.git"
            clone_dir = os.path.join(tmp_dir, "clone")
            os.makedirs(clone_dir)
            target_commit = "deadbeef" * 5  # 40-char hex SHA
            mock_repo = MagicMock()
            mock_origin = MagicMock()
            mock_repo.remote.return_value = mock_origin
            mock_repo.git.checkout.side_effect = [
                git.exc.GitCommandError("git checkout", 128),
                None,
            ]

            with patch("robot_descriptions._cache.Repo", return_value=mock_repo):
                result = clone_to_directory(repo_url, clone_dir, commit=target_commit)

            mock_repo.remote.assert_called_with("origin")
            mock_origin.fetch.assert_called_once_with(target_commit, depth=1)  # SHA passed directly
            self.assertEqual(mock_repo.git.checkout.call_count, 2)
            self.assertIs(result, mock_repo)

    def test_existing_repo_at_requested_revision_skips_checkout(self):
        """When HEAD already matches the revision, checkout and fetch are skipped."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            repo_url = "https://example.com/repo.git"
            clone_dir = os.path.join(tmp_dir, "clone")
            os.makedirs(clone_dir)
            target_commit = "deadbeef" * 5
            mock_repo = MagicMock()
            mock_repo.git.rev_parse.side_effect = [target_commit, target_commit]

            with patch("robot_descriptions._cache.Repo", return_value=mock_repo):
                result = clone_to_directory(repo_url, clone_dir, commit=target_commit)

            mock_repo.git.checkout.assert_not_called()
            mock_repo.remote.assert_not_called()
            self.assertIs(result, mock_repo)

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

    def test_clone_to_directory_with_different_commit(self):
        """Cloning with pinned commits can move an existing clone to a new commit."""

        def get_commit(repo):
            return repo.git.rev_parse("HEAD")

        with tempfile.TemporaryDirectory() as tmp_dir:
            source_repo_dir = os.path.join(tmp_dir, "source_repo")
            clone_dir = os.path.join(tmp_dir, "clone")
            source_repo = git.Repo.init(source_repo_dir)

            tracked_file = os.path.join(source_repo_dir, "model.txt")
            with open(tracked_file, "w", encoding="utf-8") as f:
                f.write("v1\n")
            source_repo.index.add([tracked_file])
            source_repo.index.commit("Commit c1")
            c1 = source_repo.head.commit.hexsha

            with open(tracked_file, "w", encoding="utf-8") as f:
                f.write("v2\n")
            source_repo.index.add([tracked_file])
            source_repo.index.commit("Commit c2")
            c2 = source_repo.head.commit.hexsha

            repo = clone_to_directory(source_repo_dir, clone_dir, commit=c1)
            self.assertFalse(repo.bare)
            self.assertEqual(get_commit(repo), c1)

            repo_bis = clone_to_directory(source_repo_dir, clone_dir, commit=c2)
            self.assertEqual(get_commit(repo_bis), c2)

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
