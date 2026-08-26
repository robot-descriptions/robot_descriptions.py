#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import os
import threading
import unittest

from robot_descriptions._cache import _resolve_commit, description_commit


class TestDescriptionCommit(unittest.TestCase):
    def test_scope_applies_and_restores(self):
        """
        Check that a scoped commit only applies inside its block.
        """
        self.assertIsNone(_resolve_commit(None))
        with description_commit("abc123"):
            self.assertEqual(_resolve_commit(None), "abc123")
        self.assertIsNone(_resolve_commit(None))

    def test_scope_restores_when_the_block_raises(self):
        """
        Check that a failed import does not leak its commit.

        The commit used to be written to the environment and removed
        after the import, so an import which raised in between left it
        set and every later description was checked out at it.
        """
        with self.assertRaises(RuntimeError):
            with description_commit("abc123"):
                raise RuntimeError("import failed")
        self.assertIsNone(_resolve_commit(None))

    def test_scope_is_isolated_between_threads(self):
        """
        Check that two threads do not overwrite each other's commit.
        """
        commits = {}
        # The barrier holds both threads inside their own scope at the
        # same time, which is when they would overwrite each other
        barrier = threading.Barrier(2)

        def resolve(name: str, commit: str) -> None:
            with description_commit(commit):
                barrier.wait(timeout=5)
                commits[name] = _resolve_commit(None)

        first = threading.Thread(target=resolve, args=("first", "aaa"))
        second = threading.Thread(target=resolve, args=("second", "bbb"))
        first.start()
        second.start()
        first.join(timeout=5)
        second.join(timeout=5)

        self.assertEqual(commits, {"first": "aaa", "second": "bbb"})

    def test_scoped_commit_wins_over_the_argument(self):
        """
        Check that a loader argument beats the environment variable.

        A description module reads ROBOT_DESCRIPTION_COMMIT itself and
        passes it down as an argument, so the scoped commit has to win
        for an explicit argument to beat a process-wide variable.
        """
        with description_commit("from_loader"):
            self.assertEqual(
                _resolve_commit("from_environment"), "from_loader"
            )

    def test_none_leaves_the_surrounding_scope_alone(self):
        """
        Check that no commit at all does not clear an outer one.
        """
        with description_commit("outer"):
            with description_commit(None):
                self.assertEqual(_resolve_commit(None), "outer")
            self.assertEqual(_resolve_commit(None), "outer")

    def test_the_environment_is_left_alone(self):
        """
        Check that scoping a commit no longer touches the environment.
        """
        before = dict(os.environ)
        with description_commit("abc123"):
            self.assertEqual(dict(os.environ), before)
        self.assertEqual(dict(os.environ), before)


if __name__ == "__main__":
    unittest.main()
