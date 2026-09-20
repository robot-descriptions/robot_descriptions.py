#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0

import unittest

from robot_descriptions._cache import CloneProgressBar


class TestProgressBar(unittest.TestCase):
    def test_progress_bar(self):
        """
        Check progress bar.
        """
        bar = CloneProgressBar()
        bar.update(0, 42, 42)
        self.assertEqual(bar.progress.n, 42)
        self.assertEqual(bar.progress.total, 42)
