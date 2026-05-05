#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import subprocess
import sys
import unittest
from pathlib import Path


class TestReadme(unittest.TestCase):
    def test_generated_description_tables_are_up_to_date(self):
        repo_root = Path(__file__).resolve().parents[1]
        script = repo_root / "scripts" / "generate_readme_descriptions.py"
        result = subprocess.run(
            [sys.executable, str(script), "--check"],
            cwd=repo_root,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr)


if __name__ == "__main__":
    unittest.main()
