#!/usr/bin/env python3
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

"""
Git utility functions to clone model repositories.
"""

import os
from typing import Union

from git import RemoteProgress, Repo
from tqdm import tqdm

from .repositories import REPOSITORIES


class CloneProgressBar(RemoteProgress):

    """
    Progress bar when cloning.
    """

    def __init__(self):
        """
        Initialize progress bar.
        """
        super().__init__()
        self.progress = tqdm()

    def update(
        self,
        op_code: int,
        cur_count: Union[str, float],
        max_count: Union[str, float, None] = None,
        message: str = "",
    ) -> None:
        """
        Update progress bar.

        Args:
            op_code: Integer that can be compared against Operation IDs and
                stage IDs.
            cur_count: Current item count.
            max_count: Maximum item count, or ``None`` if there is none.
            message: Unused here.
        """
        self.progress.total = max_count
        self.progress.n = cur_count
        self.progress.refresh()


def clone_to_directory(repo_url: str, target_dir: str) -> Repo:
    """
    Clone a git repository to a designated directory.

    Args:
        repo_url: URL to the git repository to clone.
        target_dir: Directory to clone the repository to.

    Returns:
        Cloned git repository.
    """
    repo_name = os.path.basename(repo_url)
    if repo_name.endswith(".git"):
        repo_name = repo_name[:-4]

    repo_path = os.path.join(target_dir, repo_name)
    if os.path.exists(repo_path):
        return Repo(repo_path)

    print(f"Downloading {repo_name}...")
    progress_bar = CloneProgressBar()
    return Repo.clone_from(
        repo_url,
        repo_path,
        progress=progress_bar.update,
    )


def clone_to_cache(
    description_name: str,
    cache_dir: str = "~/.cache/robot_descriptions",
) -> str:
    """
    Get a local working directory cloned from a remote git repository.

    Args:
        repo_url: URL to the remote git repository.
        cache_dir: Path to a directory where robot descriptions will be
            downloaded (default: ``~/.cache/robot_descriptions``). If the
            directory does not exist it will be created.
    Returns:
        Path to the resulting local working directory.
    """
    try:
        repository = REPOSITORIES[description_name]
    except KeyError:
        raise ImportError(f"Unknown description: {description_name}")

    cache_dir = os.path.expanduser(cache_dir)
    target_dir = os.path.join(cache_dir, repository.cache_path)
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    clone = clone_to_directory(repository.url, target_dir)
    clone.git.checkout(repository.commit)
    if clone.working_dir is None:
        raise ImportError("Git repository for the robot description is empty")

    return str(clone.working_dir)
