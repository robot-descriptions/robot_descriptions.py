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


import os
from typing import Union

from git import RemoteProgress, Repo
from tqdm import tqdm


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


def git_clone_to_descriptions_dir(
    repo_url: str,
    descriptions_dir: str = "~/.cache/robot_descriptions",
    **kwargs,
) -> Repo:
    """
    Clone a repository to the descriptions directory.

    Args:
        repo_url: URL to the git repository to clone.
        descriptions_dir: Path to a directory where robot descriptions will be
            downloaded (default: ``~/.cache/robot_descriptions``). If the
            directory does not exist it will be created.
        kwargs: Remaining keyword arguments are forwarded to the git-clone
            command.

    Returns:
        Cloned git repository.
    """
    descriptions_dir = os.path.expanduser(descriptions_dir)
    if not os.path.exists(descriptions_dir):
        os.makedirs(descriptions_dir)

    repo_name = os.path.basename(repo_url)
    if repo_name.endswith(".git"):
        repo_name = repo_name[:-4]
    repo_path = f"{descriptions_dir}/{repo_name}"

    if os.path.exists(repo_path):
        return Repo(repo_path, **kwargs)
    print(f"Downloading {repo_name}...")
    progress_bar = CloneProgressBar()
    return Repo.clone_from(
        repo_url, repo_path, progress=progress_bar.update, **kwargs
    )
