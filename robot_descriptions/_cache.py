#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Git utility functions to clone model repositories."""

import os
import shutil
from typing import Optional, Union

import tqdm
from git import (
    GitCommandError,
    InvalidGitRepositoryError,
    RemoteProgress,
    Repo,
)

from ._repositories import REPOSITORIES


class CloneProgressBar(RemoteProgress):
    """Progress bar when cloning."""

    def __init__(self):
        """Initialize progress bar."""
        super().__init__()
        self.progress = tqdm.tqdm()

    def update(
        self,
        op_code: int,
        cur_count: Union[str, float],
        max_count: Union[str, float, None] = None,
        message: str = "",
    ) -> None:
        """Update progress bar.

        Args:
            op_code: Integer that can be compared against Operation IDs and
                stage IDs.
            cur_count: Current item count.
            max_count: Maximum item count, or `None` if there is none.
            message: Unused here.
        """
        self.progress.total = max_count
        self.progress.n = cur_count
        self.progress.refresh()


def _refspec_for_revision(revision: str) -> str:
    """Return a fetch refspec that materializes a local ref for string revision.

    For 40-char hex SHA commits, returns the SHA directly. For tag names,
    returns a refspec that also creates the local tag so checkout by name works.
    """
    if len(revision) == 40:
        try:
            int(revision, 16)
            return revision
        except ValueError:
            pass
    return f"refs/tags/{revision}:refs/tags/{revision}"


def _is_missing_remote_ref(error: GitCommandError) -> bool:
    """Return whether git fetch failed because a remote ref does not exist."""
    return "couldn't find remote ref" in str(error)


def _fetch_revision_shallow(
    remote,
    revision: str,
    progress: Optional[RemoteProgress] = None,
) -> None:
    """Fetch a revision with depth 1.

    For non-SHA revisions, first try a tag refspec so the tag name is
    materialized locally for checkout. If that tag does not exist, fall back
    to fetching the revision directly (e.g., branch names).
    """
    fetch_kwargs = {"depth": 1}
    if progress is not None:
        fetch_kwargs["progress"] = progress

    if _refspec_for_revision(revision) == revision:
        remote.fetch(revision, **fetch_kwargs)
        return

    tag_refspec = _refspec_for_revision(revision)
    try:
        remote.fetch(tag_refspec, **fetch_kwargs)
    except GitCommandError as error:
        if not _is_missing_remote_ref(error):
            raise
        remote.fetch(revision, **fetch_kwargs)


def _is_head_at_revision(repo: Repo, revision: str) -> bool:
    """Return whether HEAD already resolves to the requested revision."""
    try:
        requested = repo.git.rev_parse(revision).strip()
        head = repo.git.rev_parse("HEAD").strip()
    except GitCommandError:
        return False
    return requested == head


def clone_to_directory(
    repo_url: str,
    target_dir: str,
    commit: Optional[str] = None,
) -> Repo:
    """Clone a git repository to a designated directory.

    Args:
        repo_url: URL to the git repository to clone.
        target_dir: Directory to clone the repository to.
        commit: Optional commit to check out after cloning.

    Returns:
        Cloned git repository.
    """
    clone = None
    if os.path.exists(target_dir):
        try:
            clone = Repo(target_dir)
        except InvalidGitRepositoryError:
            print(f"Repository at {target_dir} is invalid, recreating it...")
            shutil.rmtree(target_dir)
            clone = None

    if clone is None:
        print(f"Cloning {repo_url}...")
        progress_bar = CloneProgressBar()
        if commit is not None:
            os.makedirs(target_dir, exist_ok=True)
            clone = Repo.init(target_dir)
            origin = clone.create_remote("origin", repo_url)
            _fetch_revision_shallow(origin, commit, progress=progress_bar)
        else:
            clone = Repo.clone_from(
                repo_url,
                target_dir,
                progress=progress_bar.update,
            )

    if commit is not None and not _is_head_at_revision(clone, commit):
        try:
            clone.git.checkout(commit)
        except GitCommandError:
            print(
                f"Commit {commit} not found, "
                "let's fetch origin and try again..."
            )
            _fetch_revision_shallow(clone.remote("origin"), commit)
            clone.git.checkout(commit)
            print(f"Found commit {commit} successfully!")

    return clone


def clone_to_cache(description_name: str, commit: Optional[str] = None) -> str:
    """Get a local working directory cloned from a remote git repository.

    Args:
        description_name: Name of the robot description to clone.
        commit: If provided, checkout that specific commit of the description.

    Returns:
        Path to the resulting local working directory.

    Notes:
        By default, robot descriptions are cached to
        `~/.cache/robot_descriptions`. This behavior can be overriden by
        setting the `ROBOT_DESCRIPTIONS_CACHE` environment variable to an
        alternative path.
    """
    try:
        repository = REPOSITORIES[description_name]
    except KeyError as exn:
        raise ImportError(f"Unknown description: {description_name}") from exn

    cache_dir = os.path.expanduser(
        os.environ.get(
            "ROBOT_DESCRIPTIONS_CACHE",
            "~/.cache/robot_descriptions",
        )
    )

    cache_path = repository.cache_path
    if commit is not None:
        # Requirement: the last directory in the cache path is named after the
        # cache path (more precisely the package name) so that package:// URIs
        # work in frameworks that resolve them via directories
        cache_path = f"{cache_path}-{commit}/{cache_path}"
    target_dir = os.path.join(cache_dir, cache_path)
    clone = clone_to_directory(
        repository.url,
        target_dir,
        commit=repository.commit if commit is None else commit,
    )
    return str(clone.working_dir)


def clear_cache() -> None:
    """Clears the local cache where robot descriptions are stored."""
    cache_dir = os.path.expanduser(
        os.environ.get(
            "ROBOT_DESCRIPTIONS_CACHE",
            "~/.cache/robot_descriptions",
        )
    )
    shutil.rmtree(cache_dir, ignore_errors=True)
