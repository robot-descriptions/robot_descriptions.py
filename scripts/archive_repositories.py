#!/usr/bin/env python3

"""Archive a clone of every description repository under archive/.

Each repository listed in REPOSITORIES is cloned in full, then checked out at
the pinned commit. Repositories already present in the archive are verified
rather than fetched again (what matters to maintain robot_descriptions is that
the pinned commit is available).
"""

from __future__ import annotations

import argparse
import shutil
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from git import GitCommandError, InvalidGitRepositoryError, Repo

from robot_descriptions._cache import _is_head_at_revision
from robot_descriptions._repositories import REPOSITORIES, Repository

ARCHIVE_DIR = Path(__file__).resolve().parents[1] / "archive"


class Status(Enum):
    """Outcome of archiving a single repository."""

    CLONED = "cloned"
    CHECKED_OUT = "checked out"
    UP_TO_DATE = "up to date"
    FAILED = "failed"


@dataclass(frozen=True)
class Result:
    """Outcome of archiving a repository, with an optional explanation."""

    name: str
    status: Status
    detail: str = ""


def _has_commit(repo: Repo, commit: str) -> bool:
    """Check whether a commit is present in a local repository.

    Args:
        repo: Handle to a working copy of the git repository.
        commit: Commit ID or tag pinned in REPOSITORIES.

    Returns:
        True if the revision resolves to a commit object locally.
    """
    try:
        repo.git.rev_parse("--verify", "--quiet", f"{commit}^{{commit}}")
    except GitCommandError:
        return False
    return True


def _check_existing_clone(
    name: str,
    repository: Repository,
    target_dir: Path,
) -> Result:
    """Check that an archived repository can serve its pinned commit.

    Args:
        name: Key of the repository in REPOSITORIES.
        repository: Remote repository pinned by robot_descriptions.
        target_dir: Existing directory in the archive.

    Returns:
        Outcome for this repository.
    """
    try:
        repo = Repo(target_dir)
    except InvalidGitRepositoryError:
        return Result(
            name,
            Status.FAILED,
            f"{target_dir} exists but is not a git repository",
        )
    if not _has_commit(repo, repository.commit):
        return Result(
            name,
            Status.FAILED,
            f"commit {repository.commit} is missing from {target_dir}",
        )
    if _is_head_at_revision(repo, repository.commit):
        return Result(name, Status.UP_TO_DATE)
    try:
        repo.git.checkout(repository.commit)
    except GitCommandError as error:
        return Result(
            name,
            Status.FAILED,
            f"cannot check out {repository.commit}: {error}",
        )
    return Result(name, Status.CHECKED_OUT)


def _clone_repository(
    name: str,
    repository: Repository,
    target_dir: Path,
) -> Result:
    """Clone a repository in full, then check out its pinned commit.

    Args:
        name: Key of the repository in REPOSITORIES.
        repository: Remote repository pinned by robot_descriptions.
        target_dir: Directory to clone the repository to.

    Returns:
        Outcome for this repository.
    """
    print(f"Cloning {name} from {repository.url}...")
    target_dir.parent.mkdir(parents=True, exist_ok=True)
    try:
        repo = Repo.clone_from(repository.url, target_dir)
    except GitCommandError as error:
        # Remove the partial clone so the next run starts from scratch.
        shutil.rmtree(target_dir, ignore_errors=True)
        return Result(name, Status.FAILED, f"cannot clone: {error}")
    try:
        repo.git.checkout(repository.commit)
    except GitCommandError as error:
        return Result(
            name,
            Status.FAILED,
            f"cannot check out {repository.commit}: {error}",
        )
    return Result(name, Status.CLONED)


def archive_repository(
    name: str,
    repository: Repository,
    archive_dir: Path,
) -> Result:
    """Make sure the archive holds the pinned commit of a repository.

    Args:
        name: Key of the repository in REPOSITORIES.
        repository: Remote repository pinned by robot_descriptions.
        archive_dir: Directory holding the archived repositories.

    Returns:
        Outcome for this repository.
    """
    # The leaf directory is named after the repository cache path, so
    # that the archive mirrors the layout of the description cache.
    target_dir = archive_dir / repository.cache_path
    if target_dir.exists():
        return _check_existing_clone(name, repository, target_dir)
    return _clone_repository(name, repository, target_dir)


def archive_repositories(archive_dir: Path, jobs: int) -> list[Result]:
    """Archive all repositories, at most ``jobs`` at a time.

    Args:
        archive_dir: Directory holding the archived repositories.
        jobs: Number of repositories to process in parallel.

    Returns:
        Outcome for each repository.
    """
    archive_dir.mkdir(parents=True, exist_ok=True)
    results = []
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        futures = [
            executor.submit(archive_repository, name, repository, archive_dir)
            for name, repository in REPOSITORIES.items()
        ]
        for future in as_completed(futures):
            result = future.result()
            suffix = f": {result.detail}" if result.detail else ""
            print(f"[{result.status.value}] {result.name}{suffix}")
            results.append(result)
    return results


def report(results: list[Result], archive_dir: Path) -> int:
    """Summarize outcomes to the standard output.

    Args:
        results: Outcome for each repository.
        archive_dir: Directory holding the archived repositories.

    Returns:
        Zero if every repository is archived at its pinned commit.
    """
    counts = Counter(result.status for result in results)
    print(f"\n{len(results)} repositories in {archive_dir}:")
    for status in Status:
        if counts[status]:
            print(f"- {counts[status]} {status.value}")

    failures = sorted(
        (result for result in results if result.status is Status.FAILED),
        key=lambda result: result.name,
    )
    if failures:
        print("\nFailed repositories:")
        for result in failures:
            print(f"- {result.name}: {result.detail}")
        return 1
    return 0


def main() -> int:
    """Command line entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--archive-dir",
        type=Path,
        default=ARCHIVE_DIR,
        help=f"directory to archive repositories to (default: {ARCHIVE_DIR})",
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=4,
        help="number of repositories to clone in parallel (default: 4)",
    )
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("there should be at least one job")

    results = archive_repositories(args.archive_dir, args.jobs)
    return report(results, args.archive_dir)


if __name__ == "__main__":
    raise SystemExit(main())
