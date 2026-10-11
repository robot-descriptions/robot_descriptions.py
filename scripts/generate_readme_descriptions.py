#!/usr/bin/env python3

"""Generate the robot descriptions tables in the readme."""

from __future__ import annotations

import argparse
from pathlib import Path

from robot_descriptions._descriptions import DESCRIPTIONS, Description, Format
from robot_descriptions._repositories import REPOSITORIES

PRIMARY_CATEGORY_TAGS = [
    ("arm", "Arms"),
    ("biped", "Bipeds"),
    ("dual_arm", "Dual arms"),
    ("drone", "Drones"),
    ("educational", "Educational"),
    ("end_effector", "End effectors"),
    ("humanoid", "Humanoids"),
    ("mobile_manipulator", "Mobile manipulators"),
    ("quadruped", "Quadrupeds"),
    ("wheeled", "Wheeled"),
]

BEGIN_MARKER = "<!-- BEGIN GENERATED DESCRIPTION TABLES -->"
END_MARKER = "<!-- END GENERATED DESCRIPTION TABLES -->"
README_PATH = Path(__file__).resolve().parents[1] / "README.md"
OPEN_SOURCE_LICENSES = {
    "Apache-2.0",
    "BSD",
    "BSD-2-Clause",
    "BSD-3-Clause",
    "BSD-3-Clause-Clear",
    "Clear BSD",
    "GPL-3.0",
    "LGPL-3.0",
    "MIT",
    "NASA-1.3",
    "Zlib",
}
COMPATIBLE_LICENSES = {
    "CC-BY-4.0",
}


def _display_license(license_spdx: str) -> str:
    """Annotate a license's SPDX identifier for the readme.

    Open-source licenses are returned as-is; others get a cross-mark suffix.

    Args:
        license_spdx: SPDX identifier of the license.

    Returns:
        The license string to display in the README.
    """
    if (
        license_spdx in OPEN_SOURCE_LICENSES
        or license_spdx in COMPATIBLE_LICENSES
    ):
        return license_spdx
    return f"{license_spdx} ✖️"


def _format_cell(description: Description, column: str, name: str) -> str:
    """Render a single table cell for a given description and column.

    Args:
        description: Metadata of the robot description.
        column: Name of the table column.
        name: Package name of the robot description.

    Returns:
        The rendered Markdown table cell.

    Raises:
        ValueError: If the column name is unknown.
    """
    if column == "Name":
        return f"`{name}`"
    if column == "Robot":
        return description.robot
    if column == "Maker":
        return description.maker or ""
    if column == "DOF":
        return "" if description.dof is None else str(description.dof)
    if column == "Format":
        return ", ".join(
            description_format.name
            for description_format in Format
            if description_format in description.formats
        )
    if column == "License":
        if not description.license_spdx:
            return ""
        display_license = _display_license(description.license_spdx)
        if description.license_file and description.repository:
            repository = REPOSITORIES[description.repository]
            repo_url = repository.url.removesuffix(".git")
            url = (
                f"{repo_url}/blob/{repository.commit}/"
                f"{description.license_file}"
            )
            return f"[{display_license}]({url})"
        return display_license
    raise ValueError(column)


def _has_category(
    description: Description, category_tag: str, name: str
) -> bool:
    """Check whether a description belongs to a given category.

    A description matches if it carries the category tag itself. It does not
    match if it carries a different primary category tag; otherwise, it has no
    category tag at all and a ValueError is raised.

    Args:
        description: Metadata of the robot description.
        category_tag: Tag of the category to check against.
        name: Package name of the robot description.

    Returns:
        True if the description belongs to the category.

    Raises:
        ValueError: If the description has no category tag at all.
    """
    if category_tag in description.tags:
        return True
    if any(tag in description.tags for tag, _ in PRIMARY_CATEGORY_TAGS):
        return False
    raise ValueError(f"{name} has no category tag")


def render_tables() -> str:
    """Render the Markdown description tables for all categories.

    Returns:
        The rendered Markdown tables, one section per category.
    """
    sections: list[str] = []
    for category_tag, category in PRIMARY_CATEGORY_TAGS:
        rows = sorted(
            (name, description)
            for name, description in DESCRIPTIONS.items()
            if _has_category(description, category_tag, name)
        )
        if not rows:
            continue

        columns = ["Name", "Robot"]
        if any(description.maker for _, description in rows):
            columns.append("Maker")
        if any(description.dof is not None for _, description in rows):
            columns.append("DOF")
        columns.append("Format")
        if any(description.license_spdx for _, description in rows):
            columns.append("License")

        rendered_rows: list[dict[str, str]] = []
        widths = {column: len(column) for column in columns}
        for name, description in rows:
            rendered = {
                column: _format_cell(description, column, name)
                for column in columns
            }
            rendered_rows.append(rendered)
            for column, value in rendered.items():
                widths[column] = max(widths[column], len(value))

        lines = [f"### {category}", ""]
        lines.append(
            "| "
            + " | ".join(column.ljust(widths[column]) for column in columns)
            + " |"
        )
        lines.append(
            "|"
            + "|".join("-" * (widths[column] + 2) for column in columns)
            + "|"
        )
        for rendered in rendered_rows:
            lines.append(
                "| "
                + " | ".join(
                    rendered[column].ljust(widths[column])
                    for column in columns
                )
                + " |"
            )
        sections.append("\n".join(lines))
    return "\n\n".join(sections) + "\n"


def update_readme(readme_text: str) -> str:
    """Replace the generated tables between the markers in the README text.

    Args:
        readme_text: Current content of the README.

    Returns:
        The updated README text.
    """
    generated = render_tables()
    replacement = f"{BEGIN_MARKER}\n\n{generated}{END_MARKER}"
    start = readme_text.index(BEGIN_MARKER)
    end = readme_text.index(END_MARKER) + len(END_MARKER)
    return readme_text[:start] + replacement + readme_text[end:]


def main() -> int:
    """Regenerate the README tables, or check they are up to date with --check.

    Returns:
        Process exit code: 0 on success, 1 if the --check fails.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    readme_text = README_PATH.read_text()
    updated = update_readme(readme_text)
    if args.check:
        return 0 if updated == readme_text else 1

    README_PATH.write_text(updated)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
