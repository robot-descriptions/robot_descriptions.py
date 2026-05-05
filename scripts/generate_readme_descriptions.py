#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from robot_descriptions._descriptions import DESCRIPTIONS, Description
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
FREE_LICENSES = {
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


def _display_license(license_spdx: str) -> str:
    if license_spdx in FREE_LICENSES:
        return license_spdx
    return f"{license_spdx} ✖️"


def _format_cell(description: Description, column: str, name: str) -> str:
    if column == "Name":
        return f"`{name}`"
    if column == "Robot":
        return description.robot
    if column == "Maker":
        return description.maker or ""
    if column == "DOF":
        return "" if description.dof is None else str(description.dof)
    if column == "Format":
        return "MJCF" if description.has_mjcf else "URDF"
    if column == "License":
        if not description.license_spdx:
            return ""
        display_license = _display_license(description.license_spdx)
        if description.license_file and description.repository:
            repository = REPOSITORIES[description.repository]
            repo_url = repository.url.removesuffix(".git")
            url = f"{repo_url}/blob/{repository.commit}/{description.license_file}"
            return f"[{display_license}]({url})"
        return display_license
    raise ValueError(column)


def _has_category(description: Description, category_tag: str, name: str) -> bool:
    if category_tag in description.tags:
        return True
    if any(tag in description.tags for tag, _ in PRIMARY_CATEGORY_TAGS):
        return False
    raise ValueError(f"{name} has no category tag")


def render_tables() -> str:
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
    generated = render_tables()
    replacement = f"{BEGIN_MARKER}\n\n{generated}{END_MARKER}"
    start = readme_text.index(BEGIN_MARKER)
    end = readme_text.index(END_MARKER) + len(END_MARKER)
    return readme_text[:start] + replacement + readme_text[end:]


def main() -> int:
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
