# 👷 Contributing

![PRs welcome](https://img.shields.io/badge/PRs-welcome-green.svg)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

The goal of this project is to facilitate loading and sharing of robot descriptions for the robotics community. All contributions are welcome. For example, here are some ways to help:

- [Add a new robot description](#adding-a-new-robot-description)
- Raise any issue you find in a description, preferably directly in the description's repository (check out [`_repositories.py`](https://github.com/stephane-caron/robot_descriptions.py/blob/main/robot_descriptions/_repositories.py))
- Make a standalone `<robot_name>_description` repository for a description embedded in one of the big framework repositories (Bullet, Drake, ...)

## Adding a new robot description

1. **License:** The robot description is distributed legally and under an open source license (permissive or copyleft).
2. **Repository:** If needed, add the repository containing the new description to `_repositories.py`.
    - Use a specific git commit ID. This way the robot description will still work in the interval between a change in the file structure of the target repository and the corresponding update in `robot_descriptions`.
3. **Submodule:** add a Python file for the robot descriptions to `robot_descriptions/`.
    - The file name for the new submodule is `<robot_name>_description.py` in snake-case.
    - For example, the file name for the Kinova (maker) Gen2 (robot name) is `gen2_description.py`.
    - Use the `mj_description` suffix for an MJCF description.
4. **Listing:** Add the description metadata to the `DESCRIPTIONS` dictionary in `_descriptions.py`.
    - Include the metadata fields there as well: robot name, maker, DOF (when applicable), SPDX license identifier, and optionally the path to the license file in the source repository.
5. **Visualize:** At this point you can check that the new description loads and renders properly using a visualizer, for example:

```
uv run --with yourdfpy --with "pyglet<2" python -m robot_descriptions show_in_yourdfpy <new_description>
uv run --with mujoco-python-viewer python -m robot_descriptions show_in_mujoco <new_mj_description>
```

6. **README:** Regenerate the Descriptions section of the [README](README.md):

```
python scripts/generate_readme_descriptions.py
```

   - Use an [SPDX License Identifier](https://spdx.org/licenses/) in `license_spdx`.
   - When you know the license file location, store its repository-relative path in `license_file` so the README link can be derived automatically.
7. **CHANGELOG:** Write down the new model at the top of the [changelog](CHANGELOG.md).
8. **Testing:** Check that all unit tests are successful by `tox`.
