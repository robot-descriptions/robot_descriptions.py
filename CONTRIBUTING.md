# 👷 Contributing

This project's goal is to facilitate loading and sharing of robot descriptions for the whole robotics communities. All contributions are welcome, for example here are some ways to help:

- Add a new robot description
- Raise any issue you find in a description, preferably directly in the description's repository (check out [`_repositories.py`](https://github.com/stephane-caron/robot_descriptions.py/blob/master/robot_descriptions/_repositories.py))
- Make a standalone ``<robot_name>_description`` repository for a description embedded in one of the big framework repositories (Bullet, Drake, ...)

## Adding a new robot description

1. **License:** The robot description is distributed legally and under an open source license (permissive or copyleft).
2. **Repository:** If needed, add the repository containing the new description to ``_repositories.py``.
    - Use a specific git commit ID. This way the robot description will still work in the interval between a change in the file structure of the target repository and the corresponding update in `robot_descriptions`.
3. **Submodule:** add a Python file for the robot descriptions to ``robot_descriptions/``.
    - The file name for the new submodule is ``<robot_name>_description.py`` in snake-case.
    - For example, the file name for the Kinova (maker) Gen2 (robot name) is ``gen2_description.py``.
    - Use the ``mj_description`` suffix for an MJCF description.
4. **Listing:** Add the description metadata to the ``DESCRIPTIONS`` dictionary in ``_descriptions.py``.
5. **Testing:** Check that all unit tests are successful by ``tox -e py``.
6. **README:** Document the description's submodule name in the Descriptions section of the [README](README.md).
7. **Changelog:** Write down the new model to the [changelog](CHANGELOG.md).
