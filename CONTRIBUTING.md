# 👷 Contributing

This project's goal is to facilitate loading and sharing of robot descriptions for the whole robotics communities. All contributions are welcome, for example here are some ways to help:

- Add a new robot description
- Raise any issue you find in a description, preferably directly in the description's repository (check out [`_repositories.py`](https://github.com/stephane-caron/robot_descriptions.py/blob/master/robot_descriptions/_repositories.py))
- Help with any [open issue](https://github.com/stephane-caron/robot_descriptions.py/issues?q=is%3Aissue+is%3Aopen)
- Make a standalone ``<robot_name>_description`` repository for a description embedded in one of the big framework repositories (Bullet, Drake, ...)

## Adding a new robot description

Make sure that your pull request satisfies the following:

- The robot description is distributed legally and under an open source license (permissive or copyleft)
- The file name for the new submodule is ``<robot_name>_description.py`` in snake-case. For example, ``R2D2_description`` becomes ``r2d2_description.py``
- If applicable, add the repository containing the new description to ``_repositories.py``
    - If release tags are not available, use a specific git commit ID. This way the imported submodule will still work in the interval between a change in the file structure of the target repository and the corresponding update in `robot_descriptions`.
- Add the submodule name to ``DESCRIPTION_NAMES`` in ``_descriptions.py``
- Check that all unit tests are successful by ``tox -e py3X`` where ``3.X`` is the version of your Python interpreter
- Document the description's submodule name in the Descriptions section of the [README](README.md)
- Write down the new model to the [changelog](CHANGELOG.md)
