# 👷 Contributing

There are many ways you can contribute to `robot_descriptions`. Here are some ideas:

- Check out [open issues](https://github.com/stephane-caron/robot_descriptions.py/issues?q=is%3Aissue+is%3Aopen)
- Add a new robot description
- Make a standalone ``<robot_name>_description`` repository for a description embedded in one of the big framework repositories in ``_repositories.py`` (Bullet, Drake, ...)

## Adding a new robot description

Make sure that your pull request satisfies the following:

- The robot description is distributed legally and under an open source license (permissive or copyleft)
- The file name for the new submodule is ``<robot_name>_description.py`` in snake-case. For example, ``R2D2_description`` becomes ``r2d2_description.py``
- If applicable, add the repository containing the new description to ``_repositories.py``
    - If release tags are not available, use a specific git commit ID. This way the imported submodule will still work in the interval between a change in the file structure of the target repository and the corresponding update in `robot_descriptions`.
- All three ``MESHES_PATH``, ``PATH`` and ``URDF_PATH`` fields are set
- Add the submodule name to ``DESCRIPTION_NAMES`` in ``_descriptions.py``
- Document the model's submodule name in the Models section of the [README](README.md)
- Report the new model to the [changelog](CHANGELOG.md)
