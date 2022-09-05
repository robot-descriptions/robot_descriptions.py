# 👷 Contributing

There are many ways you can contribute to `robot_descriptions`. Here are some ideas:

- Check out [help-wanted issues](https://github.com/stephane-caron/robot_descriptions/issues?q=is%3Aissue+is%3Aopen+label%3A%22help+wanted%22)
- Add a new robot description

## Adding a new robot description

Make sure that your pull request satisfies the following:

- The file name for the new submodule is ``<robot_name>_description.py`` in snake-case. For example, ``R2D2_description`` becomes ``r2d2_description.py``.
- The robot description is distributed legally and under an open source license (permissive or copyleft)
- All three ``MESHES_PATH``, ``PATH`` and ``URDF_PATH`` fields are set
- Add an entry to the [changelog](CHANGELOG.md) under "New models": ``- ROBOT_NAME robot_type (license: LICENSE)``

Checkout the specific tag of the latest release

```python
__version__ = "1.1.0"
__repo__ = _clone_to_cache(REPOSITORY_NAME)
__repo__.git.checkout(f"v{__version__}")
```

If release tags are not available in the target robot description, use a specific git commit ID. This way the imported submodule will still work in the interval between a change in the file structure of the target repository and the corresponding update in `robot_descriptions`.

- Add the submodule name to ``DESCRIPTION_NAMES`` in ``_descriptions.py``
- Document the model's submodule name in the Models section of the [README](README.md)
- Report the new model to the [changelog](CHANGELOG.md)
