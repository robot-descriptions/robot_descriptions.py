# 👷 Contributing

There are many ways you can contribute to `robot_descriptions`. Here are some ideas:

- Check out [help-wanted issues](https://github.com/stephane-caron/robot_descriptions/issues?q=is%3Aissue+is%3Aopen+label%3A%22help+wanted%22)
- Add a new robot description

## Adding a new robot description

Make sure of the following:

- The file name for the new submodule is ``<robot_name>_description.py`` in snake-case
- The description is distributed legally and under an open source license (permissive or copyleft)
- Format can be URDF, MJCF, XML, ...

Checkout the specific tag of the latest release

```python
__version__ = "1.1.0"
__repo__ = _git_clone_description(...)
__repo__.git.checkout(f"v{__version__}")
```

If release tags are not available in the target robot description, use a specific git commit ID. This way the imported submodule will still work in the interval between a change in the file structure of the target repository and the corresponding update in `robot_descriptions`.
