# Robot descriptions as Python modules

[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)

Import open source [robot descriptions](https://github.com/stephane-caron/awesome-robot-descriptions) as Python modules.

## Installation

```console
pip install robot_descriptions
```

## Usage

Import the robot description you are interested in directly as a submodule of ``robot_descriptions``:

```python
from robot_descriptions import my_robot_description
```

The import will automatically download and cache the robot model, if you don't
have it already. The submodule then provides the following paths:

<dl>
    <dt>
        <code>my_robot_description.MESHES_PATH</code>
    </dt>
    <dd>
        Path to a "meshes" folder in the robot description.
    </dd>
    <dt>
        <code>my_robot.PATH</code>
    </dt>
    <dd>
        Path to the robot description directory.
    </dd>
    <dt>
        <code>my_robot_description.URDF_PATH</code>
    </dt>
    <dd>
        Path to the URDF file of the model.
    </dd>
</dl>
