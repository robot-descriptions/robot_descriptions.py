# Robot descriptions as Python modules

[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)

Import open source robot models as Python modules. The wrapper automatically downloads and cache model files at first import. URDF descriptions from [Awesome Robot Models](https://github.com/stephane-caron/awesome-robot-models) are available.

## Installation

```console
pip install robot_descriptions
```

## Usage

Import the robot description you are interested in directly as a submodule of ``robot_descriptions``:

```python
from robot_descriptions import my_robot_description
```

The import will automatically download and cache the robot model, if you don't have it already. The submodule then exports the following paths:

<dl>
    <dt>
        <code>MESHES_PATH</code>
    </dt>
    <dd>
        Path to the "meshes" folder in the robot description.
    </dd>
    <dt>
        <code>PATH</code>
    </dt>
    <dd>
        Path to the root directory of the robot description.
    </dd>
    <dt>
        <code>URDF_PATH</code>
    </dt>
    <dd>
        Path to the URDF file of the robot model.
    </dd>
</dl>
