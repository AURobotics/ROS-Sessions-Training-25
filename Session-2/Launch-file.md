# ROS 2 Launch Files 

Launch files in ROS 2 are used to **start multiple nodes and configure them** from a single file.  
They are especially useful for setting parameters, remapping topics, and organizing large systems.

---

## 1. Basic Launch Command

Run a launch file with:

```bash
ros2 launch mypkg my_launch.py
```

---

## 2. Minimal Launch File Example

**`launch/my_launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        )
    ])
```

This starts a single `turtlesim_node`.

---
### Configuring `setup.py`
To enable colcon to locate and utilize our launch files, we need to inform Python’s setup tools of their presence. To achieve this, open the setup.py file, add the necessary import statements at the top, and include the launch files into the data_files parameter of setup:

### `setup.py` File:
```bash
import os
from glob import glob
# Other imports ...

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ]
)
```

## 3. Launching Multiple Nodes

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop'
        )
    ])
```

---

## 4. Setting Parameters in Launch Files

You can pass parameters directly:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypkg',
            executable='param_node',
            name='param_node',
            parameters=[{
                'robot_name': 'victor',
                'robot_speed': 2.0,
                'manual_mode': False
            }]
        )
    ])
```

Or load from a YAML file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mypkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mypkg',
            executable='param_node',
            name='param_node',
            parameters=[config]
        )
    ])
```

---

## 5. Remapping Topics

You can also change topic names in a launch file:

```python
Node(
    package='turtlesim',
    executable='turtlesim_node',
    name='sim',
    remappings=[('/turtle1/cmd_vel', '/my_cmd_vel')]
)
```