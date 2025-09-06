# ROS 2 Python Package Workshop


## 1. Create a ROS 2 Workspace

```bash
# Create workspace folder
dmkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Initialize workspace (optional, for ROS 2 Foxy and above)
colcon build
```

---

## 2. Create a Python Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies rclpy my_pkg
```

Folder structure will look like:

```
ros2_ws/
└── src/
    └── my_pkg/
        ├── my_pkg/
        │   ├── __init__.py
        │   └── my_node.py
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── resource/
```

---

## 3. `setup.py` Example

```python
from setuptools import setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A ROS 2 Python package example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_pkg.publisher_node:main',
            'listener = my_pkg.subscriber_node:main',
        ],
    },
)
```

---

## 4. Example Publisher Node (`publisher_node.py`)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        

    def timer_callback(self):
        # define message type
        # your callback logic
        # publish your msg
        
        
def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 5. Example Subscriber Node (`subscriber_node.py`)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # your callback logic


def main():
    rclpy.init()
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 6. Build and Source the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

> You can add `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` for automatic setup.

---

## 7. Run Nodes

```bash
# Terminal 1: Run Publisher
ros2 run my_pkg talker

# Terminal 2: Run Subscriber
ros2 run my_pkg listener
```

---

## 8. Useful ROS 2 Commands

```bash
# List all packages
ros2 pkg list

# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic

# Check node info
ros2 node info /minimal_publisher
```

---

## 9. Notes

- Always **source the workspace** before running nodes.  
- Use `chmod +x filename.py` for executable Python scripts if running directly.  
- Make sure your `entry_points` in `setup.py` matches the node filenames and `main()` function names.

---


