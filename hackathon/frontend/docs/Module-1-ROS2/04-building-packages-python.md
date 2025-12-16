---
title: Building ROS 2 Packages with Python
author: Syed Ahmed Raza
sidebar_position: 4
---

# Building ROS 2 Packages with Python: A Practical Guide

Developing in ROS 2 revolves around the concept of "packages." A package is a directory that contains a collection of nodes, libraries, configuration files, and other resources that together provide a specific functionality. For Python development in ROS 2, `rclpy` is the client library that enables interaction with the ROS 2 graph.

## Creating a ROS 2 Python Package

To create a new ROS 2 Python package, you typically use the `ros2 pkg create` command. This command sets up the basic directory structure and necessary build files.

```bash
# Navigate to your workspace's src directory
cd ~/ros2_ws/src

# Create a new Python package named 'my_robot_pkg'
# --build-type ament_python is essential for Python packages
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy
```

This will create a directory structure similar to this:

```
my_robot_pkg/
├── package.xml             # Package metadata
├── setup.py                # Python build configuration
├── resource/               # Contains marker file for ament
├── my_robot_pkg/           # Python module directory
│   └── __init__.py         # Python package marker
└── test/                   # Test files
```

## Writing a Basic ROS 2 Python Node (`rclpy`)

A ROS 2 node is an executable that performs computation. Here’s a simple example of a Python node using `rclpy` that publishes a "Hello ROS 2" message to a topic every second.

### `my_robot_pkg/my_robot_pkg/simple_publisher_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')  # Initialize the node with a name
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Create a publisher
        self.i = 0
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Create a timer

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"') # Log message
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy library
    simple_publisher = SimplePublisher() # Create the node
    rclpy.spin(simple_publisher) # Keep the node alive
    simple_publisher.destroy_node() # Destroy the node when done
    rclpy.shutdown() # Shutdown rclpy library

if __name__ == '__main__':
    main()
```

## Configuring `setup.py`

For ROS 2 to know about your executable Python nodes, you need to add an entry point in `setup.py`. This tells ROS 2 how to find and run your script.

```python
# my_robot_pkg/setup.py
from setuptools import find_packages, setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_launch.py']), # Add this line for launch files
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple ROS 2 Python package example.',
    license='Apache-2.0', # Or appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_pkg.simple_publisher_node:main', # Add this line for your node
        ],
    },
)
```

## Launch Files: Orchestrating Your ROS 2 System

**Launch files** are XML or Python files used to start and configure multiple ROS 2 nodes, set parameters, and remap topics/services in a coordinated manner. They are essential for managing complex robot systems.

### `my_robot_pkg/launch/simple_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='simple_publisher',
            name='my_simple_publisher',
            output='screen',
            emulate_tty=True, # Required for seeing print statements in the console
            parameters=[
                {'param_name': 'param_value'} # Example parameter
            ]
        )
    ])
```

## Building and Running Your Package

After creating or modifying your package, you need to build your workspace and then you can run your node or launch file.

```bash
# From your workspace root (e.g., ~/ros2_ws)
colcon build --packages-select my_robot_pkg

# Source the setup files to make the package discoverable
source install/setup.bash # or setup.ps1 for PowerShell

# Run the node directly
ros2 run my_robot_pkg simple_publisher

# Or run via the launch file
ros2 launch my_robot_pkg simple_launch.py
```

This practical guide demonstrates the fundamental steps to create, implement, and orchestrate a Python-based ROS 2 package, forming the building blocks for more advanced robotic applications.
```