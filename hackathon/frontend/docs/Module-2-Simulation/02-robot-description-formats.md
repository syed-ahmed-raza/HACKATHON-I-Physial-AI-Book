---
title: Robot Description Formats (URDF vs SDF)
author: Syed Ahmed Raza
sidebar_position: 6
---

# Robot Description Formats: URDF vs. SDF

In robotics, precisely describing the physical and kinematic properties of a robot is fundamental for both simulation and real-world control. Robot Description Formats provide standardized ways to represent a robot's links, joints, sensors, and other attributes. The two most prominent formats in the ROS ecosystem are URDF (Unified Robot Description Format) and SDF (Simulation Description Format).

## URDF (Unified Robot Description Format)

URDF is an XML-based file format designed primarily for describing the kinematic and dynamic properties of a robot, particularly for use with ROS. It is well-suited for representing manipulators and mobile robots with a tree-like (acyclic) structure.

### Key Characteristics:
-   **Single Robot**: URDF is designed to describe a single robot.
-   **Tree Structure**: Represents the robot as a tree, with a single root link and subsequent links connected by joints. Loops are not allowed.
-   **Kinematics and Dynamics**: Focuses on `links` (rigid bodies) and `joints` (connections between links) with their inertial, visual, and collision properties.
-   **ROS Integration**: Tightly integrated with ROS tools for visualization (RViz), kinematics solvers (MoveIt!), and robot state publishing.
-   **Limited Environmental Description**: Does not support describing environments or multiple robots within a single file.
-   **Simulation Limitations**: While it can be converted to other formats for simulation, URDF itself lacks native support for describing environmental elements, physics engine parameters, or dynamic elements like actuators and sensors beyond basic kinematic attachment.

### URDF Structure Components:
-   **`<link>`**: Defines a rigid body segment of the robot.
    -   `inertial`: Mass, center of mass, inertia matrix.
    -   `visual`: 3D model (mesh), color, texture for rendering.
    -   `collision`: 3D model (mesh) for physics collisions.
-   **`<joint>`**: Defines a connection between two links.
    -   `type`: (revolute, continuous, prismatic, fixed, floating, planar).
    -   `parent`, `child`: Specifies the connected links.
    -   `origin`: Joint position and orientation relative to the parent link.
    -   `axis`: Axis of rotation/translation.
    -   `limit`: Joint position/velocity/effort limits.
    -   `calibration`, `dynamics`, `mimic`, `safety_controller`: Additional joint properties.

### Simplified XML for a Humanoid Leg Link (URDF)

```xml
<robot name="humanoid_leg">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
  </link>

  <link name="thigh_link">
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="hip_pitch_joint" type="revolute">
    <parent link="base_link"/>
    <child link="thigh_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>
</robot>
```
*This snippet shows a simplified `base_link` and `thigh_link` connected by a revolute `hip_pitch_joint` for a humanoid leg segment.*

## SDF (Simulation Description Format)

SDF is also an XML-based file format, but it is specifically designed for comprehensive description of robots and environments for use with Gazebo. It is a more powerful and expressive format than URDF for simulation purposes.

### Key Characteristics:
-   **Full Simulation Capabilities**: Supports describing environments, lights, sensors, physics properties, and multiple robots within a single file.
-   **Graph Structure**: Can represent both tree-like and graph-like structures, allowing for more complex robot topologies and environmental interactions.
-   **Actuators and Sensors**: Provides native tags for comprehensive sensor descriptions (e.g., camera, LiDAR, IMU properties) and actuator modeling.
-   **Gazebo Native**: The native format for Gazebo, ensuring full compatibility and optimal performance within the simulator.
-   **Backward Compatibility**: Can embed URDF models, though converting a URDF to a full SDF often requires additional specification for simulation-specific elements.

### SDF Structure Components (in addition to URDF concepts):
-   **`<world>`**: The top-level element for a simulation environment.
-   **`<model>`**: Can describe both robots and static environmental objects.
-   **`<joint>`**: Similar to URDF but often with more advanced physics properties.
-   **`<sensor>`**: Detailed descriptions of various sensor types (camera, lidar, imu, contact, force_torque, etc.) including noise models, update rates, and output topics.
-   **`<plugin>`**: Allows for extending Gazebo's functionality, often used for ROS 2 integration (e.g., `libgazebo_ros_force_torque.so`).
-   **`<light>`**: Comprehensive light source definitions.
-   **`<physics>`**: Configuration for the underlying physics engine (ODE, Bullet, DART, Simbody).

## URDF vs. SDF: When to Use Which?

| Feature             | URDF                                           | SDF                                                    |
| :------------------ | :--------------------------------------------- | :----------------------------------------------------- |
| **Primary Use**     | Robot description for ROS, visualization, planning. | Full robot and environment description for Gazebo simulation. |
| **Structure**       | Tree (single root, no loops).                  | Full graph (can handle loops).                         |
| **Environment**     | No (single robot only).                        | Yes (full world description).                          |
| **Sensors**         | Basic sensor `joint` attachment.               | Rich, native sensor models with physics properties.     |
| **Actuators**       | Implicit via joint limits.                     | Explicit actuator models.                              |
| **Physics**         | No native physics engine configuration.        | Native physics engine configuration (ODE, Bullet, etc.). |
| **Multi-robot**     | No (single robot per file).                    | Yes (multiple robots and objects in one world file).   |

**Conclusion**:
-   Use **URDF** when you primarily need to describe a robot's kinematics, visuals, and collisions for ROS tools like RViz, MoveIt!, and basic robot state publication. It's excellent for robot visualization and control stack development.
-   Use **SDF** when you need a comprehensive description for high-fidelity simulation in Gazebo, including complex environments, multiple robots, detailed sensor models, and specific physics engine tuning.

For a full robotic system, it's common to define the robot using URDF and then either convert it to SDF (often implicitly by Gazebo's ROS packages) or embed the URDF within an SDF `.world` file, adding the necessary simulation-specific elements (like detailed sensor plugins and world models) directly in SDF. This hybrid approach leverages the strengths of both formats.
```