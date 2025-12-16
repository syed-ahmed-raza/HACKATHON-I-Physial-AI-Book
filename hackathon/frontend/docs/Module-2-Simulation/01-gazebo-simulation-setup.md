---
title: Gazebo Simulation Setup
author: Syed Ahmed Raza
sidebar_position: 5
---

# Gazebo Simulation Setup

Gazebo is a powerful 3D robot simulator widely used in robotics research and development. It accurately simulates robots, sensors, and environments, allowing developers to test algorithms and control systems in a safe, repeatable virtual world before deployment on physical hardware. Integrating Gazebo with ROS 2 provides a seamless framework for simulating complex robotic systems.

## Setting up Gazebo

### Installation
Gazebo is typically installed as part of a ROS 2 distribution. For example, on Ubuntu with ROS 2 Humble Hawksbill, you would install the desktop-full package:

```bash
sudo apt update
sudo apt install ros-humble-desktop-full # Or your specific ROS 2 distribution
```
This usually installs the `gazebo_ros_pkgs` which provides the necessary bridges between ROS 2 and Gazebo.

### Launching Gazebo
You can launch Gazebo with a default empty world:

```bash
gazebo # Launches the GUI
```
Or, more commonly, launch it with a specific world file via ROS 2:

```bash
ros2 launch gazebo_ros gazebo.launch.py # Launches an empty world with ROS 2 bridge
```

## Creating World Files (`.world`)

A Gazebo `.world` file is an XML-based description of the simulation environment. It defines the terrain, objects (models), lights, sensors, and physics properties of the virtual world.

### Basic World File Structure

A `.world` file typically includes:
-   **`<world>` tag**: The root element, containing all environment definitions.
-   **`<gravity>`**: Defines the gravity vector (e.g., `0 0 -9.8`).
-   **`<light>`**: Defines light sources (directional, point, spot).
-   **`<model>`**: Includes 3D models of objects (e.g., walls, tables, robots). Gazebo supports importing various 3D formats or using its own SDF (Simulation Description Format) for models.
-   **`<physics>`**: Configures the physics engine.
-   **`<gui>`**: Customizes the Gazebo GUI, such as camera position.

### Example: Simple Plane World

Here's a minimal `.world` file defining a ground plane and a sunlight:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="empty_world">
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <ambient>0.2 0.2 0.2 1</ambient>
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    <gui>
      <camera>
        <pose>-2 0 2 0 0.2 0</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

## Configuring Physics Engines (ODE/Bullet)

Gazebo supports multiple physics engines to simulate the interactions between rigid bodies. The choice of physics engine can significantly impact simulation accuracy, stability, and performance.

### Physics Engine Configuration

The `<physics>` tag in the `.world` file allows you to specify the physics engine and its parameters.

```xml
<physics name="default_physics" default="0" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
      <friction_model>cone_model</friction_model>
    </solver>
    <constraints>
      <cfm>0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### ODE (Open Dynamics Engine)
-   **Default**: ODE is the default physics engine in Gazebo. It's a robust engine suitable for a wide range of robotics applications.
-   **Configuration**: Parameters like `max_step_size` (maximum simulation time step), `real_time_factor` (how fast simulation runs relative to real-time), `iters` (number of iterations for the solver), and `sor` (Successive Over-Relaxation parameter) can be tuned for stability and performance.

### Bullet Physics
-   **High Performance**: Bullet is often chosen for high-performance simulations, especially for complex collision detection and rigid body dynamics.
-   **Usage**: To use Bullet, change the `type` attribute in the `<physics>` tag to `bullet`:
    ```xml
    <physics name="default_physics" default="0" type="bullet">
        <!-- Bullet specific configurations -->
    </physics>
    ```

### Other Engines
Gazebo also supports Simbody and DART, each with its own strengths for specific simulation needs (e.g., biomechanics for Simbody, flexible bodies for DART).

Careful selection and tuning of the physics engine parameters are essential for achieving realistic and stable robot simulations in Gazebo, which directly impacts the validity of tested control strategies and algorithms.
```