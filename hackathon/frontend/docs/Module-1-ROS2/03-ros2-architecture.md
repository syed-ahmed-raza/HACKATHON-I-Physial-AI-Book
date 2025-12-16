---
title: ROS 2 Architecture
author: Syed Ahmed Raza
sidebar_position: 3
---

# ROS 2 Architecture: The Robotic Nervous System

ROS 2 (Robot Operating System 2) provides a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that simplify the task of creating complex and robust robot applications. At its core, ROS 2 facilitates communication between different parts of a robot's software system through a distributed, message-passing architecture.

## Key Communication Concepts in ROS 2

ROS 2's architecture is built around several fundamental communication primitives, each designed for specific interaction patterns. Unlike ROS 1 which used a centralized ROS Master, ROS 2 leverages a Distributed Data Service (DDS) for decentralized, peer-to-peer communication, offering improved real-time capabilities, security, and scalability.

### Nodes: The Modular Building Blocks

A **Node** is an executable process in a ROS 2 system. It represents a single functional unit of a robot's software, responsible for a specific task. By breaking down the robot's control system into smaller, modular nodes, development becomes easier, code is more reusable, and debugging is simplified.

*   **Example**: A robot might have a node for reading LiDAR data, another for controlling motors, and a third for path planning.
*   **Best Practice**: Each node should ideally adhere to the Single Responsibility Principle, focusing on one primary task.

### Topics: Asynchronous Data Streaming

**Topics** provide a publish/subscribe communication model, enabling asynchronous, one-to-many data streaming. A node that produces data **publishes** messages to a topic, while nodes that consume that data **subscribe** to the same topic. This is ideal for continuous streams of data, such as sensor readings, odometry, or video feeds.

*   **Communication Pattern**: Publisher -> Topic -> Subscriber (one-to-many, asynchronous)
*   **Data Flow**: Messages flow in one direction, from publisher to subscribers.
*   **Example**: A `lidar_node` might publish sensor readings to the `/scan` topic, and a `mapping_node` and `obstacle_avoidance_node` would subscribe to `/scan` to receive this data.
*   **Message Types**: Standardized message types (e.g., `sensor_msgs/msg/LaserScan`, `geometry_msgs/msg/Twist`) ensure interoperability.

### Services: Synchronous Request/Reply

**Services** implement a synchronous request/reply communication model. When a node needs a specific computation or action performed by another node and requires an immediate response, it makes a **service call**. The called node (the **server**) performs the requested operation and returns a **response**. This is suitable for discrete, blocking operations that require a direct result.

*   **Communication Pattern**: Client -> Service Request -> Service Server -> Service Response -> Client (one-to-one, synchronous)
*   **Data Flow**: Request and Response messages.
*   **Example**: A `motion_planning_node` might call a `gripper_control_service` to open or close the gripper, waiting for confirmation before proceeding.
*   **Service Types**: Defined by a request message and a response message.

### Actions: Long-Running Goal-Based Tasks

**Actions** are designed for long-running, pre-emptable, goal-based tasks. They provide feedback during execution and can be cancelled. An action consists of a **goal** (what to achieve), **feedback** (progress updates), and a **result** (outcome upon completion). This is ideal for tasks like navigating to a location, picking up an object, or performing a complex manipulation sequence.

*   **Communication Pattern**: Action Client -> Goal -> Action Server -> (Feedback loop) -> Result -> Action Client (asynchronous, goal-based, pre-emptable)
*   **Data Flow**: Goal message, Feedback message stream, Result message.
*   **Example**: A `navigate_to_pose_action_client` sends a goal to a `navigation_action_server` to reach a specific (x,y,theta) pose. The server provides continuous feedback on progress and the final result (success/failure).
*   **Action Types**: Defined by a goal message, a feedback message, and a result message.

## ROS 2 Graph Representation

The interaction between nodes, topics, services, and actions forms the computational graph of a ROS 2 system. Visualization tools like `rqt_graph` can illustrate this dynamic network.

```mermaid
graph TD
    subgraph Robot Sensors
        A[LiDAR Node] --> |/scan (LaserScan)| B(Scan Topic)
        C[IMU Node] --> |/imu_data (Imu)| D(IMU Topic)
    end

    subgraph Robot Perception
        B --> |/scan| E[Mapping Node]
        D --> |/imu_data| F[Localization Node]
        E --> |/map (OccupancyGrid)| G(Map Topic)
        F --> |/odom (Odometry)| H(Odometry Topic)
    end

    subgraph Robot Control
        I[Path Planning Node] --> |/cmd_vel (Twist)| J(Velocity Command Topic)
        J --> K[Motor Control Node]
    end

    subgraph Gripper System
        L[Gripper Control Service Server] -- "open_gripper (Request/Response)" --> M[Task Execution Node]
    end

    subgraph Navigation System
        M -- "navigate_to_pose (Goal/Feedback/Result)" --> N[Navigation Action Server]
    end

    I --- G & H
```

In this diagram:
-   Rectangles represent Nodes (e.g., `LiDAR Node`, `Mapping Node`).
-   Circles represent Topics (e.g., `Scan Topic`, `IMU Topic`).
-   Arrows indicate data flow.
-   `-- "Text"` indicates Service or Action communication, with the text denoting the service/action name and type.

This modular, distributed architecture allows for robust, concurrent, and scalable robot software development, where developers can focus on individual functionalities that seamlessly integrate into a larger, intelligent system.
```