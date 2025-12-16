---
title: Cognitive Planning with LLMs for Robotics
author: Syed Ahmed Raza
sidebar_position: 14
---

# Cognitive Planning with LLMs for Robotics: Translating Natural Language to Actions

Large Language Models (LLMs) like GPT-4o are not only capable of generating human-like text but also possess emergent reasoning and planning capabilities. This opens up exciting possibilities for robotics, allowing robots to interpret high-level natural language commands ("Clean the room") and translate them into a sequence of low-level, executable robot actions. This module explores the "Plan-Execute" pattern, leveraging LLMs for cognitive planning in a ROS 2 environment.

## The Challenge of High-Level Commands

Traditional robotics relies on meticulously programmed sequences or intricate state machines for task execution. However, humans naturally interact with robots using high-level, abstract commands. Bridging this gap requires a sophisticated planning mechanism that can:
1.  **Understand natural language**: Interpret ambiguous or underspecified commands.
2.  **Decompose tasks**: Break down complex goals into smaller, manageable sub-goals.
3.  **Ground actions**: Map abstract concepts to concrete, robot-executable primitives.
4.  **Handle uncertainty**: Adapt plans based on real-time sensory feedback and environmental changes.

LLMs, with their vast knowledge base and reasoning abilities, are proving to be powerful tools for this cognitive planning.

## The Plan-Execute Pattern with LLMs

The "Plan-Execute" pattern is a robust approach to integrating LLMs into robotic control. It involves two primary phases that iterate until a task is completed:

1.  **Plan Phase**:
    *   The robot receives a natural language command (e.g., "Bring me a cup from the kitchen").
    *   An LLM is queried with the command, along with information about the robot's current state, available tools/objects, and a list of primitive robot actions it can perform (e.g., `navigate_to(location)`, `pick_up(object)`, `place_down(object)`).
    *   The LLM generates a high-level plan, often as a sequence of these primitive actions, or as a textual description of steps.
    *   The plan is then validated and converted into a machine-readable format if necessary.

2.  **Execute Phase**:
    *   The robot executes the first action in the plan.
    *   Sensors are monitored to confirm the action's success or failure.
    *   The robot's state is updated based on the execution outcome.
    *   If the action succeeds, the next action in the plan is considered. If it fails, the robot can either:
        *   Request the LLM to replan from the current state.
        *   Execute a predefined recovery behavior.

This cycle continues until the overall task is successfully completed or an unrecoverable error occurs.

## Leveraging LLMs for Task Decomposition and Grounding

### LLM Prompt Engineering for Planning

The quality of the LLM's plan heavily depends on the prompt. A well-engineered prompt for robotic planning often includes:
-   **Task Description**: The natural language command.
-   **Robot Capabilities**: A list of primitive actions the robot can execute, including their parameters and preconditions.
-   **Environment State**: Current sensor readings, object locations, robot pose.
-   **Goal Definition**: Clear criteria for task completion.
-   **Output Format**: Explicit instructions for the LLM to output the plan in a structured, parseable format (e.g., JSON, YAML, or a specific function call format).

### Example Prompt (Conceptual for GPT-4o)

```
You are a robotic task planner. Your goal is to convert high-level natural language commands into a sequence of low-level robot actions.

Available actions:
- navigate_to(location: str): Move the robot to a specified location.
- pick_up(object_name: str): Pick up a specified object.
- place_down(object_name: str, location: str): Place down a specified object at a location.
- find_object(object_name: str): Use sensors to locate an object.

Current state:
- Robot location: Living Room
- Objects visible: [Ball, Book]
- Objects in hand: None
- Locations: [Living Room, Kitchen, Bedroom]

Command: "Clean the living room by putting the ball in the box."

Output your plan as a Python list of action dictionaries. For example:
[{"action": "navigate_to", "parameters": {"location": "Kitchen"}}]
```

LLM Response (Example):
```json
[
  {"action": "find_object", "parameters": {"object_name": "Ball"}},
  {"action": "pick_up", "parameters": {"object_name": "Ball"}},
  {"action": "find_object", "parameters": {"object_name": "Box"}},
  {"action": "place_down", "parameters": {"object_name": "Ball", "location": "Box"}}
]
```

## Integrating with ROS 2: A Plan-Execute ROS 2 Node

In a ROS 2 system, the Plan-Execute pattern can be implemented as a central planning node that communicates with an LLM and dispatches actions to other specialized robot control nodes.

```mermaid
graph TD
    A[Human User] -- Voice Command --> B(Voice Recognition Node - Whisper)
    B --> C(Transcribed Text Topic)
    C --> D[LLM Planning Node]
    D -- Query (Command, State, Actions) --> E[LLM API (GPT-4o)]
    E -- Plan (Sequence of Actions) --> D
    D --> F(Action Dispatcher Node)
    F -- ROS 2 Action Call (e.g., navigate_to_action) --> G[Navigation Action Server]
    F -- ROS 2 Action Call (e.g., pick_up_action) --> H[Manipulation Action Server]
    G & H -- Feedback/Result --> F
    F --> D -- (Update State)
    D --> I[Robot State Publisher]
    I --> F & D
```

### ROS 2 `llm_planner_node` Responsibilities:
-   **Subscription**: Listens to the `voice_command_text` topic (from the Whisper node) and robot state topics.
-   **LLM Interface**: Manages API calls to the LLM, crafts prompts, and parses LLM responses.
-   **Plan Validation**: Checks if the LLM's generated actions are valid and executable by the robot.
-   **Action Dispatch**: Initiates ROS 2 Actions or Service calls to lower-level control nodes.
-   **State Management**: Tracks the robot's progress and current state for subsequent planning iterations.

This approach transforms natural language commands into intelligent, adaptive robotic behavior, marking a significant step towards truly autonomous and user-friendly robots.
```