# Weeks 11-12: Capstone Project - Humanoid Butler Bot

This document outlines the capstone project for the humanoid development course. The goal is to integrate all the concepts learned in the previous weeks—from robot description and control to simulation and AI—to build a simulated "Butler Bot." This robot will be capable of navigating a home environment, perceiving objects, and performing a simple fetch-and-carry task.

## 1. Project Vision & Goal

**Vision:** To create a general-purpose humanoid robot that can assist with common household chores.

**Core Goal:** Develop the software stack for a simulated humanoid robot that can respond to a user's request, find a specific object in a room, pick it up, and bring it to the user.

**Example Scenario:**
1.  The user says, "Bot, please bring me the red can from the table."
2.  The robot, starting in a different room, navigates to the kitchen.
3.  It scans the room to locate the table and the objects on it.
4.  It identifies the "red can."
5.  It approaches the table, grasps the can, and picks it up.
6.  It navigates back to the user's location and offers the can.

This project will test your understanding of ROS 2, robot kinematics, simulation, navigation, perception, and basic manipulation.

## 2. System Architecture

The project will be built entirely within a simulated environment (Gazebo or Isaac Sim) and will use a ROS 2 software stack. The architecture is composed of several key modules:

![System Architecture Placeholder](https://via.placeholder.com/800x400.png?text=System+Architecture+Diagram)
*A diagram showing how the Navigation, Perception, Manipulation, and Behavior Control modules interact via ROS 2 topics.*

- **Robot Model (URDF):** You will use a pre-existing, complex humanoid URDF (e.g., a model similar to the TALOS robot) which will be provided. You will need to understand its link and joint structure.
- **Simulation Environment:** A simulated house world will be provided for Gazebo/Isaac Sim, containing rooms, furniture, and objects.
- **ROS 2:** The central framework connecting all software modules.
- **Navigation Stack (Nav2):** The standard ROS 2 navigation stack will be used for mapping, localization (AMCL), and path planning.
- **Perception Module:** A custom module you will build to detect and identify objects.
- **Manipulation Module:** A module that uses MoveIt 2 to plan and execute arm movements for grasping.
- **Behavior Control (BT):** A Behavior Tree will be used to orchestrate the high-level logic of the task (e.g., `Navigate` -> `Scan` -> `Pick` -> `Return`).

## 3. Phase 1: Simulation Setup & Navigation

**Goal:** Get the humanoid robot to successfully navigate the simulated house.

### Tasks:
1.  **Launch the Simulation:** Create a ROS 2 launch file that starts the simulator (Gazebo or Isaac Sim), loads the provided house environment, and spawns the humanoid robot model.
2.  **Integrate Navigation Stack:** Configure and launch the Nav2 stack. This involves:
    -   Setting up the costmap parameters (global and local).
    -   Configuring the AMCL (Adaptive Monte Carlo Localization) node.
    -   Choosing and configuring a path planner (e.g., Smac Planner) and a local controller (e.g., DWB controller).
3.  **Mapping:** Drive the robot around the environment (manually, using a joystick or keyboard controller) to build a map using the SLAM Toolbox. Save the map.
4.  **Localization & Path Planning:**
    -   Load the previously saved map.
    -   Use the tools in RViz2 to provide an initial pose estimate for the robot.
    -   Give the robot a navigation goal through RViz2 and verify that it can successfully plan and follow a path to the destination without collisions.

**Deliverable:** A video demonstrating the robot successfully navigating from a starting point to a goal location in the simulated house.

## 4. Phase 2: Perception for Object Detection

**Goal:** Enable the robot to "see" and identify the target object.

### Tasks:
1.  **Object Detection Node:** Create a new ROS 2 Python node called `object_detector`.
2.  **Image Processing:** This node will subscribe to the robot's camera feed (`/camera/image_raw`). You will use the OpenCV library to perform color-based segmentation.
    -   Convert the incoming image from BGR to HSV color space.
    -   Define a color range for the target object (e.g., "red").
    -   Create a binary mask that isolates pixels within this red range.
    -   Find contours in the mask to identify the object's shape and location.
3.  **Publish Detections:** When an object is detected, publish its location. You can create a custom message type, or simply publish a `geometry_msgs/PoseStamped` message containing the 2D coordinates (x, y in the image frame) or a rough 3D position of the object.
4.  **Visualization:** Use RViz markers to visualize the detected object's position in the 3D world, confirming your perception pipeline is working.

**Deliverable:** A video showing the robot's camera feed and, in RViz, a marker appearing over the "red can" when it is in the robot's field of view.

## 5. Phase 3: Manipulation with MoveIt 2

**Goal:** Make the robot pick up the detected object.

### Tasks:
1.  **MoveIt 2 Setup:** Configure the MoveIt 2 Setup Assistant for the provided humanoid robot model. This involves:
    -   Generating the Semantic Robot Description File (.srdf), which defines planning groups (e.g., "left_arm", "right_arm", "gripper").
    -   Configuring the kinematics solver (e.g., KDL).
    -   Generating the necessary configuration and launch files.
2.  **Grasping Node:** Create a ROS 2 node that acts as a simple manipulation server.
3.  **Plan and Execute:**
    -   This node will receive the 3D pose of the target object from your perception node.
    -   You will program a sequence of arm movements using the MoveIt 2 C++ or Python API:
        1.  Move the arm to a "pre-grasp" position above the object.
        2.  Move the end-effector (gripper) down to the object.
        3.  "Close" the gripper (in simulation, this might involve attaching the object to the gripper link).
        4.  Lift the arm to a "stow" or "carry" position.
4.  **Integration:** Trigger this grasping sequence when your `object_detector` node has a confirmed lock on the target.

**Deliverable:** A video of the simulated robot arm successfully moving to the detected object's location and performing a grasping motion.

## 6. Phase 4: Behavior Control & Final Integration

**Goal:** Combine all modules into a single, cohesive application using Behavior Trees.

### Tasks:
1.  **Behavior Tree Design:** Design a Behavior Tree (`.xml` file) that defines the high-level logic of the "fetch" task. It will look something like this:

    ```
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <NavigateToPose pose="{kitchen_table}"/>
          <FindObject object_id="red_can" object_pose="{target_pose}"/>
          <PickObject object_pose="{target_pose}"/>
          <NavigateToPose pose="{user_location}"/>
          <PlaceObject/>
        </Sequence>
      </BehaviorTree>
    </root>
    ```

2.  **Create Custom BT Nodes:** You will need to write the C++ or Python code for the custom "leaf nodes" in your tree:
    -   `NavigateToPose`: An action client that sends a goal to the Nav2 stack.
    -   `FindObject`: A node that monitors your perception topic until the target is found.
    -   `PickObject`: An action client that calls your manipulation server.
    -   `PlaceObject`: A simple node that moves the arm to a "release" position.
3.  **Launch and Test:** Create a final launch file that starts all components: the simulator, Nav2, MoveIt 2, your custom perception and manipulation nodes, and the BehaviorTree.Engine.

**Final Deliverable:** A single, continuous video showing the full scenario: the user gives a command (can be a simple ROS 2 topic publish to trigger the BT), and the robot autonomously navigates, finds, picks up, and returns the object.

## 7. Evaluation Criteria

- **Functionality (60%):** Does the final integrated system successfully complete the full task? Partial credit will be given for completing individual phases.
- **Code Quality (20%):** Is the code well-structured, commented, and easy to understand? Does it follow ROS 2 best practices?
- **Documentation (10%):** A brief `README.md` explaining how to launch and run your project.
- **Demonstration (10%):** The quality and clarity of your video deliverables.
