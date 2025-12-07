# Weeks 3-5: ROS 2 Fundamentals - Creating a URDF Robot Description

This document provides a comprehensive, in-depth guide to creating a Unified Robot Description Format (URDF) file from scratch. URDF is an essential XML format used in ROS 2 to describe all elements of a robot model. By the end of this tutorial, you will have a complete URDF for a simple wheeled robot and a deep understanding of each component.

## 1. Introduction to URDF

### What is URDF?
The Unified Robot Description Format (URDF) is an XML specification used in ROS to describe the physical structure of a robot. It defines the robot's links (its physical parts), its joints (which connect the links), its visual appearance, and its collision properties. Tools like RViz (the ROS 2 Visualizer) and Gazebo (a physics simulator) parse this file to display and simulate the robot.

### Why is it Important?
- **Standardization:** It provides a common format for describing robot kinematics and dynamics.
- **Visualization:** Tools like RViz use the URDF to render a 3D model of the robot.
- **Simulation:** Physics simulators like Gazebo use it to model the robot's behavior in a virtual world.
- **Kinematics & Dynamics:** ROS libraries for solving forward and inverse kinematics rely on the URDF to understand the robot's structure.

### Core Components of a URDF
A URDF file is built around two fundamental components:
- **`<link>`:** A physical part of the robot. It has inertial, visual, and collision properties.
- **`<joint>`:** Connects two links together and defines how they can move relative to each other.

---

## 2. Our Project: A Simple Two-Wheeled Robot

We will build a URDF for a simple robot named "articubot" (a fictional articulated robot). It will have:
- A main chassis.
- Two wheels that can rotate.
- A caster wheel for balance.
- A "laser" sensor on top.

This will require the following links and joints:
- **Links:** `base_link` (chassis), `left_wheel_link`, `right_wheel_link`, `caster_wheel_link`, `laser_scanner_link`.
- **Joints:** `left_wheel_joint`, `right_wheel_joint`, `caster_wheel_joint`, `laser_scanner_joint`.

---

## 3. Setting Up the URDF File

First, create a new file named `articubot.urdf` in your ROS 2 package's `description` folder.

All URDFs start with the `<robot>` tag, which is the root element.

```xml
<?xml version="1.0"?>
<robot name="articubot">

    <!-- All links and joints will go here -->

</robot>
```
The `name` attribute of the `<robot>` tag is the name of our robot.

---

## 4. Defining Links

A link is a rigid body with inertial, visual, and collision properties. Let's start with the robot's chassis, which we'll call `base_link`.

### The `base_link` (Chassis)

The `base_link` is the central part of our robot. All other components will be connected to it, directly or indirectly.

```xml
<link name="base_link">
    <visual>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <geometry>
            <box size="0.4 0.2 0.1"/>
        </geometry>
        <material name="blue">
            <color rgba="0.0 0.0 0.8 1.0"/>
        </material>
    </visual>
    <collision>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <geometry>
            <box size="0.4 0.2 0.1"/>
        </geometry>
    </collision>
    <inertial>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <mass value="5.0"/>
        <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.1"/>
    </inertial>
</link>
```

Let's break down the tags:

- **`<visual>`:** This defines how the link looks in RViz.
    - **`<origin>`:** Specifies the pose (position `xyz` and orientation `rpy` - roll, pitch, yaw) of the visual element relative to the link's origin. We've shifted it up slightly.
    - **`<geometry>`:** Defines the shape. We're using a `<box>` with dimensions `length width height`. Other options include `<cylinder>`, `<sphere>`, and `<mesh>` (for custom 3D models).
    - **`<material>`:** Sets the color. We've defined a blue color using RGBA values.

- **`<collision>`:** This defines the physical bounds of the link for physics simulation. It's crucial for Gazebo. For simple shapes, it's often identical to the `<visual>` geometry. For complex meshes, you would use a simplified shape for the collision model to save computational resources.

- **`<inertial>`:** This defines the dynamic properties of the link: its mass and rotational inertia.
    - **`<mass>`:** The mass of the link in kilograms.
    - **`<inertia>`:** The 3x3 rotational inertia matrix. This describes how the link resists angular acceleration. For simple shapes, there are formulas to calculate this. For now, these are placeholder values.

### The Wheel Links

Now, let's define the two main wheels. They will be identical except for their names.

```xml
<link name="left_wheel_link">
    <visual>
        <origin rpy="1.5707 0 0"/>
        <geometry>
            <cylinder radius="0.05" length="0.04"/>
        </geometry>
        <material name="black">
            <color rgba="0.0 0.0 0.0 1.0"/>
        </material>
    </visual>
    <collision>
        <origin rpy="1.5707 0 0"/>
        <geometry>
            <cylinder radius="0.05" length="0.04"/>
        </geometry>
    </collision>
    <inertial>
        <origin rpy="1.5707 0 0"/>
        <mass value="0.5"/>
        <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
    </inertial>
</link>

<link name="right_wheel_link">
    <visual>
        <origin rpy="1.5707 0 0"/>
        <geometry>
            <cylinder radius="0.05" length="0.04"/>
        </geometry>
        <material name="black"/>
    </visual>
    <collision>
        <origin rpy="1.5707 0 0"/>
        <geometry>
            <cylinder radius="0.05" length="0.04"/>
        </geometry>
    </collision>
    <inertial>
        <origin rpy="1.5707 0 0"/>
        <mass value="0.5"/>
        <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
    </inertial>
</link>
```
Notice the `rpy="1.5707 0 0"` in the `<origin>` tag. This rotates the cylinder by 90 degrees around the X-axis, so it stands upright like a wheel.

---

## 5. Defining Joints

Joints connect links and define their motion. A joint connects a **parent** link to a **child** link.

### The Wheel Joints

Let's connect the wheels to the chassis. These joints will be of type `continuous` because a wheel can spin indefinitely.

```xml
<joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.12 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.1"/>
</joint>

<joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0 -0.12 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.1"/>
</joint>
```

Let's break down the joint tags:

- **`name` and `type`:** Each joint has a unique name and a type. Common types are:
    - `revolute`: Rotates around an axis with limits.
    - `continuous`: Rotates around an axis without limits (like a wheel).
    - `prismatic`: Slides along an axis with limits.
    - `fixed`: Rigidly connects two links.
- **`<parent>` and `<child>`:** Defines the kinematic chain. The `child` link's pose is defined relative to the `parent` link's pose.
- **`<origin>`:** Defines the pose of the child link's origin relative to the parent link's origin. For the `left_wheel_joint`, we place it at `y=0.12`, which is to the left of the chassis's center.
- **`<axis>`:** The axis of rotation for `revolute` and `continuous` joints, or the axis of translation for `prismatic` joints. Here, `0 1 0` means the wheels will rotate around the Y-axis.
- **`<dynamics>`:** (Optional) Defines physical properties like `damping` and `friction` for simulation.

### Creating the Rest of the Robot

Now we apply the same principles to create the caster wheel and the laser scanner.

#### Caster Wheel (Fixed Joint)

For simplicity, our caster wheel won't rotate; it will be fixed to the chassis.

```xml
<link name="caster_wheel_link">
    <visual>
        <geometry>
            <sphere radius="0.03"/>
        </geometry>
        <material name="grey">
            <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <sphere radius="0.03"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
</link>

<joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel_link"/>
    <origin xyz="-0.15 0 -0.02"/>
</joint>
```
Since the `type` is `fixed`, we don't need an `<axis>` or `<dynamics>` tag.

#### Laser Scanner (Fixed Joint)

Finally, let's add a "sensor" to the top.

```xml
<link name="laser_scanner_link">
    <visual>
        <geometry>
            <box size="0.05 0.05 0.1"/>
        </geometry>
        <material name="red">
            <color rgba="0.8 0.0 0.0 1.0"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <box size="0.05 0.05 0.1"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
</link>

<joint name="laser_scanner_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_scanner_link"/>
    <origin xyz="0.15 0 0.15"/>
</joint>
```

---

## 6. The Complete URDF File

Here is the complete `articubot.urdf` file. You can copy and paste this into your file.

```xml
<?xml version="1.0"?>
<robot name="articubot">

    <!-- Materials -->
    <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
    <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
    </material>
    <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
    <material name="red">
        <color rgba="0.8 0.0 0.0 1.0"/>
    </material>

    <!-- Base Link (Chassis) -->
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <box size="0.4 0.2 0.1"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <box size="0.4 0.2 0.1"/>
            </geometry>
        </collision>
        <inertial>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <mass value="5.0"/>
            <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.1"/>
        </inertial>
    </link>

    <!-- Wheel Links -->
    <link name="left_wheel_link">
        <visual>
            <origin rpy="1.5707 0 0"/>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
            <material name="black"/>
        </visual>
        <collision>
            <origin rpy="1.5707 0 0"/>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
        </collision>
        <inertial>
            <origin rpy="1.5707 0 0"/>
            <mass value="0.5"/>
            <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
        </inertial>
    </link>

    <link name="right_wheel_link">
        <visual>
            <origin rpy="1.5707 0 0"/>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
            <material name="black"/>
        </visual>
        <collision>
            <origin rpy="1.5707 0 0"/>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
        </collision>
        <inertial>
            <origin rpy="1.5707 0 0"/>
            <mass value="0.5"/>
            <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
        </inertial>
    </link>

    <!-- Caster Wheel Link -->
    <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="0.03"/>
            </geometry>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.03"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
        </inertial>
    </link>

    <!-- Laser Scanner Link -->
    <link name="laser_scanner_link">
        <visual>
            <geometry>
                <box size="0.05 0.05 0.1"/>
            </geometry>
            <material name="red"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.05 0.05 0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
        </inertial>
    </link>

    <!-- Joints -->
    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <origin xyz="0 0.12 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <dynamics damping="0.01" friction="0.1"/>
    </joint>

    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <origin xyz="0 -0.12 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <dynamics damping="0.01" friction="0.1"/>
    </joint>

    <joint name="caster_wheel_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_wheel_link"/>
        <origin xyz="-0.15 0 -0.02"/>
    </joint>

    <joint name="laser_scanner_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_scanner_link"/>
        <origin xyz="0.15 0 0.15"/>
    </joint>

</robot>
```
*Note: Defining reusable `<material>` tags at the top level is good practice, though some older parsers might not support it. For maximum compatibility, you can define the `<material>` tag within each `<visual>` block.*

---

## 7. Next Steps: Visualizing the URDF

With this file created, the next step is to visualize it in RViz. This requires creating a ROS 2 launch file that starts the `robot_state_publisher` and `joint_state_publisher` nodes, which read the URDF and publish the robot's transformations.

This concludes our deep dive into creating a URDF file. You now have a solid foundation for describing any robot in ROS 2.
