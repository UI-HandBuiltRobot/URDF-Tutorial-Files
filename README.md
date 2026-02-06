# ME:5195 URDF and ROS Tutorial - Building a Planar Robot

## Learning Objectives

By the end of Step 5, students will be able to:

* Construct a notional robot by specifying links, joints, and connectivity
* Recognize the data flow from joint values to visualization in RViz
* Manually run ROS 2 nodes that together realize forward kinematics
* Verify that a robot description is correctly loaded and animated

---

## Step 1: Set Up Software Environment and Get Template Code

Sample code can be accessed on the course GitHub repository:
[https://github.com/UI-HandBuiltRobot/URDF-Tutorial-Files](https://github.com/UI-HandBuiltRobot/URDF-Tutorial-Files)

Files are available for both **MATLAB** and **Python**.

---

## Python Virtual Environment Setup

For this lab, you will create a local Python virtual environment (no administrator privileges required) and install the required Python packages.

### A. Create a project directory

```bash
mkdir ~/urdf_lab
cd ~/urdf_lab
```

### B. Create a virtual environment

```bash
python3 -m venv urdf_env
```

### C. Activate the virtual environment

```bash
source urdf_env/bin/activate
```

### D. Install required Python packages

```bash
python -m pip install --upgrade pip
python -m pip install pyglet<2 yourdfpy trimesh numpy catkin_pkg
```

> **Important**
> If you activate the virtual environment before running `colcon build`, ROS will use this Python environment during the build.
> Make sure all required packages are installed **before** building your workspace.

---

## Steps 2–4: Create the Robot Model

For these steps, you may use **MATLAB** or **Python** to build the URDF file. Clone or download the course repository and extract the version of the URDF builder you want to use.

* MATLAB uses the Robotics Toolbox
* Python uses `yourdfpy`

Both approaches follow an object-oriented workflow.

The URDF builder is divided into three scripts:

1. **Step 2** – Define robot geometry (links and joints)
2. **Step 3** – Add joint limits and basic visual geometry
3. **Step 4** – Export the robot to a URDF file

---

### Step 2: Create the Rigid Body Tree

The provided code defines a robot with:

* One physical link
* Two 1-DoF revolute joints
* One end-effector link

All joints rotate about the **Z-axis**, so motion is constrained to the **X–Y plane**.
However, the robot model is fully three-dimensional — you are not limited to Z-axis rotations.

Templates are included for adding additional links and joints. If you extend the robot, ensure the end-effector attachment is updated accordingly.

---

### Step 3: Add Limits and Visual Geometry

Joint limits are added to bound motion and provide meaningful control ranges in ROS 2.

Simple visual geometry (box primitives) is attached to each link so the robot can be rendered in RViz.

You can also specify a test configuration vector. The robot will be displayed in a rendered window at that configuration.

---

### Step 4: Export the Robot to URDF

This script prompts you for a location to save the URDF file.

Save it somewhere easy to remember.
If working in **Windows**, you will need to transfer the file to a Linux machine.

---

## Step 5: ROS 2 Workspace Setup and Visualization (Linux)

> **Note**
> These steps must be completed in a Linux environment with ROS 2 installed.

---

### Setup Your ROS 2 Workspace

Navigate to your project directory:

```bash
cd ~/urdf_lab
```

Create a ROS 2 workspace and move into its source directory:

```bash
mkdir -p urdf_ws/src
cd urdf_ws/src
```

This isolates ROS 2 files from your URDF authoring code and Python virtual environment.

---

### Create a Description Package

Create a description-only package to store the URDF:

```bash
ros2 pkg create planarbot_description --build-type ament_cmake
```

Create a directory for URDF files:

```bash
mkdir -p planarbot_description/urdf
```

---

### Copy the URDF File(s)

Copy your exported URDF into the package:

```bash
mv ~/.../<YOUR_URDF_FILE.urdf> \
   ~/urdf_lab/urdf_ws/src/planarbot_description/urdf/
```

Build the workspace:

```bash
cd ~/urdf_lab/urdf_ws
colcon build
```

> **Important**
> Always run `colcon build` from the workspace root (`urdf_ws`),
> **never** from the project directory (`urdf_lab`).

---

## Step 6: Install the ME:5195 Joint State GUI Package

ROS 2 normally includes a joint-state GUI, but this distribution does not.

A custom replacement package is provided:

* **hbr_joint_state_publisher**
  [https://github.com/UI-HandBuiltRobot/hbr_joint_state_publisher](https://github.com/UI-HandBuiltRobot/hbr_joint_state_publisher)

Clone the repository and copy it into your workspace `src` directory.

Then rebuild:

```bash
cd ~/urdf_lab/urdf_ws
colcon build
```

---

## Step 7: Run the ROS Nodes

You will use **three terminals**.
In each terminal, source your workspace:

```bash
source ~/urdf_lab/urdf_ws/install/setup.bash
```

---

### Terminal A — `robot_state_publisher`

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat ~/urdf_lab/urdf_ws/src/planarbot_description/urdf/YOUR_URDF_FILENAME.urdf)"
```

You should see a message similar to:

```
[INFO] [...] [robot_state_publisher]: Robot initialized
```

Warnings are normal.

This node:

* Computes forward kinematics
* Publishes the transform tree on `/tf` and `/tf_static`
* Republishes the URDF on `/robot_description`

---

### Terminal B — Joint State Publisher GUI

```bash
ros2 run hbr_joint_state_publisher hbr_joint_state_publisher_gui
```

A GUI window will appear with one slider per joint.

Moving the sliders publishes joint angles on `/joint_states`.

---

### Terminal C — RViz

```bash
rviz2
```

RViz visualizes the robot using TF.

---

### Configure RViz

Your robot will not appear until RViz is configured:

1. Add a **RobotModel** display
2. Set **RobotModel → Description Source** = `Topic`
3. Set **RobotModel → Description Topic** = `/robot_description`
4. Set **Global Options → Fixed Frame** to the robot base frame
   (e.g., `base` or `base_link`)

Moving the sliders should now animate the robot in real time.

---

## Diagnostic Commands

If the robot does not appear or move:

### Check joint states

```bash
ros2 topic echo /joint_states
```

### Check TF

```bash
ros2 topic echo /tf --once
```

### Check robot description parameter

```bash
ros2 param list /robot_state_publisher
```

---

## Step 8: Inspecting Joint and Link Positions Using TF

In ROS 2, joint and link positions are represented in the **TF transform tree**, which is continuously published by `robot_state_publisher`.

You can inspect positions in real time using `tf2_echo`.

---

### Basic Usage

```bash
ros2 run tf2_ros tf2_echo <reference_frame> <target_frame>
```

This prints the pose of `<target_frame>` expressed in `<reference_frame>`.

---

### Example

If your robot has:

* a base frame called `base`
* a link called `link2`

Run:

```bash
ros2 run tf2_ros tf2_echo base link2
```

Example output:

```text
Translation: [0.432, 0.187, 0.000]
Rotation: in Quaternion [0.000, 0.000, 0.259, 0.966]
```

The **Translation** vector gives the `(X, Y, Z)` position of the link relative to the base.

---

### Important Note on Naming

In most URDF models, TF frames are named after **links**, not joints.

If a joint connects `link1` to `link2`, the joint’s spatial location is typically obtained by querying the transform of `link2`.

---

### Listing Available Frames

```bash
ros2 topic echo /tf --once
```

This helps identify valid frame names for use with `tf2_echo`.


