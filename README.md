# ME:5195 — URDF and ROS2 Forward Kinematics Tutorial


## Learning Objectives

By the end of this tutorial, you should be able to:

- Construct a notional robot by specifying links, joints, and connectivity
- Recognize the data flow from joint values to visualization in RViz
- Manually run ROS2 nodes that together realize forward kinematics
- Verify that a robot description is correctly loaded and animated

---

## Step 1: Ensure Correct Software Environment and Get Template Code

Sample code can be accessed on the course GitHub repository:  
https://github.com/UI-HandBuiltRobot/URDF-Tutorial-Files

Files are available for MATLAB and Python.

### Python Virtual Environment Setup

For this lab, you will create a local Python virtual environment (no administrator privileges required) and install the required Python packages. A virtual environment acts like a “sandbox” where you can install libraries without any risk of conflicts with other environments.

From your Linux computer:

1. Create a new folder to act as your project directory.

```bash
mkdir ~/urdf_lab
cd ~/urdf_lab
```

2. Create a virtual environment in your project directory:

```bash
python3 -m venv urdf_env #Create a virtual environment for this activity
```

3. Activate the virtual environment:

```bash
source urdf_env/bin/activate #Activate new virtual environment
```

4. Upgrade `pip` and install the required packages:

```bash
python -m pip install --upgrade pip 
python -m pip install "pyglet<2" yourdfpy trimesh numpy catkin_pkg 
```

**Important:** If you activate the virtual environment before running `colcon build`, ROS will use this Python environment during the build. Make sure all required packages are installed before building your workspace.

---

## Steps 2 - 4: Create the Robot Model

For this step, you may use MATLAB or Python to build the URDF file. Clone/download the course GitHub repository and extract the version of the URDF builder that you want to use.

The MATLAB variant uses the robotics toolbox. The Python3 variant uses `yourdfpy`. Both offer an object-oriented approach to defining and assembling robot geometries.

### VS Code Startup for Python Users

To use the Python scripts, unzip the repo and move the Python code into your `~/urdf_lab/` project directory. Then navigate to that directory and launch Visual Studio Code

```bash
cd ~/Downloads                      #Navigate to the downloads directory
unzip URDF-Tutorial-Files-main.zip  #Unzip the archive
mv URDF-Tutorial-Files-main/Python\ Scripts/ ~/urdf_lab  #Move files to project folder
cd ~/urdf_lab #Navigate back to project folder
code . #Launch VS Code in project folder
```

Visual studio code may prompt you to install the python extension. Click 'Yes'.

You may now edit and run python scripts from the VS Code IDE.

---

The URDF builder files are separated into three scripts that you should run in sequence.

1. Step 2 defines the robot's geometry (links and joints)
2. Step 3 defines joint limits and adds very basic visual geometry to the robot (boxes to represent links)
3. Step 4 exports the robot object to a Universal Robot Description File (URDF).

### Step 2: Create the rigid body tree

The code provided will define and export a robot with:

- One physical link
- Two 1-DoF revolute joints
- One end-effector link

All joints are specified to produce rotation about the Z axis, so motions are constrained to the X-Y plane. *However, the robot is a fully three-dimensional model. You are not constrained to Z rotations only.*

Note that the code includes templates for adding additional joints and links as desired. If you wish to add additional segments, note that you need to modify where the end-effector is attached.

### Step 3: Add Limits and Visual Geometry

Joint limits are added to bound motion and provide meaningful control ranges later in ROS2.

Simple visual geometry (box primitives) is attached to each link so that the robot will be visible when rendered in RViz.

This step also allows you to specify a test configuration vector containing the angles of all joints. Your robot will be displayed in a rendered window in the specified configuration.

### Step 4: Export the Robot to URDF

This script will prompt you for a location to save the URDF file. Save it in an easy-to remember location. If working in windows, you will need to transfer the file to the Linux machine.

---

## Step 5: ROS2 Workspace Setup and Visualization (Linux)

*Note: These steps must be completed in a Linux environment with ROS2 installed.*

### 5.1 Setup your ROS2 workspace

Navigate to the project folder that you created earlier and create a ROS2 workspace with a source folder:

```bash
cd ~/urdf_lab # Navigate to project folder
mkdir -p urdf_ws/src #Create directory urdf_ws containing a directory called src
cd urdf_ws/src # Go to source directory
```

*This workspace isolates your ROS2 files from the other files you use in this project (e.g. the URDF authoring codes and your Python 3 virtual environment. You may continue to put any other files you want in your project directory, but only ROS2 files should go in this subdirectory.*

### 5.2 Create a Description Package

A description-only package is created to store the robot model. This will have no actual code - only your URDF:

```bash
ros2 pkg create planarbot_description --build-type ament_cmake 
```

Create a URDF directory inside of the package folder:

```bash
mkdir -p planarbot_description/urdf
```

### 5.3 Copy the URDF(s)

Copy the exported URDF into the package urdf folder. You can copy more than one URDF file. You may use the GUI file browser or the command line:

```bash
mv ~/.../YOUR_URDF_FILE.URDF \ 
   ~/urdf_lab/urdf_ws/src/planarbot_description/urdf/ #Substitute name of your URDF from step 4
```

Rebuild and source the workspace:

```bash
cd ~/urdf_lab/urdf_ws #Navigate to ROS2 workspace root
colcon build #Build all packages in workspace
```

**NOTE:** Only run `colcon build` from the *workspace* root folder (`urdf_ws`), never from the project folder (`urdf_lab`).

---

## Step 6: Install the ME:5195 GUI package

ROS2 *usually* comes pre-installed with a handy package that allows users to publish joint angles to a topic, but our distribution is missing that package!

A custom package called `hbr_joint_state_publisher` has been written that offers the same basic functionality. Access via GitHub:  
https://github.com/UI-HandBuiltRobot/hbr_joint_state_publisher

Download the repo and copy the `hbr_joint_state_publisher` directory into your workspace `src` folder.

Now rebuild and source the workspace:

```bash
cd ~/urdf_lab/urdf_ws
colcon build
```

---

## 7: Run Your Nodes!

Three terminals are used. In each terminal source your ROS2 workspace:

```bash
source ~/urdf_lab/urdf_ws/install/setup.bash
```

### Terminal A -- robot_state_publisher:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat ~/urdf_lab/urdf_ws/src/planarbot_description/urdf/YOUR_URDF_FILENAME.urdf)"
```

You should get a message of `[INFO] [#] [robot_state_publisher]: Robot initialized`. Don't worry if you also get a warning message -- this is normal.

This node computes forward kinematics and publishes the transform tree on the topic `/TF`. It also publishes the rest of the URDF on a topic called `/robot_description` that other nodes can read.

### Terminal B -- hbr_joint_state_publisher_gui:

```bash
ros2 run hbr_joint_state_publisher hbr_joint_state_publisher_gui
```

You should get a window with one slider for each of the joints that you specified in the URDF.

When you move the sliders, this GUI publishes joint angles on `/joint_states`.

### Terminal C -- RViz:

```bash
rviz2
```

A 3D rendering window (RViz) should appear. RViz subscribes to the `/TF` topic and visualizes the robot.

### Configure RViz

Your robot won't immediately appear in RViz until you take the following steps:

1. Add a **RobotModel** display: Add → RobotModel
2. Set **RobotModel → Description Source** = `Topic`
3. Set **RobotModel → Description Topic** = `/robot_description`
4. Set **Global Options → Fixed Frame** to the robot base frame (e.g., `base`)

When joint sliders are moved, the robot should animate in real time.

---

## Diagnostic Commands

If the robot does not appear or move:

### Check joint states:
```bash
ros2 topic echo /joint_states
```

### Check TF:
```bash
ros2 topic echo /tf --once
```

### Check robot description parameter:
```bash
ros2 param list /robot_state_publisher
```

---

## Step 8 (Final Step): Inspecting Joint and Link Positions Using TF

In ROS 2, the position and orientation of each robot joint and link are represented as part of the **TF transform tree**. This transform tree is continuously published by the `robot_state_publisher` node based on the current joint angles.

To inspect the position of a particular joint or link in real time, you can use the command-line tool `tf2_echo`.

### Basic Usage

```bash
ros2 run tf2_ros tf2_echo <reference_frame> <target_frame>
```

This prints the pose of `<target_frame>` expressed in the coordinate system of `<reference_frame>`.

### Example

If your robot has:
- a base frame called `base`, and
- a link called `link2`,

you can view the position of `link2` relative to the base by running:

```bash
ros2 run tf2_ros tf2_echo base link2
```

The output will look similar to:

```text
Translation: [0.432, 0.187, 0.000]
Rotation: in Quaternion [0.000, 0.000, 0.259, 0.966]
```

The **Translation** vector corresponds to the (X, Y, Z) position of the target frame expressed in the reference frame.

### Important Note on Naming

In most URDF models, TF frames are named after **links**, not joints.  
If a joint connects `link1` to `link2`, the position of that joint is typically obtained by querying the transform of the `link2` frame.

### Listing Available Frames

```bash
ros2 topic echo /tf --once
```

This allows you to determine the correct frame names to use with `tf2_echo`.
