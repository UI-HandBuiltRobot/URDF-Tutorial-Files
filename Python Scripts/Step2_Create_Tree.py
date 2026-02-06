#!/usr/bin/env python3
"""Step2_Python_Create_Tree_yourdfpy.py

PURPOSE
  This file is the *Python / yourdfpy* analog of the MATLAB rigidBodyTree script
  "Step2_Matlab_Create_Tree.m".

  It builds the *kinematic structure* of a simple planar robot arm:
    - 1 physical link (link1)
    - 1 revolute joint (joint1)
    - Students can add more links and joints as needed

  At this stage we are intentionally focusing on *kinematics*:
    - link/joint naming
    - joint placement (fixed transforms)
    - joint axes

  Visual geometry and joint limits are added later in:
    Step3_Python_Add_Visuals_and_Limits_yourdfpy.py

CONVENTIONS (same as the MATLAB tutorial)
  - Planar arm in the XY plane
  - All joint axes are +Z
  - Link lengths extend along +X
  - Units: meters, radians

REFERENCES
  - yourdfpy URDF class provides FK utilities (scene graph) and viewer: URDF.get_transform(), URDF.show().
    See the yourdfpy API docs for Joint signature (origin, axis) and URDF.get_transform/show.
"""

from __future__ import annotations

import numpy as np

# yourdfpy is a pure-Python URDF parser/authoring library.
# It provides an object model (Robot, Link, Joint, etc.) and a URDF wrapper
# that can build a scene graph for forward kinematics.
from yourdfpy.urdf import URDF, Robot, Link, Joint


# -----------------------------------------------------------------------------
# Helper functions (No need to mess with these right now)
# -----------------------------------------------------------------------------

def T_from_xyz(x: float, y: float, z: float) -> np.ndarray:
    """Return a 4x4 homogeneous transform for a pure translation.

    yourdfpy uses 4x4 numpy arrays for origin transforms (Joint.origin, Visual.origin).
    """
    T = np.eye(4, dtype=float)
    T[0:3, 3] = [x, y, z]
    return T


def make_link(name: str) -> Link:
    """Create an empty URDF Link.

    In yourdfpy, Link has defaults for inertial/visuals/collisions, so we can keep
    construction simple.
    """
    return Link(name=name)


# -----------------------------------------------------------------------------
# Step 2: Build the kinematic chain (links + joints)
# -----------------------------------------------------------------------------

def build_planar_robot(
    L1: float = 0.30,
    ee_size: float = 0.05,
    robot_name: str = "simple_planar_robot",
) -> tuple[URDF, dict[str, float]]:
    """Build and return (urdf_model, params).

    Returns
      urdf_model:
        A yourdfpy.urdf.URDF instance wrapping a Robot.
        The URDF wrapper can perform FK if the scene graph is built.

      params:
        A dictionary of key geometry parameters (useful for Step 3).
    """

    # 1) Define links
    # Students: Add more links below as needed
    base_link = make_link("base_link")
    link1 = make_link("link1")
    ee_link = make_link("ee_link")

    # 2) Define joints
    # Students: You can change the axis to any direction
    # Examples: axis_z for rotation about Z, axis_x for rotation about X, etc.
    axis_z = np.array([0.0, 0.0, 1.0], dtype=float)
    # axis_x = np.array([1.0, 0.0, 0.0], dtype=float)  # Uncomment if needed
    # axis_y = np.array([0.0, 1.0, 0.0], dtype=float)  # Uncomment if needed

    # Joint1: base_link -> link1, located at the base origin
    j1 = Joint(
        name="joint1",
        type="revolute",
        parent="base_link",
        child="link1",
        origin=T_from_xyz(0.0, 0.0, 0.0),
        axis=axis_z,
        # NOTE: joint limits are added in Step 3
    )

    # Joint_ee: link1 -> ee_link, located at end of link1
    joint_ee = Joint(
        name="joint_ee",
        type="revolute",
        parent="link1",
        child="ee_link",
        origin=T_from_xyz(L1, 0.0, 0.0),  # Position at end of link1
        axis=axis_z,
    )

    ###################################################
    # Students: Add more joints below following the pattern above
    ###################################################
    # Example template for adding joint2:
    # j2 = Joint(
    #     name="joint2",
    #     type="revolute",
    #     parent="link1",      # Parent link
    #     child="link2",       # Child link
    #     origin=T_from_xyz(L1, 0.0, 0.0),  # Position relative to parent
    #     axis=axis_z,         # Rotation axis
    # )

    # 3) Assemble the Robot object
    # Students: Add any new links and joints to these lists
    robot = Robot(
        name=robot_name,
        links=[base_link, link1, ee_link],  # Add more links here: link2, link3, etc.
        joints=[j1, joint_ee],              # Add more joints here: j2, j3, etc.
        materials=[],
    )

    # 4) Wrap it as a URDF model.
    #    build_scene_graph=True enables FK queries like get_transform.
    urdf = URDF(robot=robot, build_scene_graph=True, load_meshes=False)

    # Students: Add more link lengths to params as needed
    params = {"L1": L1, "ee_size": ee_size}  # Add more: "L2": L2, "L3": L3, etc.
    return urdf, params


# -----------------------------------------------------------------------------
# Demonstration / sanity checks (similar to MATLAB "show" and FK test)
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    urdf, params = build_planar_robot()

    print("Built robot:", urdf.robot.name)
    print("Links:", [L.name for L in urdf.robot.links])
    print("Joints:", [J.name for J in urdf.robot.joints])
    print("Parameters:", params)

    # FK sanity check: compute transform base_link -> ee_link at a sample configuration
    # yourdfpy expects a dict mapping joint names -> joint positions (radians for revolute).
    # Students: Add more joints to this configuration as you add them to the robot
    cfg = {
        "joint1": 0.3,
        "joint_ee": 0.0,
        # "joint2": -0.6,  # Add more joints here
        # "joint3": 0.5,
    }

    # Update the internal configuration (performs FK over the scene graph)
    urdf.update_cfg(cfg)

    # Now we can query transforms between frames.
    T_base_to_ee = urdf.get_transform("ee_link", "base_link")

    print("\nEnd-effector transform (base_link -> ee_link):")
    print(T_base_to_ee)

    print("\nEnd-effector position [x y z] (m):")
    print(T_base_to_ee[0:3, 3])

    # Visualization at this step will likely show only coordinate frames (no visuals yet).
    # We add visuals in Step 3.
    # urdf.show()
