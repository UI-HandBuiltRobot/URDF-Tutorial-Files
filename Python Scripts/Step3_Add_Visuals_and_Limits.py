#!/usr/bin/env python3
"""Step3_Python_Add_Visuals_and_Limits_yourdfpy.py

PURPOSE
  This file mirrors the MATLAB script "Step3_Add_Robot_Visuals_and_Limits.m".

  Starting from a kinematic URDF model (created in Step 2), we:
    1) Add joint limits to revolute joints
    2) Add simple *visual* geometry so the robot looks like physical links
    3) (Optional) Visualize the robot at zero and at a sample configuration
    4) Students can extend this to add visuals for additional links

WHY DO THIS AS A SEPARATE STEP?
  In teaching, it helps to separate:
    - the *kinematic structure* (tree of frames)
    - the *appearance* and *limits* used by RViz and joint_state_publisher_gui

yourdfpy NOTES
  - yourdfpy supports object-based visuals: Visual(Geometry(Box(...)))
  - Materials use Material(color=Color(rgba=[r,g,b,a]))
  - Joint limits use Limit(lower=..., upper=..., effort=..., velocity=...)

REFERENCES
  - Material signature: Material(name, color: Color, texture)  (yourdfpy API)
  - Visual signature: Visual(name, origin: ndarray, geometry, material) (yourdfpy API)
  - Link signature: Link(name, inertial=None, visuals=[...], collisions=[...]) (yourdfpy API)
"""

from __future__ import annotations

import numpy as np

from yourdfpy.urdf import (
    URDF,
    Link,
    Joint,
    Visual,
    Geometry,
    Box,
    Material,
    Color,
    Limit,
)


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

def T_from_xyz(x: float, y: float, z: float) -> np.ndarray:
    """4x4 translation transform (same helper as Step 2)."""
    T = np.eye(4, dtype=float)
    T[0:3, 3] = [x, y, z]
    return T


def find_link(urdf: URDF, name: str) -> Link:
    """Convenience helper: fetch a Link by name."""
    return urdf.link_map[name]


def find_joint(urdf: URDF, name: str) -> Joint:
    """Convenience helper: fetch a Joint by name."""
    return urdf.joint_map[name]

def visualize_robot(robot: URDF) -> None:
    """Helper to visualize the robot after modifications."""
    cfg = robot.cfg.copy() # current configuration to be recalled after reloading
    # 1) Write a temporary URDF to disk so yourdfpy can reload it
    tmp_urdf_path = "tmp_step3_preview.urdf"
    robot.write_xml_file(tmp_urdf_path)   # writes the URDF XML

    # 2) Reload using flags required to build a visualizable scene
    urdf = URDF.load(tmp_urdf_path, build_scene_graph=True, load_meshes=True)

    # 3) Set the desired configuration
    urdf.update_cfg(cfg)

    # 4) Show it (should no longer be an empty scene)
    print("Opening visualization window... (close the window to continue)")
    urdf.show()  # This opens a visualizer window that stays open

#-----------------------------------------------------------------------------
# Step 3 core functionality
# -----------------------------------------------------------------------------

def add_limits_and_visuals(
    urdf: URDF,
    params: dict[str, float]
) -> None:
    """Modify an existing URDF model in-place.

    Parameters
      urdf: URDF
        The URDF model from Step 2.
      params: dict[str, float]
        Dictionary of parameters, including link lengths and other dimensions.

    This function updates:
      - each joint's limit
      - each link's visuals
      - robot.materials (so materials are defined globally)
    
    Students: Add parameters for additional link lengths (L2, L3, etc.) as needed.
    """
    link_thickness = 0.04 # set a default link thickness
    # ---------------------------------------------------------------------
    # 1) Joint limits
    # ---------------------------------------------------------------------
    # In URDF, revolute joints typically require limits.
    # These limits feed tools like joint_state_publisher_gui sliders.
    #

    # Create limits for all joints 
  
    limit_all = Limit(lower=-np.pi, upper=np.pi, effort=1.0, velocity=1.0)

    #####################################################
    ################# Students: Add more joints to this list as you create them
    #####################################################
    for jname in ["joint1", "joint_ee"]:  # Add: "joint2", "joint3", etc.
        j = find_joint(urdf, jname)
        j.limit = limit_all

    # ---------------------------------------------------------------------
    # 2) Materials
    # ---------------------------------------------------------------------
    # In yourdfpy, a Material references a Color object.
    gray = Material(
        name="gray",
        color=Color(rgba=[0.7, 0.7, 0.7, 1.0]),
    )

    # Register material on the Robot so it can be referenced by visuals.
    # (This mirrors how <material> is usually declared in URDF XML.)
    if urdf.robot.materials is None:
        urdf.robot.materials = []
    urdf.robot.materials.append(gray)

    # ---------------------------------------------------------------------
    # 3) Visual geometry
    # ---------------------------------------------------------------------
    # We model each link as a simple box.
    # Convention: links extend along +X from each joint frame.
    # To center a box along a link of length L, we offset the visual origin by +L/2 in X.

    # Students: Add more links here as you create them
    L1 =params.get("L1")
   
    link1 = find_link(urdf, "link1")
    link1.visuals.append(
        Visual(
            name="link1_visual",
            origin=T_from_xyz(L1 / 2.0, 0.0, 0.0),
            geometry=Geometry(box=Box(size=[L1, link_thickness, link_thickness])),
            material=gray,
        )
    )

    # End-effector visual (small cube) 
    ee_size = params.get("ee_size", 0.05)  # Default to 0.05 if not specified
    ee_link = find_link(urdf, "ee_link")
    ee_link.visuals.append(
        Visual(
            name="ee_visual",
            origin=T_from_xyz(0.0, 0.0, 0.0),
            geometry=Geometry(box=Box(size=[ee_size, ee_size, ee_size])),
            material=gray,
        )
    )

    ########################################################
    # Students: Add visuals for additional links following the pattern above
    ########################################################
    # Example template for link2:
    # L2 = link_lengths.get("L2")
    # link2 = find_link(urdf, "link2")
    # link2.visuals.append(
    #     Visual(
    #         name="link2_visual",
    #         origin=T_from_xyz(L2 / 2.0, 0.0, 0.0),
    #         geometry=Geometry(box=Box(size=[L2, link_thickness, link_thickness])),
    #         material=gray,
    #     )
    # )


# -----------------------------------------------------------------------------
# Demonstration
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    # We import the Step 2 builder so students can run Step 3 standalone.
    from Step2_Create_Tree import build_planar_robot

    urdf, params = build_planar_robot()

    # Students: Add more link length parameters as you extend the robot
    add_limits_and_visuals(
        urdf,
        params
    )

    # Visualize zero configuration
    print("Showing robot at zero configuration...")
    # Students: Add more joints to this configuration as you create them
    urdf.update_cfg({"joint1": 0.0, "joint_ee": 0.0})  # Add: "joint2": 0.0, etc.
    visualize_robot(urdf)

    T_base_to_ee = urdf.get_transform("ee_link", "base_link")

    print("\nEnd-effector transform (base_link -> ee_link):")
    print(T_base_to_ee)

    print("\nEnd-effector position [x y z] (m):")
    print(T_base_to_ee[0:3, 3])

    # Visualize sample configuration
    print("Showing robot at a sample configuration...")
    # Students: Add more joints to this configuration as you create them
    q = {"joint1": 0.5, "joint_ee": -0.5}  # Add: "joint2": -0.6, "joint3": 0.5, etc.
    urdf.update_cfg(q)
    visualize_robot(urdf)

    T_base_to_ee = urdf.get_transform("ee_link", "base_link")

    print("\nEnd-effector transform (base_link -> ee_link):")
    print(T_base_to_ee)

    print("\nEnd-effector position [x y z] (m):")
    print(T_base_to_ee[0:3, 3])
