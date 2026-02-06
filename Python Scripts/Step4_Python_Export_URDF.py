#!/usr/bin/env python3
"""Step4_Python_Export_URDF_yourdfpy.py

PURPOSE
  Export the yourdfpy URDF model to a .urdf XML file.

  This mirrors the MATLAB script "Step4_Export_URDF.m".

NOTES
  - yourdfpy provides convenience methods to write URDF XML.
    The API includes URDF.write_xml_string() and URDF.write_xml_file(...)
    (see docs).

  - For maximum robustness in a teaching environment, we use:
      1) try URDF.write_xml_file(path)
      2) otherwise write URDF.write_xml_string() to a file ourselves

  - After export, you can copy the generated .urdf into a ROS2 package (e.g.,
    <your_pkg>/urdf/) and visualize with RViz or Gazebo.
"""

from __future__ import annotations

from pathlib import Path

from yourdfpy.urdf import URDF

import tkinter as tk
from tkinter import filedialog


def pick_urdf_save_path(default_name="robot.urdf"):
    """Open a save-file dialog and return the chosen path (or None)."""
    root = tk.Tk()
    root.withdraw()  # hide the empty Tk window

    path = filedialog.asksaveasfilename(
        title="Save URDF file",
        defaultextension=".urdf",
        initialfile=default_name,
        filetypes=[("URDF files", "*.urdf"), ("All files", "*.*")]
    )

    root.destroy()
    return path if path else None


def export_urdf(urdf: URDF, output_path: str | Path) -> Path:
    """Write URDF to disk and return the resolved output path."""
    out = Path(output_path).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    # Preferred: use yourdfpy's file writer if available.
    if hasattr(urdf, "write_xml_file"):
        urdf.write_xml_file(str(out))
    else:
        # Fallback: write the XML string ourselves.
        xml = urdf.write_xml_string()
        out.write_text(xml, encoding="utf-8")

    return out


if __name__ == "__main__":
    # Import Step 2 + Step 3 so this is runnable end-to-end.
    from Step2_Create_Tree import build_planar_robot
    from Step3_Add_Visuals_and_Limits import add_limits_and_visuals

    urdf, params = build_planar_robot()
    add_limits_and_visuals(urdf, params)

    # Export next to this script by default.
    try:
        out_path = export_urdf(urdf, pick_urdf_save_path("planar_robot.urdf"))
    except Exception:
        out_path = export_urdf(urdf, "planar_robot.urdf")

    print("Wrote URDF to:")
    print(out_path)