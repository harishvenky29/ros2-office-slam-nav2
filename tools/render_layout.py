#!/usr/bin/env python3
"""Render a top-down view of worlds/small_office.sdf and check the patrol route.

Draws every collision shape (light) and the slice the robot actually interacts
with -- anything between the floor and 0.20 m, which includes the LIDAR plane at
~0.17 m (dark). Prints each waypoint's clearance to the nearest obstacle.

    python3 tools/render_layout.py            # writes docs/office_layout.png
Needs: matplotlib, pyyaml (dev-only; not used at runtime).
"""
import math
import os
import xml.etree.ElementTree as ET

import yaml

ROBOT_TOP = 0.20  # TurtleBot3 Burger is ~0.19 m tall; LIDAR plane ~0.17 m


def parse_pose(text):
    v = [float(t) for t in (text or "0 0 0 0 0 0").split()]
    return v[0], v[1], v[2], v[5]


def load_shapes(sdf_path):
    shapes = []  # (kind, x, y, z, yaw, dims, name)
    for mdl in ET.parse(sdf_path).getroot().iter("model"):
        for link in mdl.findall("link"):
            x, y, z, yaw = parse_pose(link.findtext("pose"))
            for col in link.findall("collision"):
                g = col.find("geometry")
                if g.find("box") is not None:
                    sx, sy, sz = (float(t) for t in g.find("box/size").text.split())
                    shapes.append(("box", x, y, z, yaw, (sx, sy, sz), mdl.get("name")))
                elif g.find("cylinder") is not None:
                    r = float(g.findtext("cylinder/radius"))
                    h = float(g.findtext("cylinder/length"))
                    shapes.append(("cyl", x, y, z, 0.0, (r, h), mdl.get("name")))
    return shapes


def low(shape):
    kind, _, _, z, _, dims, _ = shape
    h = dims[2] if kind == "box" else dims[1]
    return z - h / 2 < ROBOT_TOP


def dist_to(shape, px, py):
    kind, x, y, _, yaw, dims, _ = shape
    if kind == "cyl":
        return max(0.0, math.hypot(px - x, py - y) - dims[0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * (px - x) - s * (py - y), s * (px - x) + c * (py - y)
    dx, dy = max(abs(lx) - dims[0] / 2, 0), max(abs(ly) - dims[1] / 2, 0)
    return math.hypot(dx, dy)


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from matplotlib.transforms import Affine2D

    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    shapes = load_shapes(os.path.join(root, "worlds", "small_office.sdf"))
    with open(os.path.join(root, "config", "office_waypoints.yaml")) as f:
        route = yaml.safe_load(f)

    fig, ax = plt.subplots(figsize=(12, 8.4))
    for sh in shapes:
        kind, x, y, _, yaw, dims, name = sh
        if name == "floor":
            continue
        dark = low(sh)
        style = dict(fc="#3a3a3a" if dark else "#d9cbb5", ec="#222" if dark else "#a89880",
                     lw=0.6, zorder=3 if dark else 2)
        if kind == "box":
            p = Rectangle((x - dims[0] / 2, y - dims[1] / 2), dims[0], dims[1], **style)
            p.set_transform(Affine2D().rotate_around(x, y, yaw) + ax.transData)
        else:
            p = Circle((x, y), dims[0], **style)
        ax.add_patch(p)

    pts = [(route["initial_pose"]["x"], route["initial_pose"]["y"])] + \
          [(w["x"], w["y"]) for w in route["waypoints"]]
    ax.plot(*zip(*pts), "--", color="#e8590c", lw=1.5, zorder=4)
    print("waypoint clearances (robot radius 0.105 m):")
    for w in route["waypoints"]:
        clr = min(dist_to(s, w["x"], w["y"]) for s in shapes if low(s) and s[6] != "floor")
        ax.plot(w["x"], w["y"], "o", color="#e8590c", ms=9, zorder=5)
        ax.annotate(w["name"], (w["x"], w["y"]), xytext=(6, 6), textcoords="offset points",
                    fontsize=9, color="#b03a00", zorder=6)
        print(f"  {w['name']:<15} ({w['x']:5.1f},{w['y']:5.1f})  clearance {clr:.2f} m"
              + ("" if clr > 0.30 else "   <-- TIGHT"))
    ax.plot(0, 0, "^", color="#1971c2", ms=12, zorder=6)
    ax.annotate("spawn (0,0)", (0, 0), xytext=(6, -14), textcoords="offset points",
                fontsize=9, color="#1971c2")
    for label, (lx, ly) in {"MEETING ROOM": (-4, 3.75), "STORAGE": (0, 3.2),
                            "MANAGER OFFICE": (4.0, 3.75), "OPEN-PLAN WORKSPACE": (-2.5, 0.4),
                            "KITCHEN": (4.75, 0.5)}.items():
        ax.text(lx, ly, label, ha="center", fontsize=10, color="#555", weight="bold", zorder=7)

    ax.set_xlim(-6.4, 6.4)
    ax.set_ylim(-4.4, 4.4)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Small office world: dark = what the robot and its LIDAR interact with "
                 "(< 0.2 m), light = above LIDAR height")
    os.makedirs(os.path.join(root, "docs"), exist_ok=True)
    out = os.path.join(root, "docs", "office_layout.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("wrote", os.path.normpath(out))


if __name__ == "__main__":
    main()
