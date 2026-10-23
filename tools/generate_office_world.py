#!/usr/bin/env python3
"""Generate worlds/small_office.sdf -- a 12 m x 8 m office for Gazebo Harmonic.

Edit the layout below and re-run:
    python3 tools/generate_office_world.py

World frame == SLAM map frame because the robot spawns at (0, 0, yaw 0).

Layout (metres, top-down, +x east, +y north):

    y=4 +-----------------+--------+-----------------+
        |  MEETING ROOM   |STORAGE |  MANAGER OFFICE |
        |    [table]      |[shelf] |       [desk]    |
    y=1 +----   ----------+  open  +---------   -----+
        |                                  |         |
        |   [desks]       [desks]   (0,0)  | KITCHEN |
        |   OPEN-PLAN WORKSPACE      start |  [tbl]  |
        |                       [sofa]     |[counter]|
   y=-4 +----------------------------------+---------+
       x=-6                               x=3.5     x=6
"""
import math
import os

# ---------------------------------------------------------------- colours
WALL = (0.85, 0.85, 0.82)
GLASS = (0.55, 0.70, 0.80)
WOOD = (0.55, 0.38, 0.22)
METAL = (0.30, 0.30, 0.33)
GREY = (0.45, 0.45, 0.48)
FABRIC = (0.20, 0.30, 0.55)
GREEN = (0.15, 0.50, 0.20)
POT = (0.60, 0.35, 0.25)
WHITE = (0.92, 0.92, 0.92)

WALL_T = 0.10   # wall thickness
WALL_H = 2.00   # wall height


def material(rgb):
    r, g, b = rgb
    return (f"<material><ambient>{r} {g} {b} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def box_link(name, x, y, z, sx, sy, sz, rgb, yaw=0.0):
    geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return (f'<link name="{name}"><pose>{x} {y} {z} 0 0 {yaw}</pose>'
            f'<collision name="c">{geom}</collision>'
            f'<visual name="v">{geom}{material(rgb)}</visual></link>')


def cyl_link(name, x, y, z, r, h, rgb):
    geom = f"<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>"
    return (f'<link name="{name}"><pose>{x} {y} {z} 0 0 0</pose>'
            f'<collision name="c">{geom}</collision>'
            f'<visual name="v">{geom}{material(rgb)}</visual></link>')


def sphere_visual_link(name, x, y, z, r, rgb):
    geom = f"<geometry><sphere><radius>{r}</radius></sphere></geometry>"
    return (f'<link name="{name}"><pose>{x} {y} {z} 0 0 0</pose>'
            f'<visual name="v">{geom}{material(rgb)}</visual></link>')


def model(name, links):
    return f'<model name="{name}"><static>true</static>{"".join(links)}</model>'


# ------------------------------------------------------------ primitives
def wall(name, x1, y1, x2, y2, rgb=WALL):
    """Straight wall between two points (axis-aligned or not)."""
    length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    return model(name, [box_link("wall", cx, cy, WALL_H / 2,
                                 length, WALL_T, WALL_H, rgb, yaw)])


def desk(name, x, y, w=1.4, d=0.7, pedestal_side=1):
    """Desk with 4 legs + a drawer pedestal (the LIDAR sees legs + pedestal)."""
    top_z, leg = 0.74, 0.04
    links = [box_link("top", x, y, top_z, w, d, 0.03, WOOD)]
    for i, (dx, dy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        links.append(box_link(f"leg{i}", x + dx * (w / 2 - 0.05), y + dy * (d / 2 - 0.05),
                              (top_z - 0.015) / 2, leg, leg, top_z - 0.015, METAL))
    px = x + pedestal_side * (w / 2 - 0.30)
    links.append(box_link("pedestal", px, y, 0.30, 0.40, d - 0.15, 0.60, GREY))
    return model(name, links)


def chair(name, x, y, back_dir=0.0):
    """Office chair -- solid seat column so a 2D LIDAR at ~0.17 m sees it."""
    links = [cyl_link("seat", x, y, 0.225, 0.24, 0.45, FABRIC)]
    bx = x + 0.22 * math.cos(back_dir)
    by = y + 0.22 * math.sin(back_dir)
    links.append(box_link("back", bx, by, 0.70, 0.06, 0.42, 0.50, FABRIC, back_dir))
    return model(name, links)


def plant(name, x, y):
    return model(name, [cyl_link("pot", x, y, 0.20, 0.20, 0.40, POT),
                        sphere_visual_link("leaves", x, y, 0.75, 0.35, GREEN)])


def block(name, x, y, sx, sy, sz, rgb):
    return model(name, [box_link("body", x, y, sz / 2, sx, sy, sz, rgb)])


def meeting_table(name, x, y, w=2.0, d=1.0):
    """Conference table on two panel legs -- shows up as two short lines in the scan."""
    links = [box_link("top", x, y, 0.74, w, d, 0.04, WOOD)]
    for i, dx in enumerate((-1, 1)):
        links.append(box_link(f"panel{i}", x + dx * (w / 2 - 0.15), y, 0.36,
                              0.05, d - 0.2, 0.72, WOOD))
    return model(name, links)


def round_table(name, x, y, r=0.45):
    return model(name, [cyl_link("base", x, y, 0.10, 0.25, 0.20, METAL),
                        cyl_link("pole", x, y, 0.45, 0.05, 0.50, METAL),
                        cyl_link("top", x, y, 0.74, r, 0.03, WHITE)])


def sofa(name, x, y, w=1.8, d=0.7):
    return model(name, [box_link("seat", x, y, 0.22, w, d, 0.44, FABRIC),
                        box_link("back", x, y - d / 2 + 0.08, 0.55, w, 0.16, 0.40, FABRIC)])


# ------------------------------------------------------------------ layout
def build_layout():
    m = []
    X0, X1, Y0, Y1 = -6.0, 6.0, -4.0, 4.0

    # Outer shell
    m += [wall("wall_north", X0 - 0.05, Y1, X1 + 0.05, Y1),
          wall("wall_south", X0 - 0.05, Y0, X1 + 0.05, Y0),
          wall("wall_west", X0, Y0, X0, Y1),
          wall("wall_east", X1, Y0, X1, Y1)]

    # North rooms: wall at y=1 with 1 m doorways
    m += [wall("wall_meeting_s1", -6.0, 1.0, -3.5, 1.0),      # door x in [-3.5, -2.5]
          wall("wall_meeting_s2", -2.5, 1.0, -2.0, 1.0),
          wall("wall_meeting_e", -2.0, 1.0, -2.0, 4.0, GLASS),
          wall("wall_office_w", 2.0, 1.0, 2.0, 4.0, GLASS),
          wall("wall_office_s1", 2.0, 1.0, 2.5, 1.0),         # door x in [2.5, 3.5]
          wall("wall_office_s2", 3.5, 1.0, 6.0, 1.0)]

    # Kitchen partition (open to the north between y=-1.5 and y=1)
    m += [wall("wall_kitchen", 3.5, -4.0, 3.5, -1.5)]

    # Meeting room
    m += [meeting_table("meeting_table", -4.0, 2.5)]
    for i, (cx, cy, back) in enumerate([(-4.6, 1.75, -math.pi / 2), (-3.4, 1.75, -math.pi / 2),
                                        (-4.6, 3.25, math.pi / 2), (-3.4, 3.25, math.pi / 2)]):
        m.append(chair(f"meeting_chair_{i}", cx, cy, back))
    m.append(plant("plant_meeting", -5.6, 3.6))

    # Storage / print area (open to the south)
    m += [block("shelving", 0.0, 3.72, 3.0, 0.40, 1.8, WOOD),
          block("printer", 1.55, 2.0, 0.50, 0.60, 1.0, WHITE)]

    # Manager office
    m += [desk("office_desk", 4.5, 3.2, pedestal_side=1),
          chair("office_chair", 4.5, 2.55, -math.pi / 2),
          block("filing_cabinet", 5.65, 1.5, 0.50, 0.60, 1.0, GREY),
          plant("plant_office", 2.4, 3.6)]

    # Open-plan workspace: two clusters of back-to-back desks
    for c, cx in enumerate((-4.0, -1.0)):
        m.append(desk(f"desk_{c}_n", cx, -1.15, pedestal_side=1))
        m.append(desk(f"desk_{c}_s", cx, -1.85, pedestal_side=-1))
        for k, dx in enumerate((-0.35, 0.35)):
            m.append(chair(f"chair_{c}_n{k}", cx + dx, -0.50, math.pi / 2))
            m.append(chair(f"chair_{c}_s{k}", cx + dx, -2.50, -math.pi / 2))
    m += [block("filing_west_1", -5.70, 0.20, 0.50, 0.50, 1.0, GREY),
          block("filing_west_2", -5.70, -0.40, 0.50, 0.50, 1.0, GREY),
          sofa("sofa", 1.4, -3.55),
          plant("plant_sw", -5.6, -3.6),
          plant("plant_se", 3.1, -3.6)]

    # Kitchen
    m += [block("kitchen_counter", 4.975, -3.65, 1.95, 0.60, 0.90, WHITE),
          block("fridge", 5.60, -0.90, 0.70, 0.70, 1.80, METAL),
          round_table("kitchen_table", 4.7, -2.4)]
    for i, (cx, cy, back) in enumerate([(4.0, -2.4, math.pi), (5.4, -2.4, 0.0)]):
        m.append(chair(f"kitchen_chair_{i}", cx, cy, back))
    return m


WORLD_HEADER = """<?xml version="1.0"?>
<!-- AUTO-GENERATED by tools/generate_office_world.py. Edit the generator, not this file. -->
<sdf version="1.8">
  <world name="small_office">
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.80 0.85 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="floor">
      <static>true</static>
      <link name="floor">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.62 0.60 0.56 1</ambient><diffuse>0.62 0.60 0.56 1</diffuse></material></visual>
      </link>
    </model>
"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "worlds", "small_office.sdf")
    models = build_layout()
    with open(out, "w") as f:
        f.write(WORLD_HEADER)
        for mdl in models:
            f.write("    " + mdl + "\n")
        f.write("  </world>\n</sdf>\n")
    print(f"wrote {os.path.normpath(out)} ({len(models)} models)")


if __name__ == "__main__":
    main()
