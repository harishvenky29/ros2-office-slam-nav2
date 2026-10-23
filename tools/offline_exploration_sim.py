#!/usr/bin/env python3
"""Offline sanity check for the frontier explorer -- no ROS or Gazebo needed.

Rasterises worlds/small_office.sdf at the robot's height, fakes the Burger's LIDAR
(360 beams, 3.5 m range) with ray casting, and runs the *same* frontier code the ROS
node uses (office_nav/frontiers.py) in a loop: pick frontier -> plan a path -> drive
it while scanning -> repeat. Reports how much of the reachable office got mapped.

    python3 tools/offline_exploration_sim.py          # also writes docs/offline_exploration.png
"""
from collections import deque
import math
import os
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from office_nav.frontiers import choose_frontier, find_frontiers  # noqa: E402
from render_layout import dist_to, load_shapes, low  # noqa: E402

RES, X0, Y0, W, H = 0.05, -6.5, -4.5, 260, 180   # 13 m x 9 m grid
LIDAR_RANGE, N_BEAMS, ROBOT_R = 3.5, 360, 0.105


def world_to_cell(x, y):
    return int((y - Y0) / RES), int((x - X0) / RES)


def cell_to_world(r, c):
    return X0 + (c + 0.5) * RES, Y0 + (r + 0.5) * RES


def ground_truth():
    shapes = [s for s in load_shapes(os.path.join(ROOT, "worlds", "small_office.sdf"))
              if low(s) and s[6] != "floor"]
    occ = np.zeros((H, W), bool)
    for r in range(H):
        for c in range(W):
            x, y = cell_to_world(r, c)
            occ[r, c] = any(dist_to(s, x, y) <= RES * 0.5 for s in shapes)
    return occ


MIN_PASS_THROUGH = 2   # SLAM Toolbox default: a cell is free once >= 2 beams crossed it
PASSES = None


def scan(occ, known, x, y):
    """Ray-cast one 360-beam scan. Like SLAM Toolbox, a cell becomes free only after
    MIN_PASS_THROUGH beams have passed through it; the cell a beam hits becomes occupied."""
    global PASSES
    if PASSES is None:
        PASSES = np.zeros(known.shape, int)
    steps = np.arange(0.0, LIDAR_RANGE, RES / 2)
    for a in np.linspace(0, 2 * math.pi, N_BEAMS, endpoint=False):
        rr = ((y + steps * math.sin(a) - Y0) / RES).astype(int)
        cc = ((x + steps * math.cos(a) - X0) / RES).astype(int)
        ok = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
        rr, cc = rr[ok], cc[ok]
        hits = np.nonzero(occ[rr, cc])[0]
        end = hits[0] if len(hits) else len(rr)
        cells = np.unique(np.stack([rr[:end], cc[:end]]), axis=1)
        PASSES[cells[0], cells[1]] += 1
        if len(hits):
            known[rr[end], cc[end]] = 100
    known[(PASSES >= MIN_PASS_THROUGH) & (known != 100)] = 0


def inflate(occ, cells):
    out = occ.copy()
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            if dr * dr + dc * dc <= cells * cells:
                out |= np.roll(np.roll(occ, dr, 0), dc, 1)
    return out


def bfs_path(blocked, start, goal):
    prev = {start: None}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            path = []
            while cur:
                path.append(cur)
                cur = prev[cur]
            return path[::-1]
        r, c = cur
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            n = (r + dr, c + dc)
            if 0 <= n[0] < H and 0 <= n[1] < W and not blocked[n] and n not in prev:
                prev[n] = cur
                q.append(n)
    return None


def reachable_free(blocked, start):
    seen = np.zeros_like(blocked)
    seen[start] = True
    q = deque([start])
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (r + dr, c + dc)
            if 0 <= n[0] < H and 0 <= n[1] < W and not blocked[n] and not seen[n]:
                seen[n] = True
                q.append(n)
    return seen


def main():
    occ = ground_truth()
    blocked = inflate(occ, int(math.ceil(ROBOT_R / RES)))
    known = np.full((H, W), -1, np.int8)
    x, y = 0.0, 0.0
    trail, blacklist, goals, dist = [(x, y)], [], 0, 0.0
    scan(occ, known, x, y)
    first = choose_frontier(find_frontiers(known, RES, X0, Y0), x, y)
    print(f"after first scan: {((known == 0).sum() * RES**2):.1f} m^2 free, first goal: "
          + (f"({first.x:.2f}, {first.y:.2f})" if first else "NONE  <-- explorer would stall"))
    while goals < 200:
        frontiers = find_frontiers(known, RES, X0, Y0)
        target = choose_frontier(frontiers, x, y, blacklist=blacklist)
        if target is None:
            break
        path = bfs_path(blocked, world_to_cell(x, y), world_to_cell(target.x, target.y))
        if path is None:
            blacklist.append((target.x, target.y))
            continue
        goals += 1
        for i, cell in enumerate(path):
            nx, ny = cell_to_world(*cell)
            dist += math.hypot(nx - x, ny - y)
            x, y = nx, ny
            trail.append((x, y))
            if i % 6 == 0:                       # scan every ~0.3 m of travel
                scan(occ, known, x, y)
        scan(occ, known, x, y)

    reach = reachable_free(blocked, world_to_cell(0.0, 0.0))
    # A cell counts as mapped if it is known free (LIDAR swept it).
    mapped = reach & (known == 0)
    coverage = 100.0 * mapped.sum() / reach.sum()
    print(f"goals sent: {goals}, blacklisted: {len(blacklist)}, path length: {dist:.1f} m")
    print(f"reachable floor mapped: {coverage:.1f}%  "
          f"({mapped.sum() * RES**2:.1f} of {reach.sum() * RES**2:.1f} m^2)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        img = np.full((H, W, 3), 0.55)
        img[known == 0] = (1, 1, 1)
        img[known == 100] = (0, 0, 0)
        fig, ax = plt.subplots(figsize=(10, 7))
        ax.imshow(img, origin="lower", extent=(X0, X0 + W * RES, Y0, Y0 + H * RES))
        tx, ty = zip(*trail)
        ax.plot(tx, ty, color="#e8590c", lw=1.2, label="robot path")
        ax.plot(0, 0, "^", color="#1971c2", ms=10, label="start")
        ax.set_title(f"Offline frontier exploration: {coverage:.1f}% of reachable floor mapped, "
                     f"{goals} goals, {dist:.0f} m driven")
        ax.legend(loc="upper right")
        out = os.path.join(ROOT, "docs", "offline_exploration.png")
        fig.savefig(out, dpi=100, bbox_inches="tight")
        print("wrote", os.path.normpath(out))
    except ImportError:
        pass
    return coverage


if __name__ == "__main__":
    main()
