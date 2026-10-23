"""Frontier detection on an occupancy grid -- pure numpy, no ROS, so it is unit-testable.

A *frontier* cell is a known-free cell next to an unknown cell: the edge of what the
LIDAR has seen so far. Driving to frontiers until none are left maps the whole office.
"""
from dataclasses import dataclass
import math

import numpy as np

UNKNOWN = -1


@dataclass
class Frontier:
    x: float          # goal point in the map frame (a free frontier cell near the centroid)
    y: float
    size: int         # number of cells
    length_m: float   # approximate frontier length in metres


def _shift_any(mask, radius):
    """Dilate a boolean mask by `radius` cells (square kernel) using array shifts."""
    out = mask.copy()
    h, w = mask.shape
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if dr == 0 and dc == 0:
                continue
            src = mask[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
            out[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)] |= src
    return out


def frontier_mask(grid, free_threshold=25, occupied_threshold=65, obstacle_clearance_cells=3):
    """Boolean mask of frontier cells.

    grid: 2-D int array (rows = y, cols = x) with -1 unknown, 0..100 occupancy.
    Frontier cells closer than `obstacle_clearance_cells` to an obstacle are dropped,
    because Nav2 cannot reach a goal inside the inflated obstacle anyway.
    """
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < free_threshold)
    unknown = grid == UNKNOWN
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    mask = free & near_unknown
    if obstacle_clearance_cells > 0:
        occupied = grid >= occupied_threshold
        mask &= ~_shift_any(occupied, obstacle_clearance_cells)
    return mask


def cluster_cells(mask):
    """Group frontier cells into 8-connected clusters. Returns a list of (N, 2) [row, col] arrays.

    Cells are listed in depth-first order, which walks *along* a thin frontier, so
    consecutive slices of a cluster are spatially contiguous (used for splitting).
    """
    visited = np.zeros_like(mask, dtype=bool)
    h, w = mask.shape
    clusters = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if visited[r0, c0]:
            continue
        visited[r0, c0] = True
        stack, cells = [(r0, c0)], []
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not visited[rr, cc]:
                        visited[rr, cc] = True
                        stack.append((rr, cc))
        clusters.append(np.array(cells))
    return clusters


def split_cluster(cells, max_cells):
    """Cut a long frontier into contiguous pieces of at most `max_cells` cells.

    A frontier that wraps around the robot (typical right after start-up) has its
    centroid on top of the robot; splitting gives goals spread along the frontier instead.
    """
    if max_cells <= 0 or len(cells) <= max_cells:
        return [cells]
    pieces = [cells[i:i + max_cells] for i in range(0, len(cells), max_cells)]
    if len(pieces) > 1 and len(pieces[-1]) < max_cells // 2:
        tail = pieces.pop()
        pieces[-1] = np.vstack([pieces[-1], tail])
    return pieces


def find_frontiers(grid, resolution, origin_x, origin_y, min_size=8, max_cluster_m=2.0,
                   **mask_kwargs):
    """Return Frontier objects (map-frame goals) for every cluster with >= min_size cells.

    Clusters longer than `max_cluster_m` are split into several frontiers.
    """
    frontiers = []
    max_cells = int(max_cluster_m / resolution) if max_cluster_m else 0
    pieces = []
    for cluster in cluster_cells(frontier_mask(grid, **mask_kwargs)):
        if len(cluster) >= min_size:
            pieces.extend(split_cluster(cluster, max_cells))
    for cells in pieces:
        centroid = cells.mean(axis=0)
        # The centroid of a curved frontier can lie in unknown space -- use the
        # frontier cell nearest to it, which is guaranteed free.
        r, c = cells[np.argmin(((cells - centroid) ** 2).sum(axis=1))]
        frontiers.append(Frontier(
            x=origin_x + (c + 0.5) * resolution,
            y=origin_y + (r + 0.5) * resolution,
            size=len(cells),
            length_m=len(cells) * resolution))
    return frontiers


def choose_frontier(frontiers, robot_x, robot_y, blacklist=(), blacklist_radius=0.4,
                    min_distance=0.35, size_weight=0.5):
    """Pick the next goal: nearby and large frontiers win; blacklisted spots are skipped.

    cost = distance_to_robot - size_weight * frontier_length
    """
    best, best_cost = None, math.inf
    for f in frontiers:
        d = math.hypot(f.x - robot_x, f.y - robot_y)
        if d < min_distance:
            continue
        if any(math.hypot(f.x - bx, f.y - by) < blacklist_radius for bx, by in blacklist):
            continue
        cost = d - size_weight * f.length_m
        if cost < best_cost:
            best, best_cost = f, cost
    return best


def explored_area_m2(grid, resolution, free_threshold=25):
    """Area of known free space -- a simple 'how much of the office is mapped' metric."""
    grid = np.asarray(grid)
    return float(((grid >= 0) & (grid < free_threshold)).sum()) * resolution ** 2
