import numpy as np

from office_nav.frontiers import (choose_frontier, cluster_cells, explored_area_m2,
                                  find_frontiers, frontier_mask)


def room_half_seen():
    """10x10 room: walls on the border, left half seen (free), right half unknown."""
    g = np.full((40, 40), -1, np.int8)
    g[:, :20] = 0
    g[0, :20] = g[-1, :20] = g[:, 0] = 100
    return g


def test_frontier_is_the_seen_unseen_boundary():
    mask = frontier_mask(room_half_seen(), obstacle_clearance_cells=0)
    cols = set(np.nonzero(mask)[1])
    assert cols == {19}


def test_frontier_cells_next_to_walls_are_dropped():
    mask = frontier_mask(room_half_seen(), obstacle_clearance_cells=3)
    rows = np.nonzero(mask)[0]
    assert rows.min() >= 4 and rows.max() <= 35


def test_clusters_are_split_by_gaps():
    m = np.zeros((10, 10), bool)
    m[2, 1:4] = True
    m[7, 5:9] = True
    sizes = sorted(len(c) for c in cluster_cells(m))
    assert sizes == [3, 4]


def test_goal_is_a_free_cell_in_world_coordinates():
    g = room_half_seen()
    fr = find_frontiers(g, 0.05, -1.0, -1.0, min_size=5)
    assert len(fr) == 1
    f = fr[0]
    c = int((f.x + 1.0) / 0.05)
    r = int((f.y + 1.0) / 0.05)
    assert g[r, c] == 0


def test_fully_mapped_room_has_no_frontiers():
    g = np.zeros((30, 30), np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    assert find_frontiers(g, 0.05, 0, 0) == []
    assert explored_area_m2(g, 0.05) == 28 * 28 * 0.05 ** 2


def test_choose_skips_blacklisted_and_too_close():
    g = room_half_seen()
    fr = find_frontiers(g, 0.05, 0.0, 0.0, min_size=5)
    f = fr[0]
    assert choose_frontier(fr, f.x, f.y) is None                      # too close
    assert choose_frontier(fr, 0.2, 0.2, blacklist=[(f.x, f.y)]) is None
    assert choose_frontier(fr, 0.2, 0.2) is f


def test_split_long_frontier_into_contiguous_pieces():
    from office_nav.frontiers import split_cluster
    line = np.array([(0, c) for c in range(95)])
    pieces = split_cluster(line, 40)
    assert [len(p) for p in pieces] == [40, 55]          # short tail merged
    assert sum(len(p) for p in pieces) == 95
    assert len(split_cluster(line[:30], 40)) == 1


def test_ring_frontier_around_robot_gives_goals_away_from_robot():
    """Right after start-up the frontier is a ring around the robot: its centroid is the
    robot itself. Splitting must still yield a goal far enough away to drive to."""
    g = np.full((61, 61), -1, np.int8)
    yy, xx = np.mgrid[:61, :61]
    g[(yy - 30) ** 2 + (xx - 30) ** 2 <= 20 ** 2] = 0            # 1 m free disk
    fr = find_frontiers(g, 0.05, -1.525, -1.525)
    assert len(fr) > 1
    assert choose_frontier(fr, 0.0, 0.0, min_distance=0.35) is not None
