"""Graph-search utilities for the SOAR global planner.

Builds a visibility graph over sampled free cells of the map, runs Breadth-First
Search between two nodes, reconstructs the path, and provides the Bresenham
line-of-sight helpers used both for graph edges and for densifying the path.

A node is identified by its grid coordinates encoded as the string 'gx.gy' (the
dot is the SOAR cookbook's convention, e.g. '4.4', '10.4'); edges are dicts with
'parent', 'child' and 'cost'.
"""

from collections import deque
import math


CARDINAL_MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def to_node_name(gx, gy):
    """Encode a grid cell as the node name 'gx.gy' (cookbook convention)."""
    return f'{int(gx)}.{int(gy)}'


def from_node_name(name):
    """Decode a node name 'gx.gy' back into the (gx, gy) integer grid cell."""
    x_str, y_str = str(name).split('.')
    return int(x_str), int(y_str)


def edge_cost(parent_name, child_name):
    parent = from_node_name(parent_name)
    child = from_node_name(child_name)
    return float(math.hypot(child[0] - parent[0], child[1] - parent[1]))


def line_cells(start_cell, end_cell):
    # Bresenham's line algorithm: returns every grid cell the straight segment
    # start->end passes through, using only integer arithmetic. Used to test
    # line-of-sight (no wall between two cells) and to densify path segments.
    x0, y0 = start_cell
    x1, y1 = end_cell

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    cells = []
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if (x, y) == (x1, y1):
            return cells
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def line_is_free(grid_map, start_cell, end_cell):
    # Line of sight: True only if every cell on the Bresenham segment is in bounds
    # and free (no wall blocks the straight path between the two cells).
    for gx, gy in line_cells(start_cell, end_cell):
        if not grid_map.in_bounds(gx, gy) or not grid_map.is_free(gx, gy):
            return False
    return True


def nearest_visible_node(target_cell, node_cells, grid_map):
    # Anchor an arbitrary cell (robot start or exit) to the graph: pick the closest
    # graph node that has clear line of sight to it, so the robot can reach the
    # graph (and the goal off the graph) in a straight collision-free segment.
    visible = [cell for cell in node_cells if line_is_free(grid_map, target_cell, cell)]
    if not visible:
        return None
    return min(visible, key=lambda cell: math.hypot(cell[0] - target_cell[0], cell[1] - target_cell[1]))


def sampling_anchor_cell(origin, resolution):
    # Grid cell whose centre sits at world coordinate 0. Used so the sampling
    # lattice is phase-aligned to integer world coords: with this anchor and
    # graph_step=6 the nodes land exactly on world 0,1,2,3 (grid 4,10,16,22),
    # matching the cookbook's node positions.
    return int(round((-origin / resolution) - 0.5))


def sampled_free_cells(grid_map, graph_step):
    anchor_x = sampling_anchor_cell(grid_map.origin_x, grid_map.resolution)
    anchor_y = sampling_anchor_cell(grid_map.origin_y, grid_map.resolution)

    # Keep every free cell that lies on the lattice: its offset from the anchor is
    # a multiple of graph_step in both axes.
    sampled = set()
    for gy in range(grid_map.height):
        for gx in range(grid_map.width):
            if not grid_map.is_free(gx, gy):
                continue
            if (gx - anchor_x) % graph_step == 0 and (gy - anchor_y) % graph_step == 0:
                sampled.add((gx, gy))
    return sampled


def connect_visible_neighbors(edges, seen_pairs, node_a, node_b):
    if node_a == node_b:
        return

    name_a = to_node_name(*node_a)
    name_b = to_node_name(*node_b)
    pair_key = tuple(sorted((name_a, name_b)))
    if pair_key in seen_pairs:
        return

    seen_pairs.add(pair_key)
    edges.append({'parent': name_a, 'child': name_b, 'cost': edge_cost(name_a, name_b)})


def build_graph(grid_map, required_cells=None, graph_step=6):
    # Visibility graph: nodes are free cells sampled on a regular lattice (every
    # graph_step cells, anchored so they fall on integer world coords). Edges
    # connect consecutive nodes in the same row or column when there is clear line
    # of sight between them (no wall in between). Edge cost = Euclidean distance.
    required_cells = set(required_cells or [])
    sampled_cells = sampled_free_cells(grid_map, max(1, int(graph_step)))
    node_cells = set(sampled_cells)

    edges = []
    seen_pairs = set()

    # Group nodes by row (shared gy) and by column (shared gx) so we only test
    # line of sight between immediate neighbours along each axis.
    rows = {}
    cols = {}
    for gx, gy in node_cells:
        rows.setdefault(gy, []).append((gx, gy))
        cols.setdefault(gx, []).append((gx, gy))

    for gy, row_nodes in rows.items():
        row_nodes.sort(key=lambda cell: cell[0])
        for index in range(1, len(row_nodes)):
            left = row_nodes[index - 1]
            right = row_nodes[index]
            if line_is_free(grid_map, left, right):
                connect_visible_neighbors(edges, seen_pairs, left, right)

    for gx, col_nodes in cols.items():
        col_nodes.sort(key=lambda cell: cell[1])
        for index in range(1, len(col_nodes)):
            lower = col_nodes[index - 1]
            upper = col_nodes[index]
            if line_is_free(grid_map, lower, upper):
                connect_visible_neighbors(edges, seen_pairs, lower, upper)

    return sorted(node_cells), edges


def get_children(edges, node_name):
    children = []
    for edge in edges:
        if edge['parent'] == node_name:
            children.append(edge['child'])
        elif edge['child'] == node_name:
            children.append(edge['parent'])
    return children


def bfs_search(start_name, edges, goal_name):
    # Breadth-First Search (SOAR cookbook). Explores the graph level by level from
    # start. Returns the list of edges discovered during the search (the search
    # tree): each visited child stores the edge from the node that discovered it,
    # which reconstruct_path() later follows backwards from the goal to the start.
    if start_name is None or goal_name is None:
        return []
    if start_name == goal_name:
        return []

    discovered_edges = []
    visited = {start_name}          # nodes already enqueued (avoid revisiting)
    queue = deque([start_name])     # FIFO frontier -> breadth-first order

    while queue:
        current = queue.popleft()
        if current == goal_name:
            return discovered_edges

        for child in get_children(edges, current):
            if child in visited:
                continue
            visited.add(child)
            discovered_edges.append({'parent': current, 'child': child, 'cost': edge_cost(current, child)})
            queue.append(child)

    return []


def reconstruct_path(discovered_edges, start_name, goal_name):
    # Walk the BFS search tree backwards: from the goal, follow each node's parent
    # until reaching the start, then reverse to get start -> goal. The search tree
    # guarantees exactly one parent per discovered node, so this is unambiguous.
    if start_name is None or goal_name is None:
        return []
    if start_name == goal_name:
        return [start_name]
    if not discovered_edges:
        return []

    parent_lookup = {edge['child']: edge['parent'] for edge in discovered_edges}
    if goal_name not in parent_lookup:        # goal was never reached
        return []

    path = [goal_name]
    current = goal_name

    while current != start_name:
        current = parent_lookup.get(current)
        if current is None:
            return []
        path.append(current)

    path.reverse()
    return path


def path_name_cost(path_names):
    if len(path_names) < 2:
        return 0.0

    total = 0.0
    for index in range(1, len(path_names)):
        total += edge_cost(path_names[index - 1], path_names[index])
    return total


def path_names_to_cells(path_names):
    return [from_node_name(name) for name in path_names]


def expand_path_cells(path_cells):
    if len(path_cells) < 2:
        return path_cells

    expanded = [path_cells[0]]

    for index in range(1, len(path_cells)):
        segment_cells = line_cells(path_cells[index - 1], path_cells[index])
        expanded.extend(segment_cells[1:])

    return expanded


def expand_sparse_path(grid_map, path_cells, step_cells=2):
    # Turn the sparse graph path into something Nav2 can actually execute:
    # keep the original sparse waypoints (so the planner stays "graph-based"),
    # but seed intermediate waypoints every `step_cells` cells along each long
    # edge instead of jumping straight between distant nodes. Each intermediate
    # waypoint is validated to land on a free cell.
    if len(path_cells) < 2:
        return list(path_cells)

    step_cells = max(1, int(step_cells))
    expanded = [path_cells[0]]

    for index in range(1, len(path_cells)):
        start_cell = path_cells[index - 1]
        end_cell = path_cells[index]
        segment = line_cells(start_cell, end_cell)

        # segment[0] is the previous waypoint (already added); segment[-1] is the
        # node endpoint, which we always keep below.
        for offset in range(step_cells, len(segment) - 1, step_cells):
            candidate = segment[offset]
            if grid_map.is_free(*candidate) and candidate != expanded[-1]:
                expanded.append(candidate)

        if end_cell != expanded[-1]:
            expanded.append(end_cell)

    return expanded
