from collections import deque
import math


CARDINAL_MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def to_node_name(gx, gy):
    return f'{int(gx)}.{int(gy)}'


def from_node_name(name):
    x_str, y_str = str(name).split('.')
    return int(x_str), int(y_str)


def edge_cost(parent_name, child_name):
    parent = from_node_name(parent_name)
    child = from_node_name(child_name)
    return float(math.hypot(child[0] - parent[0], child[1] - parent[1]))


def line_cells(start_cell, end_cell):
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
    for gx, gy in line_cells(start_cell, end_cell):
        if not grid_map.in_bounds(gx, gy) or not grid_map.is_free(gx, gy):
            return False
    return True


def nearest_visible_node(target_cell, node_cells, grid_map):
    visible = [cell for cell in node_cells if line_is_free(grid_map, target_cell, cell)]
    if not visible:
        return None
    return min(visible, key=lambda cell: math.hypot(cell[0] - target_cell[0], cell[1] - target_cell[1]))


def sampling_anchor_cell(origin, resolution):
    return int(round((-origin / resolution) - 0.5))


def sampled_free_cells(grid_map, graph_step):
    anchor_x = sampling_anchor_cell(grid_map.origin_x, grid_map.resolution)
    anchor_y = sampling_anchor_cell(grid_map.origin_y, grid_map.resolution)

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
    required_cells = set(required_cells or [])
    sampled_cells = sampled_free_cells(grid_map, max(1, int(graph_step)))
    node_cells = set(sampled_cells)
    node_cells.update(required_cells)

    edges = []
    seen_pairs = set()

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

    for required_cell in required_cells:
        if required_cell in sampled_cells:
            continue
        anchor_cell = nearest_visible_node(required_cell, sampled_cells, grid_map)
        if anchor_cell is not None:
            connect_visible_neighbors(edges, seen_pairs, required_cell, anchor_cell)

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
    if start_name is None or goal_name is None:
        return []
    if start_name == goal_name:
        return []

    discovered_edges = []
    visited = {start_name}
    queue = deque([start_name])

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
    if start_name is None or goal_name is None:
        return []
    if start_name == goal_name:
        return [start_name]
    if not discovered_edges:
        return []

    parent_lookup = {edge['child']: edge['parent'] for edge in discovered_edges}
    if goal_name not in parent_lookup:
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
        x0, y0 = path_cells[index - 1]
        x1, y1 = path_cells[index]
        dx = 0 if x1 == x0 else (1 if x1 > x0 else -1)
        dy = 0 if y1 == y0 else (1 if y1 > y0 else -1)

        current_x = x0
        current_y = y0
        while (current_x, current_y) != (x1, y1):
            current_x += dx
            current_y += dy
            expanded.append((current_x, current_y))

    return expanded
