import heapq
import math


def euclidean_heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def move_cost(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar_search(grid_map, start, goal, connectivity=8):
    if start is None or goal is None:
        return []
    if start == goal:
        return [start]

    open_heap = []
    heapq.heappush(open_heap, (euclidean_heuristic(start, goal), 0.0, start))

    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, current_cost, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in grid_map.free_neighbor_cells(current[0], current[1], connectivity=connectivity):
            if neighbor in closed:
                continue

            tentative_g = current_cost + move_cost(current, neighbor)
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + euclidean_heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, tentative_g, neighbor))

    return []


def line_is_free(grid_map, start, goal):
    x0, y0 = start
    x1, y1 = goal

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x, y = x0, y0
    while True:
        if not grid_map.is_free(x, y):
            return False
        if (x, y) == (x1, y1):
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def shortcut_smooth_path(grid_map, path_cells):
    if len(path_cells) < 3:
        return path_cells

    smoothed = [path_cells[0]]
    anchor_index = 0

    while anchor_index < len(path_cells) - 1:
        next_index = len(path_cells) - 1
        while next_index > anchor_index + 1:
            if line_is_free(grid_map, path_cells[anchor_index], path_cells[next_index]):
                break
            next_index -= 1

        smoothed.append(path_cells[next_index])
        anchor_index = next_index

    return smoothed
