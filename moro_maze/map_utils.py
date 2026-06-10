import math

import numpy as np


class GridMap:
    """Thin wrapper around a nav_msgs/OccupancyGrid.

    Stores the occupancy data as a 2D array indexed [gy, gx] (row-major, gy=0 is
    the bottom row, matching ROS map convention) and offers world<->grid
    conversions plus the free/occupied/exit queries the planner needs. Occupancy
    values follow ROS: 0..100 cost, -1 unknown; a cell counts as occupied at or
    above `occupied_threshold`.
    """

    def __init__(self, data, width, height, resolution, origin_x, origin_y, occupied_threshold=65):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.occupied_threshold = int(occupied_threshold)
        self.data = np.array(data, dtype=np.int16).reshape((self.height, self.width))
        self._occupied_world_points = None
        self._free_world_points = None

    @classmethod
    def from_msg(cls, msg, occupied_threshold=65):
        return cls(
            data=msg.data,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            occupied_threshold=occupied_threshold,
        )

    def world_to_grid(self, x, y):
        # World (metres) -> grid index: subtract the map origin and divide by the
        # cell size, flooring to the cell that contains the point.
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        # Grid index -> world (metres): inverse of world_to_grid, returning the
        # cell CENTRE (the +0.5 offset), which is what the planner publishes.
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def occupancy_at(self, gx, gy):
        if not self.in_bounds(gx, gy):
            return None
        return int(self.data[gy, gx])

    def is_unknown(self, gx, gy):
        value = self.occupancy_at(gx, gy)
        return value is None or value < 0

    def is_occupied(self, gx, gy):
        value = self.occupancy_at(gx, gy)
        return value is None or value >= self.occupied_threshold

    def is_free(self, gx, gy):
        value = self.occupancy_at(gx, gy)
        return value is not None and 0 <= value < self.occupied_threshold

    def summary(self):
        known_cells = self.data[self.data >= 0]
        free_cells = int(np.sum((self.data >= 0) & (self.data < self.occupied_threshold)))
        occupied_cells = int(np.sum(self.data >= self.occupied_threshold))
        unknown_cells = int(np.sum(self.data < 0))
        mean_known = float(np.mean(known_cells)) if known_cells.size > 0 else 0.0
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'free_cells': free_cells,
            'occupied_cells': occupied_cells,
            'unknown_cells': unknown_cells,
            'mean_known_occupancy': mean_known,
        }

    def occupied_world_points(self):
        if self._occupied_world_points is not None:
            return self._occupied_world_points

        occupied_indices = np.argwhere(self.data >= self.occupied_threshold)
        if occupied_indices.size == 0:
            self._occupied_world_points = np.empty((0, 2), dtype=np.float32)
            return self._occupied_world_points

        world_points = []
        for gy, gx in occupied_indices:
            world_points.append(self.grid_to_world(int(gx), int(gy)))

        self._occupied_world_points = np.array(world_points, dtype=np.float32)
        return self._occupied_world_points

    def free_world_points(self):
        if self._free_world_points is not None:
            return self._free_world_points

        free_indices = np.argwhere((self.data >= 0) & (self.data < self.occupied_threshold))
        if free_indices.size == 0:
            self._free_world_points = np.empty((0, 2), dtype=np.float32)
            return self._free_world_points

        world_points = []
        for gy, gx in free_indices:
            world_points.append(self.grid_to_world(int(gx), int(gy)))

        self._free_world_points = np.array(world_points, dtype=np.float32)
        return self._free_world_points

    def wall_world_points(self):
        return self.occupied_world_points()

    def knn_dataset(self):
        free_points = self.free_world_points()
        wall_points = self.wall_world_points()

        if free_points.size == 0 and wall_points.size == 0:
            return np.empty((0, 2), dtype=np.float32), np.empty((0,), dtype=np.int8)

        x = np.vstack([free_points, wall_points]).astype(np.float32)
        y = np.concatenate([
            np.zeros(len(free_points), dtype=np.int8),
            np.ones(len(wall_points), dtype=np.int8),
        ])
        return x, y

    def nearest_obstacle_distance(self, x, y):
        occupied = self.occupied_world_points()
        if occupied.size == 0:
            return float('inf')

        delta = occupied - np.array([x, y], dtype=np.float32)
        distances = np.linalg.norm(delta, axis=1)
        return float(np.min(distances))

    def nearest_free_cell(self, gx, gy, max_radius=4):
        # Snap a (possibly occupied/out-of-bounds) cell to the closest free cell by
        # searching outward in growing square rings, up to max_radius. Used to keep
        # the robot start and the exit goals on valid free cells.
        if self.in_bounds(gx, gy) and self.is_free(gx, gy):
            return gx, gy

        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx = gx + dx
                    ny = gy + dy
                    if self.in_bounds(nx, ny) and self.is_free(nx, ny):
                        return nx, ny
        return None

    def free_neighbor_cells(self, gx, gy, connectivity=8):
        if connectivity == 4:
            moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        else:
            moves = [
                (-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1),
            ]

        neighbors = []
        for dx, dy in moves:
            nx = gx + dx
            ny = gy + dy
            if self.in_bounds(nx, ny) and self.is_free(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    def detect_border_exits(self, minimum_run=2):
        # Real maze exits are openings in the maze's wall shell, NOT free runs along
        # the outer padding ring. The map has a one-cell free border all the way
        # around, so scanning that ring just returns the midpoint of every edge
        # (e.g. (13,0),(0,13),...), which are not where the maze actually opens.
        #
        # Instead, locate the wall shell via the bounding box of all occupied cells,
        # then scan each of its four sides for free runs. A free run on a wall side
        # is a genuine opening; we return the outer-border cell aligned with it, so
        # the planner can drive the robot out through that gap.
        occupied = np.argwhere(self.data >= self.occupied_threshold)
        if occupied.size == 0:
            return []

        wall_min_y, wall_min_x = (int(v) for v in occupied.min(axis=0))
        wall_max_y, wall_max_x = (int(v) for v in occupied.max(axis=0))

        exits = []

        def scan_side(cells, to_border):
            run = []
            for gx, gy in cells:
                if self.is_free(gx, gy):
                    run.append((gx, gy))
                else:
                    if len(run) >= minimum_run:
                        exits.append(to_border(run[len(run) // 2]))
                    run = []
            if len(run) >= minimum_run:
                exits.append(to_border(run[len(run) // 2]))

        # Left / right walls (vertical sides) open out the left / right outer border.
        scan_side(
            [(wall_min_x, gy) for gy in range(wall_min_y, wall_max_y + 1)],
            lambda cell: (0, cell[1]),
        )
        scan_side(
            [(wall_max_x, gy) for gy in range(wall_min_y, wall_max_y + 1)],
            lambda cell: (self.width - 1, cell[1]),
        )
        # Bottom / top walls (horizontal sides) open out the bottom / top outer border.
        scan_side(
            [(gx, wall_min_y) for gx in range(wall_min_x, wall_max_x + 1)],
            lambda cell: (cell[0], 0),
        )
        scan_side(
            [(gx, wall_max_y) for gx in range(wall_min_x, wall_max_x + 1)],
            lambda cell: (cell[0], self.height - 1),
        )

        unique = []
        seen = set()
        for cell in exits:
            if cell not in seen:
                seen.add(cell)
                unique.append(cell)
        return unique
