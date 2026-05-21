import math

import numpy as np


class GridMap:
    def __init__(self, data, width, height, resolution, origin_x, origin_y, occupied_threshold=65):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.occupied_threshold = int(occupied_threshold)
        self.data = np.array(data, dtype=np.int16).reshape((self.height, self.width))
        self._occupied_world_points = None

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
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
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

    def nearest_obstacle_distance(self, x, y):
        occupied = self.occupied_world_points()
        if occupied.size == 0:
            return float('inf')

        delta = occupied - np.array([x, y], dtype=np.float32)
        distances = np.linalg.norm(delta, axis=1)
        return float(np.min(distances))
