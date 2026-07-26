"""Pure coverage-grid model used by the ROS adapter."""

import math


class CoverageModel:
    """Accumulate distance and unique XY grid cells from mapping poses."""

    def __init__(self, cell_size):
        if cell_size <= 0.0:
            raise ValueError('cell_size must be positive')
        self.cell_size = float(cell_size)
        self.cells = set()
        self.distance = 0.0
        self.pose_count = 0
        self.last_position = None

    def add(self, x, y):
        """Add one finite XY pose and return whether it entered a new cell."""
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        position = (float(x), float(y))
        if self.last_position is not None:
            self.distance += math.hypot(
                position[0] - self.last_position[0],
                position[1] - self.last_position[1],
            )
        self.last_position = position
        self.pose_count += 1
        cell = (
            math.floor(position[0] / self.cell_size),
            math.floor(position[1] / self.cell_size),
        )
        previous_size = len(self.cells)
        self.cells.add(cell)
        return len(self.cells) != previous_size

    @property
    def area(self):
        """Return the visited grid-cell area in square metres."""
        return len(self.cells) * self.cell_size * self.cell_size

    def reset(self):
        """Clear all accumulated mapping preview state."""
        self.cells.clear()
        self.distance = 0.0
        self.pose_count = 0
        self.last_position = None
