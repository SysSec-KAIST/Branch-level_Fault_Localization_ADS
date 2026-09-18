from __future__ import annotations
from typing import Any

from .point import Point
import itertools


class AxisAlignedBoundingBox:
    def __init__(self, one_corner: Point, opposite_coner: Point) -> None:
        self.info = None
        self.data = None

        self.center = (one_corner + opposite_coner) / 2
        self.width = abs(one_corner.x - opposite_coner.x)
        self.height = abs(one_corner.y - opposite_coner.y)

        self.min = Point(
            self.center.x - (self.width / 2), self.center.y - (self.height / 2)
        )
        self.max = Point(
            self.center.x + (self.width / 2), self.center.y + (self.height / 2)
        )

        self.points = []
        for point in itertools.product(
            [self.min.x, self.max.x], [self.min.y, self.max.y]
        ):
            self.points.append(Point(point[0], point[1]))

    def add_meta(self, info: Any, data: Any) -> None:
        self.info = info
        self.data = data

    def is_x_overwrapped(self, bounding_box: AxisAlignedBoundingBox) -> bool:
        diff_x = abs(self.center.x - bounding_box.center.x)
        return (self.width + bounding_box.width) / 2 < diff_x

    def is_y_overwrapped(self, bounding_box: AxisAlignedBoundingBox) -> bool:
        diff_y = abs(self.center.y - bounding_box.center.y)
        return (self.height + bounding_box.height) / 2 < diff_y
