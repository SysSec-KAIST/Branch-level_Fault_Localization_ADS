from abc import ABC, abstractmethod

from .point import Point
import math


class MapObject(ABC):
    @abstractmethod
    def distance_square_to(self, point: Point) -> float:
        pass


class LineSegment(MapObject):
    def __init__(self, start: Point, end: Point) -> None:
        self.start = start
        self.end = end
        self.length = start.distance_to(end)
        self.unit_direction = Point(0, 0)
        if self.length > 1e-10:
            self.unit_direction = (end - start) / self.length
        self.heading = math.atan2(self.unit_direction.y, self.unit_direction.x)

    def distance_square_to(self, point: Point) -> float:
        x = point.x - self.start.x
        y = point.y - self.start.y

        proj = x * self.unit_direction.x + y * self.unit_direction.y
        if proj <= 0:
            return x**2 + y**2

        if proj >= self.length:
            return point.distance_square_to(self.end)

        return (x * self.unit_direction.y - y * self.unit_direction.x) ** 2