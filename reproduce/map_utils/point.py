from __future__ import annotations
import math


class Point:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __add__(self, point: Point) -> Point:
        return Point(self.x + point.x, self.y + point.y)

    def __sub__(self, point: Point) -> Point:
        return Point(self.x - point.x, self.y - point.y)

    def __truediv__(self, divisor: float) -> Point:
        #  print(self.x, self.y)
        return Point(self.x / divisor, self.y / divisor)

    def distance_square_to(self, point: Point) -> float:
        return (self.x - point.x) ** 2 + (self.y - point.y) ** 2

    def distance_to(self, point: Point) -> float:
        return math.hypot(self.x - point.x, self.y - point.y)
