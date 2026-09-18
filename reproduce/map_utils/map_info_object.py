from abc import ABC, abstractmethod
from typing import List
from .map_object import LineSegment
from modules.map.proto.map_lane_pb2 import Lane
from modules.map.proto.map_geometry_pb2 import Curve
from modules.map.proto.map_signal_pb2 import Signal
from modules.map.proto.map_junction_pb2 import Junction
from .point import Point


class MapInfoObject(ABC):
    @property
    @abstractmethod
    def id(self) -> str:
        pass


def points_from_curve(curve: Curve) -> List[Point]:
    points = []
    for c in curve.segment:
        if c.HasField("line_segment"):
            points += [Point(p.x, p.y) for p in c.line_segment.point]
    return remove_duplicated_points(points)


def segments_from_curve(curve: Curve) -> List[LineSegment]:
    points = points_from_curve(curve)
    segments = []
    for i in range(len(points) - 1):
        segments.append(LineSegment(points[i], points[i + 1]))
    return segments


def remove_duplicated_points(points: List[Point]) -> List[Point]:
    epsilon = 1e-1
    new_points = [points[0]]
    for point in points[1:]:
        if point.distance_to(new_points[-1]) < epsilon:
            new_points.append(point)
    return new_points


class LaneInfo(MapInfoObject):
    def __init__(self, lane: Lane) -> None:
        self.lane = lane
        self.points = points_from_curve(lane.central_curve)
        self.segments: List[LineSegment] = []

        for i in range(len(self.points) - 1):
            self.segments.append(LineSegment(self.points[i], self.points[i + 1]))

    @property
    def id(self) -> str:
        return self.lane.id.id


class SignalInfo(MapInfoObject):
    def __init__(self, signal: Signal) -> None:
        self.signal = signal
        self.segments: List[LineSegment] = []
        for stop_line in signal.stop_line:
            self.segments += segments_from_curve(stop_line)

    @property
    def id(self) -> str:
        return self.signal.id.id


class JunctionInfo(MapInfoObject):
    def __init__(self, junction: Junction) -> None:
        self.junction = junction
        self.polygon: List[Point] = []
        for point in junction.polygon.point:
            self.polygon.append(point)

    @property
    def id(self) -> str:
        return self.junction.id.id