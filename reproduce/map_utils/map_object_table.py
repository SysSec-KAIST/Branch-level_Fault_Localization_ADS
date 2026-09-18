from abc import ABC, abstractmethod
from typing import Dict, List
from .aabbox import AxisAlignedBoundingBox
from .map_info_object import LaneInfo, SignalInfo, JunctionInfo
from .object_with_aabbox import ObjectWithAxisAlignedBoundingBox
from .point import Point

from modules.map.proto.map_pb2 import Map
import math

class MapObjectTable(ABC):
    @abstractmethod
    def __init__(self, map_data: Map) -> None:
        pass

    @abstractmethod
    def _build_table(self, map_data: Map) -> None:
        pass


class LaneTable(MapObjectTable):
    def __init__(self, map_data: Map) -> None:
        self.table: Dict[str, LaneInfo] = {}
        self.boxes: List[ObjectWithAxisAlignedBoundingBox] = []
        self._build_table(map_data)

    def _build_table(self, map_data: Map) -> None:
        for lane in map_data.lane:
            lane_info = LaneInfo(lane)
            for segment in lane_info.segments:
                aabbox = AxisAlignedBoundingBox(segment.start, segment.end)
                segment_aabbox = ObjectWithAxisAlignedBoundingBox(
                    aabbox, segment, lane_info
                )
                self.boxes.append(segment_aabbox)
            self.table[lane_info.id] = lane_info


class SignalTable(MapObjectTable):
    def __init__(self, map_data: Map) -> None:
        self.table: Dict[str, SignalInfo] = {}
        self.boxes: List[ObjectWithAxisAlignedBoundingBox] = []
        self._build_table(map_data)

    def _build_table(self, map_data: Map) -> None:
        for signal in map_data.signal:
            signal_info = SignalInfo(signal)
            for segment in signal_info.segments:
                aabbox = AxisAlignedBoundingBox(segment.start, segment.end)
                segment_aabbox = ObjectWithAxisAlignedBoundingBox(
                    aabbox, segment, signal_info
                )
                self.boxes.append(segment_aabbox)
            self.table[signal_info.id] = signal_info

class JunctionTable(MapObjectTable):
    def __init__(self, map_data: Map) -> None:
        self.table: Dict[str, JunctionInfo] = {}
        self.boxes: List[ObjectWithAxisAlignedBoundingBox] = []
        self._build_table(map_data)

    def _build_table(self, map_data: Map) -> None:
        for junction in map_data.junction:
            junction_info = JunctionInfo(junction)
            x_min = y_min = math.inf
            x_max = y_max = -math.inf
            for point in junction_info.polygon:
                if point.x < x_min:
                    x_min = point.x
                if point.x > x_max:
                    x_max = point.x
                if point.y < y_min:
                    y_min = point.y
                if point.y > y_max:
                    y_max = point.y
            self.boxes.append([x_min, y_min, x_max, y_max])
            self.table[junction_info.id] = junction_info