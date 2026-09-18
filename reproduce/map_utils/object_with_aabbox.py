from typing import Generic, TypeVar


from .aabbox import AxisAlignedBoundingBox
from .map_object import MapObject
from .map_info_object import MapInfoObject
from .point import Point

T = TypeVar("T", bound=MapObject)
S = TypeVar("S", bound=MapInfoObject)
AABBox = TypeVar("AABBox", bound=AxisAlignedBoundingBox)


class ObjectWithAxisAlignedBoundingBox(Generic[T, S, AABBox]):
    def __init__(self, aabbox: AABBox, object: T, info: S) -> None:
        self.object = object
        self.aabbox = aabbox
        self.info = info

    def distance_square_to(self, point: Point) -> float:
        return self.object.distance_square_to(point)

    @property
    def id(self) -> str:
        return self.info.id
