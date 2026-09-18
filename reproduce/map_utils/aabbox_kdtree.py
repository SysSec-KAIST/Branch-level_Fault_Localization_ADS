from __future__ import annotations
from typing import List, Tuple, Optional

from .point import Point
from enum import Enum
from functools import cmp_to_key
from .object_with_aabbox import ObjectWithAxisAlignedBoundingBox


class PartitionAxis(Enum):
    X = 1
    Y = 2


class AxisAlignedBoundingBoxKDTreeConfig:
    def __init__(self) -> None:
        self.max_depth = None
        self.max_leaf_size = 1
        self.max_leaf_dimension = None


class AxisAlignedBoundingBoxKDTree:
    def __init__(
        self,
        objects: List[ObjectWithAxisAlignedBoundingBox],
        config: AxisAlignedBoundingBoxKDTreeConfig,
    ) -> None:
        self.objects = objects
        self.root = AxisAlignedBoundingBoxKDTreeNode(self.objects, 0, config)


class AxisAlignedBoundingBoxKDTreeNode:
    def __init__(
        self,
        objects: List[ObjectWithAxisAlignedBoundingBox],
        depth: float,
        config: AxisAlignedBoundingBoxKDTreeConfig,
    ) -> None:
        self.max_depth = config.max_depth
        self.max_leaf_size = config.max_leaf_size
        self.max_leaf_dimension = config.max_leaf_dimension
        self.objects = objects
        self.depth = depth

        self.left_subnode = None
        self.right_subnode = None

        self.__compute_boundary()
        self.__compute_partition()

        if self.__should_split_to_subnodes():
            (left_subnodes, right_subnodes) = self.__partition_nodes()
            if left_subnodes:
                self.left_subnode = AxisAlignedBoundingBoxKDTreeNode(
                    left_subnodes, depth + 1, config
                )
            if right_subnodes:
                self.right_subnode = AxisAlignedBoundingBoxKDTreeNode(
                    right_subnodes, depth + 1, config
                )
        else:
            self.__sort_and_init_bounds()

    def get_nearest_object(
        self,
        point: Point,
        nearest_object: Optional[ObjectWithAxisAlignedBoundingBox] = None,
        current_min: float = float("inf"),
    ) -> Tuple[ObjectWithAxisAlignedBoundingBox, float]:
        if not nearest_object:
            nearest_object = self.objects[0]
        (nearest_object, minimum_distance_square) = self.__get_nearest_object(
            point, nearest_object, current_min
        )
        return (nearest_object, minimum_distance_square)

    def __get_nearest_object(
        self,
        point: Point,
        nearest_object: ObjectWithAxisAlignedBoundingBox,
        current_min: float,
    ) -> Tuple[ObjectWithAxisAlignedBoundingBox, float]:
        anchor = point.x if self.partition_axis is PartitionAxis.X else point.y
        if anchor < self.partition_position:
            if self.left_subnode:
                self.__traverse(self.left_subnode, point, nearest_object, current_min)
            for object, bound in zip(self.sorted_by_min, self.sorted_by_min_bounds):
                if bound > anchor and (bound - anchor) ** 2 > current_min:
                    break
                distance = object.distance_square_to(point)
                if distance < current_min:
                    current_min = distance
                    nearest_object = object
            if self.right_subnode:
                self.__traverse(self.right_subnode, point, nearest_object, current_min)
        else:
            if self.left_subnode:
                self.__traverse(self.left_subnode, point, nearest_object, current_min)
            for object, bound in zip(self.sorted_by_max, self.sorted_by_max_bounds):
                if bound < anchor and (bound - anchor) ** 2 < current_min:
                    break
                distance = object.distance_square_to(point)
                if distance < current_min:
                    current_min = distance
                    nearest_object = object
            if self.right_subnode:
                self.__traverse(self.right_subnode, point, nearest_object, current_min)

        return (nearest_object, current_min)

    def __traverse(
        self,
        subnode: AxisAlignedBoundingBoxKDTreeNode,
        point: Point,
        nearest_object: ObjectWithAxisAlignedBoundingBox,
        current_min: float,
    ) -> Tuple[ObjectWithAxisAlignedBoundingBox, float]:
        (object, local_min) = subnode.get_nearest_object(
            point, nearest_object, current_min
        )
        if local_min < current_min:
            current_min = local_min
            nearest_object = object
        return (nearest_object, current_min)

    def __compute_boundary(self) -> None:
        max_x = float("-inf")
        min_x = float("inf")
        max_y = float("-inf")
        min_y = float("inf")
        for object in self.objects:
            aabbox = object.aabbox
            max_x = max(max_x, aabbox.max.x)
            min_x = min(min_x, aabbox.min.x)
            max_y = max(max_y, aabbox.max.y)
            min_y = min(min_y, aabbox.min.y)
        self.max = Point(max_x, max_y)
        self.min = Point(min_x, min_y)
        self.center = (self.max + self.min) / 2

    def __compute_partition(self) -> None:
        if self.max.x - self.min.x >= self.max.y - self.min.y:
            self.partition_axis = PartitionAxis.X
            self.partition_position = self.center.x
        else:
            self.partition_axis = PartitionAxis.Y
            self.partition_position = self.center.y

    def __should_split_to_subnodes(self) -> bool:
        if self.max_depth and self.max_depth <= self.depth:
            return False
        if len(self.objects) <= self.max_leaf_size:
            return False
        if (
            self.max_leaf_dimension
            and max(self.max.x - self.min.x, self.max.y - self.min.y)
            <= self.max_leaf_dimension
        ):
            return False
        return True

    def __partition_nodes(
        self,
    ) -> Tuple[
        List[ObjectWithAxisAlignedBoundingBox], List[ObjectWithAxisAlignedBoundingBox]
    ]:
        left_subnodes = []
        right_subnodes = []

        for object in self.objects:
            aabbox = object.aabbox
            anchor = (
                aabbox.max.x if self.partition_axis is PartitionAxis.X else aabbox.max.y
            )
            if anchor <= self.partition_position:
                left_subnodes.append(object)
            else:
                right_subnodes.append(object)

        return (left_subnodes, right_subnodes)

    def __sort_and_init_bounds(self) -> None:
        def compare_max(object1, object2):
            aabbox1 = object1.aabbox
            aabbox2 = object2.aabbox
            if self.partition_axis is PartitionAxis.X:
                return aabbox1.max_x - aabbox2.max_x
            else:
                return aabbox1.max_y - aabbox2.max_y

        def compare_min(object1, object2):
            aabbox1 = object1.aabbox
            aabbox2 = object2.aabbox
            if self.partition_axis is PartitionAxis.X:
                return aabbox1.min_x - aabbox2.min_x
            else:
                return aabbox1.min_y - aabbox2.min_y

        self.sorted_by_max = sorted(self.objects, key=cmp_to_key(compare_max))
        self.sorted_by_min = sorted(self.objects, key=cmp_to_key(compare_min))

        self.sorted_by_max_bounds = [
            object.aabbox.max.x
            if self.partition_axis is PartitionAxis.X
            else object.aabbox.max.y
            for object in self.sorted_by_max
        ]
        self.sorted_by_min_bounds = [
            object.aabbox.min.x
            if self.partition_axis is PartitionAxis.X
            else object.aabbox.min.y
            for object in self.sorted_by_max
        ]
