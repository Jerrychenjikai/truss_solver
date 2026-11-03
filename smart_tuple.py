from __future__ import annotations
import math
from typing import Sequence, Union, Tuple, List, Optional

Numeric = Union[int, float]

class SmartTuple(tuple):
    """
    重载运算符的自定义 tuple 类：
    1. SmartTuple + SmartTuple: 支持等长且元素全为数字的逐元素相加。
    2. SmartTuple * float/int: 支持逐元素标量乘法。
    """

    @staticmethod
    def _is_numeric_tuple(t: Sequence) -> bool:
        return all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in t)

    def __add__(self, other: Sequence) -> SmartTuple:
        if isinstance(other, (tuple, list, SmartTuple)):
            if len(self) != len(other):
                raise ValueError(f"长度不匹配，无法逐元素相加：len={len(self)} 与 len={len(other)}")
            if not (self._is_numeric_tuple(self) and self._is_numeric_tuple(other)):
                raise TypeError("两元组元素必须均为 int 或 float 类型才可以进行逐元素相加")
            return SmartTuple(x + y for x, y in zip(self, other))
        return NotImplemented

    def __radd__(self, other: Sequence) -> SmartTuple:
        return self.__add__(other)

    def __mul__(self, other: Numeric) -> SmartTuple:
        if isinstance(other, (int, float)) and not isinstance(other, bool):
            if not self._is_numeric_tuple(self):
                raise TypeError("当前 SmartTuple 包含非数字元素，无法进行逐元素相乘")
            return SmartTuple(x * other for x in self)
        return NotImplemented

    def __rmul__(self, other: Numeric) -> SmartTuple:
        return self.__mul__(other)


class Vector2D(SmartTuple):
    """专门代表二维 (x, y) 数值向量的 SmartTuple"""

    def __new__(cls, x: Numeric = 0.0, y: Numeric = 0.0) -> Vector2D:
        return super().__new__(cls, (float(x), float(y)))

    @property
    def x(self) -> float:
        return self[0]

    @property
    def y(self) -> float:
        return self[1]

    def set_x(self, x) -> Vector2D:
        return Vector2D(x, self.y)

    def set_y(self, y) -> Vector2D:
            return Vector2D(self.x, y)

    def get_angle_radians(self) -> float:
        """获取向量弧度角 (-π 到 π)"""
        return math.atan2(self.y, self.x)

    def get_angle_degree(self) -> float:
        """获取角度值 (-180° 到 180°)"""
        return math.degrees(self.get_angle_radians())

    def get_magnitude(self) -> float:
        """获取向量的模长 (Magnitude)"""
        return math.hypot(self.x, self.y)

    def get_direction(self) -> Vector2D:
        """获取归一化方向向量 (Unit Vector)"""
        mag = self.get_magnitude()
        if mag == 0:
            return Vector2D(0.0, 0.0)
        return Vector2D(self.x / mag, self.y / mag)

    def with_magnitude(self, new_magnitude: Numeric) -> Vector2D:
        """保持方向不变，返回更改模长后的新向量"""
        direction = self.get_direction()
        dir_x, dir_y = direction.x, direction.y
        if dir_x == 0 and dir_y == 0 and new_magnitude != 0:
            dir_x = 1.0
        return Vector2D(dir_x * new_magnitude, dir_y * new_magnitude)

    def with_direction(self, direction: Sequence[Numeric]) -> Vector2D:
        """保持模长不变，返回更改方向后的新向量"""
        dir_x, dir_y = float(direction[0]), float(direction[1])
        dir_mag = math.hypot(dir_x, dir_y)
        if dir_mag == 0:
            raise ValueError("Direction vector cannot be a zero vector.")
        
        current_mag = self.get_magnitude()
        unit_x = dir_x / dir_mag
        unit_y = dir_y / dir_mag
        return Vector2D(unit_x * current_mag, unit_y * current_mag)

    def is_parallel(self, other_vector: Vector2D) -> bool:
        """判断当前向量与另一个向量是否平行"""
        cross_product = self.x * other_vector.y - self.y * other_vector.x
        return math.isclose(cross_product, 0.0, abs_tol=1e-9)