from __future__ import annotations
from smart_tuple import *
import math
from typing import Sequence, Union, Tuple, List, Optional

Numeric = Union[int, float]

class Force:
    def __init__(self, value: Vector2D = Vector2D(0,0)) -> None:
        self.value: Vector2D = value

    def set_x(self, x: Numeric) -> None:
        self.value = self.value.set_x(x)

    def set_y(self, y: Numeric) -> None:
        self.value = self.value.set_y(y)

    def set_value(self, value: Sequence[Numeric]) -> None:
        self.value = Vector2D(value[0], value[1])

    def __repr__(self) -> str:
        return f"Force(x={self.value.x:.2f}, y={self.value.y:.2f})"


class Joint:
    def __init__(self, pos: Sequence[Numeric]) -> None:
        self.pos: Vector2D = Vector2D(pos[0], pos[1])
        self.displacement: Vector2D = Vector2D(0.0, 0.0) #assume displacement very small compared to position
        self.forces: List[Force] = []  # 节点受到的外力列表
        self.connected_members: List[Member] = []
        self.truss: Optional[Truss] = None

    def add_member(self, member: Member) -> None:
        self.connected_members.append(member)

    def add_force(self, force: Force) -> None:
        self.forces.append(force)
        self.truss.forces_solved = False

    def solvable(self):
        unsolved = []

        for i in self.connected_members:
            if not i.force_determined():
                unsolved.append(i)

        if len(unsolved) <= 1:
            return True
        if len(unsolved) == 2:
            # 依赖 Vector2D 的 is_parallel 方法进行判断
            if unsolved[0].get_direction_vector().is_parallel(unsolved[1].get_direction_vector()):
                return False
            return True
        return False

    def solved(self):
        for i in self.connected_members:
            if not i.force_determined():
                return False

        return True

    def solve(self, virtual: bool = False, target_joint: Optional[Joint] = None, force: Force = None):
        if not self.solvable():
            raise ValueError("This joint is not yet solvable")

        unsolved = []

        if not virtual:
            net_force = Force(Vector2D(0.0, 0.0))
            # 1. 汇总所有外力
            for f in self.forces:
                net_force.value += f.value
        else:
            net_force = Force(Vector2D(0.0, 0.0)) if target_joint != self else force

        # 2. 汇总已知的构件作用力，并筛选出未知的构件
        for m in self.connected_members:
            if m.force_determined():
                net_force.value += m.force_for_joint(self).value
            else:
                unsolved.append(m)

        fx_known = net_force.value.x
        fy_known = net_force.value.y

        # 3. 计算 1 个未知构件的情况
        if len(unsolved) == 1:
            m1 = unsolved[0]
            # 直接计算背离当前节点的单位方向向量
            other_joint = m1.joints[1] if m1.joints[0] == self else m1.joints[0]
            unit_dir = (other_joint.pos + (self.pos * -1)).get_direction()
            u1x, u1y = unit_dir.x, unit_dir.y
            
            # 为了防止除以0，选择绝对值较大的那个分量进行计算
            if abs(u1x) > abs(u1y):
                m1.force = -fx_known / u1x
            else:
                m1.force = -fy_known / u1y

        # 4. 计算 2 个未知构件的情况 (求解二元一次方程组)
        elif len(unsolved) == 2:
            m1, m2 = unsolved[0], unsolved[1]
            
            # 分别获取两个未知构件的单位方向向量
            other_joint1 = m1.joints[1] if m1.joints[0] == self else m1.joints[0]
            unit_dir1 = (other_joint1.pos + (self.pos * -1)).get_direction()
            u1x, u1y = unit_dir1.x, unit_dir1.y
            
            other_joint2 = m2.joints[1] if m2.joints[0] == self else m2.joints[0]
            unit_dir2 = (other_joint2.pos + (self.pos * -1)).get_direction()
            u2x, u2y = unit_dir2.x, unit_dir2.y

            # 建立方程使用克莱姆法则求解，先计算行列式
            det = u1x * u2y - u1y * u2x
            
            if math.isclose(det, 0.0, abs_tol=1e-9):
                raise ValueError("Cannot solve: The two unknown members are collinear.")

            # 求解 F1 和 F2
            f1 = ((-fx_known) * u2y - (-fy_known) * u2x) / det
            f2 = (u1x * (-fy_known) - u1y * (-fx_known)) / det

            m1.force = f1
            m2.force = f2


class Member:
    def __init__(self, joint1: Joint, joint2: Joint, E: float = 200000, A: float = 10) -> None:
        self.joints: SmartTuple = SmartTuple((joint1, joint2))
        self.force: Optional[float] = None  # 正值为拉力 (tension)，负值为压力 (compression)
        self.E: float = E
        self.A: float = A

    def force_determined(self):
        return self.force is not None

    def length(self):
        return (self.joints[0] + (-1*self.joints[1])).get_magnitude()
    
    def force_for_joints(self) -> Tuple[Force, Force]:
        """
        计算该构件施加给两个 Joint 的作用力向量
        """
        if self.force is None:
            raise ValueError("force of this member not determined yet")
        
        j1, j2 = self.joints[0], self.joints[1]
        
        delta = j2.pos + (j1.pos * -1) 
        
        if delta.get_magnitude() == 0:
            return Force(Vector2D(0.0, 0.0)), Force(Vector2D(0.0, 0.0))
            
        unit_dir = delta.get_direction()
        
        f_j1 = Force(unit_dir * self.force)
        f_j2 = Force(unit_dir * (-self.force))
        
        return f_j1, f_j2

    def force_for_joint(self, joint: Joint) -> Force:
        if joint not in self.joints:
            raise ValueError("this joint is not connected to this member")

        return self.force_for_joints()[self.joints.index(joint)]
    
    def get_direction_vector(self) -> Vector2D:
        """获取从 joint1 指向 joint2 的方向向量 (dx, dy)"""
        j1_pos = self.joints[0].pos
        j2_pos = self.joints[1].pos
        return j2_pos + (j1_pos * -1)


class Truss:
    def __init__(self):
        self.joints: List[Joint] = []
        self.members: List[Member] = []

        self.forces_solved = False

    def unsolve(self):
        self.forces_solved = False
        for i in self.members:
            i.force = None

    def add_joint(self, joint: Joint):
        self.joints.append(joint)
        joint.truss = self

    def add_member(self, member: Member):
        self.members.append(member)  # 修正了原代码拼写错误

    def solve_forces(self, virtual: bool = False, target_joint: Optional[Joint] = None, force: Force = None):
        self.unsolve()

        solved = False
        has_solvable_component = True

        while not solved and has_solvable_component:
            has_solvable_component = False
            solved = True

            for i in self.joints:
                if i.solvable():
                    has_solvable_component = True
                    i.solve(virtual, target_joint, force)

                if not i.solved():
                    solved = False

        if not solved:
            raise Warning("This truss has unsolvable components")
        self.forces_solved = solved

    def solve_displacement(self):
        directions = [Force(Vector2D(1, 0)), Force(Vector2D(0, 1))]

        self.solve_forces()

        # 拷贝一份 members 的真实受力数据用于后续相乘
        real_forces = [member.force for member in self.members]

        for i in self.joints:
            # 用于暂存 x 和 y 方向的位移结果
            disp_x, disp_y = 0.0, 0.0
            
            for j in range(len(directions)):
                # 施加虚拟单位力并计算此时的构件虚拟内力
                self.solve_forces(virtual=True, target_joint=i, force=directions[j])

                # 进行乘法，利用每个 member 的 E 和 A 和 length 属性计算位移分量
                displacement = 0.0
                for k, member in enumerate(self.members):
                    real_f = real_forces[k]
                    virtual_f = member.force
                    
                    if real_f is not None and virtual_f is not None:
                        L = member.length()
                        E = member.E
                        A = member.A
                        # 单位力法核心公式：Δ = Σ (F_real * F_virtual * L) / (E * A)
                        displacement += (real_f * virtual_f * L) / (E * A)

                if j == 0:
                    disp_x = displacement
                else:
                    disp_y = displacement
            
            # Vector2D 的 set_x 和 set_y 返回的是新对象，必须重新赋值。
            # 注意：basic.py 原文 Joint 的初始化中将该属性命名为 displacement
            i.displacement = i.displacement.set_x(disp_x).set_y(disp_y)
