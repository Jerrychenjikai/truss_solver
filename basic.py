from __future__ import annotations
from smart_tuple import *
import math
from typing import Sequence, Union, Tuple, List, Optional

Numeric = Union[int, float]
COMP_TOL = 1e-12
DET_TOL = 1e-9


def _clean(value: float) -> float:
    if abs(value) < 1e-10:
        return 0.0
    return value


def _as_component(value: Optional[Numeric]) -> Optional[float]:
    if value is None:
        return None
    return float(value)


class Force:
    """外力。分量为 None 表示大小未定。

    任一未定分量都使该力成为反力；两个分量都是具体数值时它是已知荷载。
    求解得到的反力写在 *_solved，不会覆盖用户指定的 None。
    """

    def __init__(self, value: Union[Vector2D, Sequence[Optional[Numeric]], None] = None) -> None:
        if value is None:
            raw_x: Optional[Numeric] = 0.0
            raw_y: Optional[Numeric] = 0.0
        else:
            raw_x, raw_y = value[0], value[1]
        self.fx_prescribed: Optional[float] = _as_component(raw_x)
        self.fy_prescribed: Optional[float] = _as_component(raw_y)
        self.fx_solved: Optional[float] = None
        self.fy_solved: Optional[float] = None

    @property
    def is_reaction(self) -> bool:
        return self.fx_prescribed is None or self.fy_prescribed is None

    def effective_x(self) -> float:
        if self.fx_prescribed is not None:
            return self.fx_prescribed
        if self.fx_solved is not None:
            return self.fx_solved
        return 0.0

    def effective_y(self) -> float:
        if self.fy_prescribed is not None:
            return self.fy_prescribed
        if self.fy_solved is not None:
            return self.fy_solved
        return 0.0

    @property
    def value(self) -> Vector2D:
        return Vector2D(self.effective_x(), self.effective_y())

    def set_x(self, x: Optional[Numeric]) -> None:
        self.fx_prescribed = _as_component(x)
        self.fx_solved = None
        self.fy_solved = None

    def set_y(self, y: Optional[Numeric]) -> None:
        self.fy_prescribed = _as_component(y)
        self.fx_solved = None
        self.fy_solved = None

    def set_value(self, value: Sequence[Optional[Numeric]]) -> None:
        self.fx_prescribed = _as_component(value[0])
        self.fy_prescribed = _as_component(value[1])
        self.fx_solved = None
        self.fy_solved = None

    def _fmt_component(self, prescribed: Optional[float], solved: Optional[float]) -> str:
        if prescribed is None:
            if solved is None:
                return "None"
            return f"None->{solved:.2f}"
        return f"{prescribed:.2f}"

    def __repr__(self) -> str:
        kind = "reaction" if self.is_reaction else "load"
        fx = self._fmt_component(self.fx_prescribed, self.fx_solved)
        fy = self._fmt_component(self.fy_prescribed, self.fy_solved)
        return f"Force(fx={fx}, fy={fy}, {kind})"


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
        if self.truss is not None:
            self.truss.unsolve()

    def _equilibrium_system(
        self,
        virtual: bool = False,
        target_joint: Optional[Joint] = None,
        unit_force: Optional[Force] = None,
    ):
        """返回 (fx_known, fy_known, unknowns)。

        unknowns 的每一项是 (kind, obj, cx, cy)：
        kind 为 "member" / "fx" / "fy"。系数是该未知量在 ΣFx、ΣFy 中的系数。
        正的构件轴力为拉力，作用在本节点上的方向指向另一端。

        虚功工况只排除两个分量都已知的荷载；反力整体保留
        （已指定的分量，通常是 0，仍作为已知值，未定分量继续当未知量）。
        """
        fx = 0.0
        fy = 0.0
        unknowns = []

        for force in self.forces:
            if virtual and not force.is_reaction:
                continue
            if force.fx_prescribed is not None:
                fx += force.fx_prescribed
            elif force.fx_solved is not None:
                fx += force.fx_solved
            else:
                unknowns.append(("fx", force, 1.0, 0.0))

            if force.fy_prescribed is not None:
                fy += force.fy_prescribed
            elif force.fy_solved is not None:
                fy += force.fy_solved
            else:
                unknowns.append(("fy", force, 0.0, 1.0))

        if virtual and target_joint is self and unit_force is not None:
            fx += unit_force.effective_x()
            fy += unit_force.effective_y()

        for member in self.connected_members:
            if member.force_determined():
                contrib = member.force_for_joint(self)
                fx += contrib.effective_x()
                fy += contrib.effective_y()
                continue

            other = member.joints[1] if member.joints[0] is self else member.joints[0]
            delta = other.pos + (self.pos * -1)
            if delta.get_magnitude() < COMP_TOL:
                ux, uy = 0.0, 0.0
            else:
                unit = delta.get_direction()
                ux, uy = unit.x, unit.y
            unknowns.append(("member", member, ux, uy))

        return fx, fy, unknowns

    @staticmethod
    def _unknowns_are_solvable(unknowns) -> bool:
        count = len(unknowns)
        if count == 0:
            return True
        if count == 1:
            _, _, ux, uy = unknowns[0]
            return abs(ux) > COMP_TOL or abs(uy) > COMP_TOL
        if count == 2:
            _, _, u1x, u1y = unknowns[0]
            _, _, u2x, u2y = unknowns[1]
            det = u1x * u2y - u1y * u2x
            return not math.isclose(det, 0.0, abs_tol=DET_TOL)
        return False

    def solvable(self):
        # 未定的外力分量与未定的构件轴力一样，都算未知量。一个节点最多两个独立未知量。
        _, _, unknowns = self._equilibrium_system()
        return self._unknowns_are_solvable(unknowns)

    def solved(self):
        for member in self.connected_members:
            if not member.force_determined():
                return False
        for force in self.forces:
            if force.fx_prescribed is None and force.fx_solved is None:
                return False
            if force.fy_prescribed is None and force.fy_solved is None:
                return False
        return True

    def solve(self, virtual: bool = False, target_joint: Optional[Joint] = None, force: Optional[Force] = None):
        fx_known, fy_known, unknowns = self._equilibrium_system(virtual, target_joint, force)
        if not self._unknowns_are_solvable(unknowns):
            raise ValueError("This joint is not yet solvable")

        values = self._solve_unknown_values(unknowns, fx_known, fy_known)
        for (kind, obj, _, _), value in zip(unknowns, values):
            if kind == "member":
                obj.force = value
            elif kind == "fx":
                obj.fx_solved = value
            else:
                obj.fy_solved = value

    @staticmethod
    def _solve_unknown_values(unknowns, fx_known: float, fy_known: float) -> List[float]:
        if len(unknowns) == 0:
            return []

        if len(unknowns) == 1:
            _, _, ux, uy = unknowns[0]
            # 选绝对值较大的分量，避免除以接近 0 的投影
            if abs(ux) > abs(uy):
                if abs(ux) < COMP_TOL:
                    raise ValueError("Cannot solve: unknown does not appear in equilibrium.")
                return [_clean(-fx_known / ux)]
            if abs(uy) < COMP_TOL:
                raise ValueError("Cannot solve: unknown does not appear in equilibrium.")
            return [_clean(-fy_known / uy)]

        _, _, u1x, u1y = unknowns[0]
        _, _, u2x, u2y = unknowns[1]
        det = u1x * u2y - u1y * u2x
        if math.isclose(det, 0.0, abs_tol=DET_TOL):
            raise ValueError("Cannot solve: the two unknowns are linearly dependent.")

        f1 = ((-fx_known) * u2y - (-fy_known) * u2x) / det
        f2 = (u1x * (-fy_known) - u1y * (-fx_known)) / det
        return [_clean(f1), _clean(f2)]


class Member:
    def __init__(self, joint1: Joint, joint2: Joint, E: float = 200000, A: float = 10) -> None:
        self.joints: SmartTuple = SmartTuple((joint1, joint2))
        self.force: Optional[float] = None  # 正值为拉力 (tension)，负值为压力 (compression)
        self.E: float = E
        self.A: float = A

    def force_determined(self):
        return self.force is not None

    def length(self):
        return (self.joints[0].pos + (-1*self.joints[1].pos)).get_magnitude()
    
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

    def unsolve(self, clear_displacement: bool = True):
        self.forces_solved = False
        for member in self.members:
            member.force = None
        for joint in self.joints:
            if clear_displacement:
                joint.displacement = Vector2D(0.0, 0.0)
            for force in joint.forces:
                force.fx_solved = None
                force.fy_solved = None

    def add_joint(self, joint: Joint):
        self.joints.append(joint)
        joint.truss = self

    def add_member(self, member: Member):
        self.members.append(member)
        self.unsolve()

    def _restore_real_case(self, real_member_forces: List[Optional[float]], real_reaction_state) -> None:
        for member, force in zip(self.members, real_member_forces):
            member.force = force
        for force, fx_solved, fy_solved in real_reaction_state:
            force.fx_solved = fx_solved
            force.fy_solved = fy_solved
        self.forces_solved = True

    def solve_forces(self, virtual: bool = False, target_joint: Optional[Joint] = None, force: Optional[Force] = None, clear_displacement: Optional[bool] = None):
        if clear_displacement is None:
            clear_displacement = not virtual
        self.unsolve(clear_displacement=clear_displacement)

        for member in self.members:
            if member.length() < COMP_TOL:
                raise ValueError("A member has zero length.")

        solved = len(self.joints) == 0
        progress = True
        guard = 0
        limit = max(4, len(self.joints) * len(self.joints) + 2)

        while not solved and progress and guard < limit:
            guard += 1
            progress = False
            solved = True

            for joint in self.joints:
                was_solved = joint.solved()
                if not was_solved and joint.solvable():
                    joint.solve(virtual, target_joint, force)
                if not joint.solved():
                    solved = False
                elif not was_solved:
                    progress = True

        if not solved:
            raise Warning(
                "This truss has unsolvable components. "
                "A joint can resolve at most two unknowns that are not collinear "
                "(members or reaction components)."
            )
        self.forces_solved = True

    def solve_displacement(self):
        self.solve_forces()

        # 真实轴力和反力要留到虚功循环之后恢复。循环里每次 solve_forces 都会清空它们。
        real_member_forces = [member.force for member in self.members]
        real_reaction_state = [
            (force, force.fx_solved, force.fy_solved)
            for joint in self.joints
            for force in joint.forces
        ]
        directions = (Vector2D(1, 0), Vector2D(0, 1))

        try:
            for joint in self.joints:
                disp_x, disp_y = 0.0, 0.0
                for axis, direction in enumerate(directions):
                    # 虚功：保留全部反力未知量，去掉已知荷载，只在目标节点加单位力。
                    self.solve_forces(
                        virtual=True,
                        target_joint=joint,
                        force=Force(direction),
                        clear_displacement=False,
                    )

                    displacement = 0.0
                    for index, member in enumerate(self.members):
                        real_f = real_member_forces[index]
                        virtual_f = member.force
                        if real_f is None or virtual_f is None:
                            continue
                        ea = member.E * member.A
                        if ea == 0:
                            raise ValueError("A member has zero EA, so displacement is undefined.")
                        # 单位力法：Δ = Σ (F_real * F_virtual * L) / (E * A)
                        displacement += (real_f * virtual_f * member.length()) / ea

                    if axis == 0:
                        disp_x = displacement
                    else:
                        disp_y = displacement

                joint.displacement = Vector2D(disp_x, disp_y)
        except Exception:
            self._restore_real_case(real_member_forces, real_reaction_state)
            for joint in self.joints:
                joint.displacement = Vector2D(0.0, 0.0)
            raise

        self._restore_real_case(real_member_forces, real_reaction_state)
