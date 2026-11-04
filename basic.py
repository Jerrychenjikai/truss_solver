from __future__ import annotations
from smart_tuple import *
from matrix_solver import solve_matrix
import math
from typing import Sequence, Union, Tuple, List, Optional

Numeric = Union[int, float]
COMP_TOL = 1e-12
DET_TOL = 1e-9
EQ_TOL = 1e-8

def _clean(value: float) -> float:
    if abs(value) < 1e-10:
        return 0.0
    return value


def _as_component(value: Optional[Numeric]) -> Optional[float]:
    if value is None:
        return None
    return float(value)


def linear_solution(
    rows: List[List[float]],
    rhs: List[float],
    tol: float = EQ_TOL,
) -> List[Optional[float]]:
    """解 rows @ x = rhs，返回每个变量的值；不能唯一确定的位置为 None。

    方程可以少于或多于未知量。某一未知量唯一，当且仅当零空间里它总是 0。
    矛盾方程抛出 ValueError。
    """
    m = len(rows)
    n = len(rows[0]) if m else 0
    if any(len(row) != n for row in rows):
        raise ValueError("Equilibrium matrix is not rectangular.")

    scale = 1.0
    for row in rows:
        for value in row:
            scale = max(scale, abs(value))
    for value in rhs:
        scale = max(scale, abs(value))
    abs_tol = tol * scale

    if n == 0:
        for value in rhs:
            if abs(value) > abs_tol:
                raise ValueError(
                    "System cannot be in equilibrium. "
                    "Not enough members to balance the loads."
                )
        return []

    aug = [list(rows[r]) + [rhs[r]] for r in range(m)]
    pivot_of_col: dict[int, int] = {}
    rank = 0
    for col in range(n):
        if rank >= m:
            break
        pivot = max(range(rank, m), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) <= abs_tol:
            continue
        if pivot != rank:
            aug[rank], aug[pivot] = aug[pivot], aug[rank]
        pivot_val = aug[rank][col]
        aug[rank] = [value / pivot_val for value in aug[rank]]
        for r in range(m):
            if r == rank:
                continue
            factor = aug[r][col]
            if abs(factor) <= abs_tol:
                continue
            aug[r] = [aug[r][c] - factor * aug[rank][c] for c in range(n + 1)]
        pivot_of_col[col] = rank
        rank += 1

    for r in range(rank, m):
        if abs(aug[r][n]) > abs_tol:
            raise ValueError(
                "System cannot be in equilibrium. "
                "Not enough members to balance the loads."
            )

    free_cols = [col for col in range(n) if col not in pivot_of_col]
    solution: List[Optional[float]] = [None] * n
    for col, row_index in pivot_of_col.items():
        if any(abs(aug[row_index][free]) > abs_tol for free in free_cols):
            continue
        solution[col] = _clean(aug[row_index][n])
    return solution



_unique_linear_solution = linear_solution # or change it to solve_matrix, the dedault function



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

    def x_solved(self) -> bool:
        if (self.fx_prescribed is not None or self.fx_solved is not None):
            return True
        return False

    def y_solved(self) -> bool:
        if (self.fy_prescribed is not None or self.fy_solved is not None):
            return True
        return False

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
        # 节点可以不挂在桁架上（测试用）。没有桁架时不改桁架侧的任何状态。
        if self.truss is not None:
            self.truss.add_external_force(force)

    def solve(self, unknowns: List[Unknown], 
              virtual: bool = False, 
              target_joint: Optional[Joint] = None, 
              force: Optional[Force] = None) -> Tuple[List[List[float]], List[float]]:
        
        def find_index(member: Union[Force, Member], unknowns: List[Unknown], kind: Optional[str] = 'x'):
            for i in range(len(unknowns)):
                if type(member) == Member and unknowns[i].value == member:
                    return i
                if type(member) == Force and unknowns[i].value == member and unknowns[i].kind == kind:
                    return i
            raise ValueError("object not in list")

        row_x = [0.0] * len(unknowns)
        row_y = [0.0] * len(unknowns)

        net_force_x = 0.0
        net_force_y = 0.0

        if virtual and self == target_joint:
            net_force_x = force.effective_x()
            net_force_y = force.effective_y()

        # 1. 遍历相连构件 (Members)
        for member in self.connected_members:
            if member.force_determined():
                # 若力已确定，计算其对节点的已知作用力 (注意 effective_x 和 effective_y 是方法)
                f = member.force_for_joint(self)
                net_force_x += f.effective_x()
                net_force_y += f.effective_y()
            else:
                # 若力未定，不能调用 force_for_joint (会报错)，需通过位置坐标计算离开当前节点的方向向量
                other_joint = member.joints[1] if member.joints[0] == self else member.joints[0]
                delta = other_joint.pos + (self.pos * -1)
                unit_dir = delta.get_direction()
                
                idx = find_index(member, unknowns)
                row_x[idx] = unit_dir.x
                row_y[idx] = unit_dir.y

        # 2. 遍历节点直接受到的外力或反力 (Forces)
        for f in self.forces:
            if virtual and not f.is_reaction:
                continue
            if f.x_solved():
                # 已知的 X 方向力，直接累加
                net_force_x += f.effective_x()
            else:
                # 未知的 X 方向力，系数记为 1
                row_x[find_index(f, unknowns, 'x')] = 1.0

            if f.y_solved():
                # 已知的 Y 方向力，直接累加
                net_force_y += f.effective_y()
            else:
                # 未知的 Y 方向力，系数记为 1
                row_y[find_index(f, unknowns, 'y')] = 1.0

        # 3. 构建方程: 系数 * 未知量 + 净已知力 = 0 => 系数 * 未知量 = -净已知力
        rhs_x = -net_force_x
        rhs_y = -net_force_y

        return [row_x, row_y], [rhs_x, rhs_y]
            




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

class Unknown:
    def __init__(self, value: Union[Member, Force], kind: Optional[str] = None):
        self.value: Union[Member, Force] = value

        self.kind: Optional[str] = kind #either x or y
        if type(value) == Force and self.kind is None:
            self.kind = 'x'

    def enumerate(self, value: Union[float, int]):
        if type(self.value) == Force:
            if self.kind == 'x':
                self.value.fx_solved = value
            else:
                self.value.fy_solved = value
        else:
            self.value.force = value

    def __repr__(self) -> str:
        return str(type(self.value))


class Truss:
    def __init__(self):
        self.joints: List[Joint] = []
        self.members: List[Member] = []
        self.forces: List[Force] = []  # 体系全部外力（荷载与反力）

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

    def add_external_force(self, force: Force) -> None:
        """登记一个体系外力，并让已有求解失效。"""
        if force not in self.forces:
            self.forces.append(force)
        self.unsolve()

    def add_joint(self, joint: Joint):
        self.joints.append(joint)
        joint.truss = self
        added_force = False
        for force in joint.forces:
            if force not in self.forces:
                self.forces.append(force)
                added_force = True
        if added_force:
            self.unsolve()

    def add_member(self, member: Member):
        self.members.append(member)
        self.unsolve()

    def _ensure_force_list(self) -> None:
        for joint in self.joints:
            for force in joint.forces:
                if force not in self.forces:
                    self.forces.append(force)

    def _joint_of_force(self, force: Force) -> Optional[Joint]:
        for joint in self.joints:
            if force in joint.forces:
                return joint
        return None

    def _solve_external_equilibrium(
        self,
        virtual: bool = False,
        target_joint: Optional[Joint] = None,
        unit_force: Optional[Force] = None,
    ) -> None:
        """节点法之前，用整体 ΣFx、ΣFy、ΣM 尽量确定未知外力。

        构件轴力是内力，不进入整体平衡。未知量多于 3 个时，只写入
        被这三条方程唯一确定的分量，其余仍留给节点法。
        与节点平衡一致：虚功工况去掉已知荷载，并在目标节点加上单位力。
        """
        self._ensure_force_list()

        cx = 0.0
        cy = 0.0
        if self.joints:
            cx = sum(joint.pos.x for joint in self.joints) / len(self.joints)
            cy = sum(joint.pos.y for joint in self.joints) / len(self.joints)

        sfx = 0.0
        sfy = 0.0
        sm = 0.0
        unknowns = []  # (kind, force, x, y)

        def add_known(fx: float, fy: float, x: float, y: float) -> None:
            nonlocal sfx, sfy, sm
            sfx += fx
            sfy += fy
            sm += (x - cx) * fy - (y - cy) * fx

        def consider(force: Force, x: float, y: float) -> None:
            if force.fx_prescribed is not None:
                fx_known = force.fx_prescribed
            elif force.fx_solved is not None:
                fx_known = force.fx_solved
            else:
                fx_known = 0.0
                unknowns.append(("fx", force, x, y))

            if force.fy_prescribed is not None:
                fy_known = force.fy_prescribed
            elif force.fy_solved is not None:
                fy_known = force.fy_solved
            else:
                fy_known = 0.0
                unknowns.append(("fy", force, x, y))

            add_known(fx_known, fy_known, x, y)

        for force in self.forces:
            if virtual and not force.is_reaction:
                continue
            joint = self._joint_of_force(force)
            if joint is None:
                continue
            consider(force, joint.pos.x, joint.pos.y)

        if virtual and target_joint is not None and unit_force is not None:
            consider(unit_force, target_joint.pos.x, target_joint.pos.y)

        n = len(unknowns)
        rows = [[0.0] * n for _ in range(3)]
        rhs = [-sfx, -sfy, -sm]
        for index, (kind, _force, x, y) in enumerate(unknowns):
            dx = x - cx
            dy = y - cy
            if kind == "fx":
                rows[0][index] = 1.0
                rows[2][index] = -dy
            else:
                rows[1][index] = 1.0
                rows[2][index] = dx

        char_len = 1.0
        for joint in self.joints:
            char_len = max(char_len, math.hypot(joint.pos.x - cx, joint.pos.y - cy))
        if unit_force is not None and target_joint is not None:
            char_len = max(char_len, math.hypot(target_joint.pos.x - cx, target_joint.pos.y - cy))
        rows[2] = [value / char_len for value in rows[2]]
        rhs[2] /= char_len

        solution = _unique_linear_solution(rows, rhs)
        for (kind, force, _x, _y), value in zip(unknowns, solution):
            if value is None:
                continue
            if kind == "fx":
                force.fx_solved = value
            else:
                force.fy_solved = value

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

        # 节点法之前先用整体平衡尽量确定未知外力，避免反力占满节点的两个未知量名额。
        self._solve_external_equilibrium(virtual, target_joint, force)


        unknowns: List[Unknown] = [] # this acts as the x vector of the matrix

        for i in self.members:
            unknowns.append(Unknown(i)) # this only has 1 unknown, that is its magnitude

        for i in self.forces:
            if not i.x_solved():
                unknowns.append(Unknown(i, 'x'))
            if not i.y_solved():
                unknowns.append(Unknown(i, 'y'))

        matrix: List[List[float]] = []
        rhs: List[float] = []
        for i in self.joints:
            cache, cache1 = i.solve(unknowns, virtual, target_joint, force) #this returns a list of lists and a list of int, representing rows in matrix
            for j in range(len(cache)):
                matrix.append(cache[j])
                rhs.append(cache1[j])

        #print(matrix)
        #print(rhs)

        ans = _unique_linear_solution(matrix, rhs)
        #print(unknowns)
        #print(ans)
        solved = True

        for i in range(len(ans)):
            if ans[i] is not None:
                unknowns[i].enumerate(ans[i])
            else:
                solved = False

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
