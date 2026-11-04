from __future__ import annotations
from typing import Sequence, Union, Tuple, List, Optional
from copy import deepcopy

EQ_TOL = 1e-8

def eq(a: float, b: float, tol: float = EQ_TOL):
    if abs(a - b) < tol:
        return True
    return False

def pivot_index(row: List[float]) -> Optional[int]:
    for i in range(len(row)):
        if not eq(row[i], 0):
            return i
    return None

def pivot_index_order(row: List[float]) -> int:
    value = pivot_index(row)
    if value is None:
        return 99999999999999
    return value

def all_zero(row: List[float]) -> bool:
    for i in row:
        if not eq(i, 0):
            return False
    return True

def check_elimination(matrix: List[List[float]],
                      row: int) -> Optional[int]: # return another row to eliminate
    col = pivot_index(matrix[row])
    if col is None:
        return None

    for i in range(len(matrix)):
        if i == row:
            continue
        if not eq(matrix[i][col], 0):
            return i

    return None

def eliminate(matrix: List[List[float]],
              rows: tuple[int, int]) -> None:
    #assume check_elimination has passed
    if pivot_index(matrix[rows[0]]) > pivot_index(matrix[rows[1]]):
        rows = (rows[1], rows[0])

    # matrix[rows[0]] = matrix[rows[0]] - matrix[rows[1]]

    bigger_pivot = pivot_index(matrix[rows[1]])
    mult = (matrix[rows[1]][bigger_pivot], matrix[rows[0]][bigger_pivot])
    mult = (1, mult[1]/mult[0])

    for i in range(len(matrix[0])):
        matrix[rows[0]][i] = matrix[rows[0]][i] * mult[0] - matrix[rows[1]][i] * mult[1]

    return None

def order(matrix: List[List[float]]) -> None:
    matrix.sort(key = pivot_index_order)
    
def is_solved(matrix: List[List[float]]):
    for i in range(len(matrix)):
        if check_elimination(matrix, i) is not None:
            return False
    return True

def solve_matrix(
    matrix: List[List[float]],
    rhs: List[float]
) -> List[Optional[float]]:

    matrix = deepcopy(matrix)
    rhs = deepcopy(rhs)
    
    m = len(matrix)
    n = len(matrix[0])

    ans = [None] * n

    for i in matrix:
        if len(i) != n:
            raise ValueError("bad matrix shape")

    #turn into augmented matrix
    for i in range(len(matrix)):
        matrix[i].append(rhs[i])

    while not is_solved(matrix):
        for i in range(len(matrix)):
            row = check_elimination(matrix, i)
            if row is not None:
                eliminate(matrix, (i, row))

    order(matrix)

    #figure out answer
    for i in range(len(matrix)):
        rhs[i] = matrix[i][-1]
        del matrix[i][-1]


    for i in range(len(matrix)):
        pivot = pivot_index(matrix[i])
        if pivot is not None:
            if all_zero(matrix[i][pivot+1:]):
                ans[pivot] = rhs[i]/matrix[i][pivot]
        else:
            if not eq(rhs[i], 0):
                raise ValueError(
                    "System cannot be in equilibrium. "
                    "Not enough members to balance the loads."
                )

    return ans
    