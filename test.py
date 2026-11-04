import numpy as np
import random
from matrix_solver import solve_matrix

def test_with_numpy(size=10):
    # 构造随机的 n x n 浮点数矩阵 M 和长度为 n 的真实解向量 x_true
    M = np.random.uniform(-10, 10, (size, size))
    x_true = np.random.uniform(-10, 10, size)
    
    # 通过矩阵乘法计算右侧常数向量 b = Mx
    b = np.dot(M, x_true)
    
    # 将 numpy 数组转为嵌套列表后传入自定义函数求解[cite: 3]
    x_found = solve_matrix(M.tolist(), b.tolist())
    
    # 打印结果进行比对
    print("预设的真实解 x_true :", np.round(x_true, 6))
    print("解算出的结果 x_found:", np.round(x_found, 6))
    
    # 计算并输出最大误差
    error = np.max(np.abs(x_true - x_found))
    print(f"最大计算误差: {error:.2e}")

if __name__ == "__main__":
    np.random.seed(random.randint(0,10000)) # 固定随机种子以便复现
    test_with_numpy(100)