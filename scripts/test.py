import numpy as np

def barycentric_coordinates(points, triangle):
    A = np.column_stack((triangle[:, :2], np.ones(3)))
    b = np.hstack((points, np.ones((len(points), 1))))
    x = np.linalg.solve(A, b.T)
    # return x[:3].T
    return b

triangle = np.array([[0, 0], [0, 1], [1, 0]])  # 三角形的三个顶点坐标
points = np.array([[0.5, 0.3], [0.2, 0.8], [0.7, 0.5]])  # 需要计算的点坐标
barycentric_coords = barycentric_coordinates(points, triangle)  # 计算重心坐标

print(barycentric_coords)
