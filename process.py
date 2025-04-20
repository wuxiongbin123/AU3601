import cv2
import numpy as np
from skimage.morphology import skeletonize
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt

# 读取图片并转为灰度图
image = cv2.imread("ikun.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 二值化处理（阈值可调整）
_, binary = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY_INV)

# 可选：降噪（如中值滤波）
binary = cv2.medianBlur(binary, 3)

# 骨架化处理
skeleton = skeletonize(binary / 255)

points = np.argwhere(skeleton > 0)  # 格式为[y,x]，需翻转
points = [(x, y) for y, x in points]

img_h, img_w = binary.shape
print(f"h, w is {img_h, img_w}")
map_w, map_h = 10.0, 10.0  # 地图尺寸（改为浮点数）

mapped_points = []
for x, y in points:
    # 归一化到地图坐标（保留2位小数）
    map_x = round((x / img_w) * map_w, 2)
    map_y = round((y / img_h) * map_h, 2)
    mapped_points.append((10 - map_x, 10 - map_y)) #这里控制图像的颠倒

# 去重（可选）
mapped_points = list(dict.fromkeys(mapped_points))

# 对轨迹点进行排序
points_array = np.array(mapped_points)
if len(points_array) > 0:  # 确保有路径点
    dist_matrix = cdist(points_array, points_array)
    # 简单排序示例：从第一个点开始，按最近邻排序（贪心算法）
    sorted_indices = [0]
    remaining_indices = list(range(1, len(points_array)))
    
    while remaining_indices:
        last_index = sorted_indices[-1]
        distances = dist_matrix[last_index, remaining_indices]
        next_index = remaining_indices[np.argmin(distances)]
        sorted_indices.append(next_index)
        remaining_indices.remove(next_index)
    
    sorted_points = points_array[sorted_indices]
else:
    sorted_points = points_array

# sorted_points就是最终结果
sorted_points = sorted_points.tolist()[::2]
with open("trajectory.txt", "w") as f:
    for i in range(len(sorted_points)):
        f.writelines(str(sorted_points[i])+"\n")



points = np.array(sorted_points)
print("points[-1] is {}".format(points[-1]))

# 创建画布
plt.figure(figsize=(8, 8))
plt.title("Robot Path Visualization")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.grid(True)

# 设置坐标轴范围（根据你的地图0-10调整）
plt.xlim(0, 10)
plt.ylim(0, 10)

# 绘制轨迹线（按顺序连接点）
plt.plot(points[:, 0], points[:, 1], 'b-', linewidth=1, label='Path')  # 蓝色线条
plt.scatter(points[:, 0], points[:, 1], c='r', s=10, label='Points')  # 红色点

# 标记起点和终点
if len(points) > 0:
    plt.scatter(points[0, 0], points[0, 1], c='g', s=50, marker='o', label='Start')  # 绿色起点
    plt.scatter(points[-1, 0], points[-1, 1], c='k', s=50, marker='x', label='End')  # 黑色终点

# 添加图例和显示
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')  # 保证坐标轴比例一致
plt.show()