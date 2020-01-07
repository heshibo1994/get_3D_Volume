#!/usr/bin/pyhton
#-*- coding: UTF-8 -*-
# from scipy.spatial import Delaunay
# import numpy as np
# import matplotlib.pyplot as plt
# import math
# class Point():
#     def __init__(self,point3d,x,y,z,d):
#         self.point3d = point3d
#         self.x = x
#         self.y = y
#         self.z = z
#         self.d = d

# #  获取3D坐标点
# class Solution():
#     def get_3D(self, path):
#         f = open(path)
#         lines = f.readlines()
#         points3D = []
#         for line in lines[3:]:
#             points3D.append([float(i) for i in line.split(" ")])
#         return points3D

#     # 获取空间点在水平面的投影和距离
#     def get_projection2D(self, points3D, A, B, C):
#         projection2D = []
#         point_d_map = {}
#         for p in points3D:
#             xp = ((B ** 2 + C ** 2) * p[0] - A * (B * p[1] + C * p[2] + 1)) / (A ** 2 + B ** 2 + C ** 2)
#             yp = ((A ** 2 + C ** 2) * p[1] - B * (A * p[0] + C * p[2] + 1)) / (A ** 2 + B ** 2 + C ** 2)
#             zp = ((A ** 2 + B ** 2) * p[2] - C * (A * p[0] + B * p[1] + 1)) / (A ** 2 + B ** 2 + C ** 2)
#             d = abs(A*p[0]+B*p[1]+C*p[2]+1)/math.sqrt(A**2+B**2+C**2)
#             point_d_map[xp*1000+yp] = d
#             projection2D.append(Point(p,xp, yp, zp,d))
#         points = np.zeros((len(projection2D), 2))
#         points[:, 0] = np.array([p.x for p in projection2D])
#         points[:, 1] = np.array([p.y for p in projection2D])
#         # print(points[:,:])
#         # plt.scatter(points[:, 0], points[:, 1],linewidths = 0.00001)
#         # plt.show()
#         tri = Delaunay(points)

#         V_total = 0.0
#         V_list = []
#         f = open("v.txt", 'w')
   
#         for point in points[tri.simplices]:
#             d = point_d_map[point[0][0]*1000+point[0][1]]+point_d_map[point[1][0]*1000+point[1][1]]+point_d_map[point[2][0]*1000+point[2][1]]
#             d_average = d/3
#             S = (point[0][0] * point[1][1] + point[1][0] * point[2][1] + point[2][0] * point[0][1] - point[0][0] * point[2][1] - point[1][0] * point[0][1] - point[2][0] * point[1][1]) *0.5
#             V = S * d_average
#             V_list.append(V)

#         l = len(V_list)
#         V_list.sort()
#         up1_4 = V_list[int(3*l/4)]
#         down1_4 = V_list[int(l/4)]
#         IQR =up1_4-down1_4 
#         for i in range(len(V_list)):
#             if V_list[i]<up1_4+10*IQR and V_list[i]>down1_4-1.5*IQR:
#                 V_total += V_list[i]
#                 f.write(str(V_list[i])+'\n')
#                 print(i,V_list[i],V_total)
#             else:
#                 print(i,"______________________________________________________________________",V_list[i])
#         print(V_total/(9.51896**3))
#         return projection2D



# A = -0.00944687
# B = -0.0904767 
# C = -0.123346
# s = Solution()
# points3D = s.get_3D("3d.txt")
# projection2D = s.get_projection2D(points3D, A, B, C)


from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt

# Triangle Settings
width = 200
height = 40
pointNumber = 1000
points = np.zeros((pointNumber, 2))
points[:, 0] = np.random.randint(0, width, pointNumber)
points[:, 1] = np.random.randint(0, height, pointNumber)

# Use scipy.spatial.Delaunay for Triangulation
tri = Delaunay(points)

# Plot Delaunay triangle with color filled
center = np.sum(points[tri.simplices], axis=1)/3.0
color = np.array([(x - width/2)**2 + (y - height/2)**2 for x, y in center])
plt.figure(figsize=(7, 3))
plt.tripcolor(points[:, 0], points[:, 1], tri.simplices.copy(), facecolors=color, edgecolors='k')


# Delete ticks, axis and background
plt.tick_params(labelbottom='off', labelleft='off', left='off', right='off',
                bottom='off', top='off')
ax = plt.gca()
ax.spines['right'].set_color('none')
ax.spines['bottom'].set_color('none')
ax.spines['left'].set_color('none')
ax.spines['top'].set_color('none')

# Save picture
plt.savefig('Delaunay.png', transparent=True, dpi=600)
