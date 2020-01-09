#!/usr/bin/python
# -*- coding: UTF-8 -*-
import sys

marker_corner_path = sys.argv[1] #二维码4个角点坐标
image_path = sys.argv[2] #每张图像上2d点和3d点对应关系
point3D_path = sys.argv[3] # 每个3d点的序号
plane_path = sys.argv[4] # 接水平方程所需3D点
xyzrgb_path = sys.argv[5] # 每个点的xyz和rgb
marker_corner_newpath = sys.argv[6] #二维码处理后的信息
fw = open(plane_path,'w')
fxyzrgb = open(xyzrgb_path,'w')
fmarker_corner_new = open(marker_corner_newpath,'w')
f_marker = open(marker_corner_path)
a = [] # 二维码角点检测结果



output3Dpoints = [] # 确定平面的3d角点
lines_marker = f_marker.readlines()
for i in range(len(lines_marker)):
	a.append([lines_marker[i].split(" ")[0]] +  [ float(l) for l in lines_marker[i].split(" ")[2:]]+[lines_marker[i].split(" ")[1]] )


f_point3D = open(point3D_path)
lines_point3D = f_point3D.readlines()
fxyzrgb.write(str(len(lines_point3D)-3)+'\n')
point3D = {}
for p in range(3,len(lines_point3D)):
	point = lines_point3D[p].split(" ")
	point3D[int(point[0])] = [point[1],point[2],point[3]]
	fxyzrgb.write(" ".join(lines_point3D[p].split(" ")[1:7])+"\n")



marker_corner_new = []
f_image = open(image_path)
lines_image = f_image.readlines()
for i in range(len(a)):
	for j in range(len(lines_image)):
		if a[i][0] in lines_image[j]: 
			l = lines_image[j+1].split(" ")
			temp = [  [float(l[k]),float(l[k+1]),int(l[k+2])] for k in range(len(l)) if k%3 ==0]

			for n in [1,3,5,7]:
				near_point = [0,0]
				index = -1
				min_distance = 100000
				for m in range(len(temp)):
					distance = ((a[i][n]-temp[m][0])**2+ (a[i][n+1]-temp[m][1])**2)  
					if distance < min_distance:
						min_distance = distance
						near_point = [temp[m][0],temp[m][1]]
						index = temp[m][2]
				print("min_dis,",min_distance)
				print(near_point)
				print("index,",index)
				if index != -1 and min_distance<10:					
					print(a[i][0],a[i][n],a[i][n+1],near_point,index,point3D[index])
					output3Dpoints.append(point3D[index])
					marker_corner_new.append([a[i][0],a[i][-1],str(n/2),point3D[index][0],point3D[index][1],point3D[index][2]])	

			continue 
print(marker_corner_new)
for m in range(len(marker_corner_new)):
	fmarker_corner_new.write(" ".join(marker_corner_new[m])+'\n')

temp = []
for point in output3Dpoints:
	if point not in temp:
		print(point)
		temp.append(point)
fw.write(str(len(temp))+"\n")
for i in temp:
	fw.write(" ".join(i)+"\n")	

