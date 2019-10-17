#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 获取每一帧图像的相机pose，求scale
import numpy as np
import math
import sys
# 整理colmap中的相机位姿
f = open(sys.argv[1])
l = f.readlines()
pose = [i for i in l if ".jpg" in i]
pose.sort(key = lambda x:int(x.split(" ")[-1][:-5]))
data = [i.split(" ")[5:8]+[i.split(" ")[-1][:i.split(" ")[-1].find(".")]] for i in pose]
x = []
y = []
z = []
colmap = {}
for i in data:
	colmap[int(i[3])] = [float(i[0]),float(i[1]),float(i[2])]
	x.append(float(i[0]))
	y.append(float(i[1]))
	z.append(float(i[2]))


# 整理opencv中的相机位姿
f = open(sys.argv[2])
l = f.readlines()
t = [i for i in l if "t" in i ]
t.sort(key = lambda x:int(x.split(" ")[0][:-4]))
x=[]
y=[]
z=[]

opencv = {}
for i in t:
	opencv[int(i.split(" ")[0][:-4])] = [float(i.split(" ")[2]),float(i.split(" ")[3]),float(i.split(" ")[4])]
	x.append(float(i.split(" ")[2]))
	y.append(float(i.split(" ")[3]))
	z.append(float(i.split(" ")[4]))


def fun(a,b):
	map = []
	for i in b.keys():
		map.append([i,a[i],b[i]]) 
	return map

def distance(a,b):
	return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)


map = fun(colmap,opencv)
f = open(sys.argv[3],'w')
for i in range(len(map)):
	s = " ".join(str(i) for i in (map[i][1]+map[i][2]))
	f.write(s)
	f.write('\n')
	#print(s)

# def k(x,a,b):
# 	print(distance(map[x][1],map[a][1])/distance(map[x][2],map[a][2]))
# 	print(distance(map[x][1],map[b][1])/distance(map[x][2],map[b][2]))
	
# 	print(distance(map[x][1],map[a][1])/distance(map[x][1],map[b][1]))
# 	print(distance(map[x][2],map[a][2])/distance(map[x][2],map[b][2]))


# k(30,20,50)
