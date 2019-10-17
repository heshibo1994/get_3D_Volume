#!/usr/bin/python
# -*- coding: UTF-8 -*-
import sys
class Soulution:

	# 读取整个地图所有3D点的坐标
	def read3D_point(self,path):
		f = open(path)
		lines = f.readlines()[3:]
		ans = {}
		for line in lines:
			l = line.split(" ")
			ans[int(l[0])] = [float(i) for i in l[1:4]]
		return ans

    # 获取2d点和3点之间的对应关系[x,y,No]
	def get2d_3d(self,path,image_name):
		f = open(path)
		lines = f.readlines()
		temp =""
		for i in range(len(lines)):
			if image_name in lines[i]:
				temp = lines[i+1]
				break
		k = temp.split(" ")
		length = len(temp.split(" "))
		ans = [[float(k[i])] + [float(k[i+1])] + [int(k[i+2])] for i in range(length) if i%3 ==0]
		return ans



	def getNearPoint(self,point,relative2d_3d,coor3d):
		ans = [0,0]
		k = -1
		min_distance = 100
		for i in range(len(relative2d_3d)):
			distance = (point[0] - relative2d_3d[i][0])**2 + (point[1] - relative2d_3d[i][1])**2
			if distance < min_distance:
				min_distance = distance
				ans = [relative2d_3d[i][0],relative2d_3d[i][1]]
				k = relative2d_3d[i][2]
		if k!=-1:
			#print(str(point[0])+" "+ str(point[1])+
			#"   nearest:"+str(ans[0])+"  "+str(ans[1])+"   "+str(coor3d[k])) # ans
			return " ".join(str(i) for i in coor3d[k])+"\n"
		else:
			#print(str(point[0])+" "+ str(point[1])+
			#"   nearest:"+str(ans[0])+"  "+str(ans[1])+"No_3Dpoint") # ans
			return ""

	# 2d_3d对应关系写进txt
	def writeTxt(self,path,relative2d_3d,coor3d):
		f = open(path,'w')
		for i in range(len(relative2d_3d)):
			if relative2d_3d[i][2] != -1:
				coor3d_PerPoint = coor3d[relative2d_3d[i][2]]
				s = relative2d_3d[i]+coor3d_PerPoint

			else:
				s = relative2d_3d[i]
			#print(s)
			f.write("  ".join([str(i) for i in s]))
			f.write("\n")


	def detectMark(self,path):
		f = open(path)
		a = []
		lines = f.readlines()
		for i in lines:
			if '[' in i:
				name = i.split(" ")[0]
				point1 = [int(i[i.index("[")+1:i.index(",")]), int( i.split(" ")[2][:-1] ) ]
				point2 = [int(i.split(" ")[3][1:-1]), int( i.split(" ")[4][:-2])]
				a.append([name,point1,point2])
		return a




s = Soulution()


coor3d_path = sys.argv[1] # colmap
marker_path = sys.argv[2] # opencv
colmap_path = sys.argv[3] # colmap




relative2d_3d_path = sys.argv[4]
plane_path = sys.argv [5]




coor3d = s.read3D_point(coor3d_path)
marker = s.detectMark(marker_path)

f = open(plane_path,'w')
for i in range(len(marker)):
	relative2d_3d = s.get2d_3d(colmap_path,marker[i][0])
	a = s.getNearPoint(marker[i][1],relative2d_3d,coor3d)
	f.write(a)
	a = s.getNearPoint(marker[i][2],relative2d_3d,coor3d)
	f.write(a)
s.writeTxt(relative2d_3d_path,relative2d_3d,coor3d)

