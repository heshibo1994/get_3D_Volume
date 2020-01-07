#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import numpy as np   ##科学计算库 
import scipy as sp   ##在numpy基础上实现的部分算法库
from scipy.optimize import leastsq  ##引入最小二乘法算法

marker_corner_newpath = sys.argv[1] 
Rr_path = sys.argv[2]
markerAndFarpath = sys.argv[3] 
point_Rtxtpath = sys.argv[4]
worldplypath = sys.argv[5]


# 提取二维码，角点，和原点数据
def getData(marker_corner_newpath,markerAndFarpath):
  f1 = open(marker_corner_newpath)
  marker_corner_newlines = f1.readlines()

  f2 = open(markerAndFarpath)
  markerAndFarlines = f2.readlines()
  Marker = markerAndFarlines[0].replace("\n","").split("-")
  Far = markerAndFarlines[1].replace("\n","")
  print("Mark",Marker)
  print("Far",Far)

  marker_point = []
  far_point = []
  for marker in Marker:
    for line in marker_corner_newlines:
      if marker in line:
        temp = [float(p) for p in line.split(" ")[3:]]
        if temp not in marker_point:
          marker_point.append(temp)


  for line in marker_corner_newlines:
      if Far in line:
        temp = [float(p) for p in line.split(" ")[3:]]
        if temp not in far_point:
          far_point.append(temp)
  print("far",far_point)
  return (marker_point,far_point)
 
# 获取X，Y
def getR(marker_point):
  Rr =    [[2.15966,-0.168739,0.0937677],[-0.168739,-1.13852,1.83758],[-0.0937677,-1.83758,-1.14713]]
  X = []
  Y = []
  for i in range(len(marker_point)):
    l = [ 0,0 ,0]
    l[0] = marker_point[i][0]*Rr[0][0]+marker_point[i][1]*Rr[0][1]+marker_point[i][2]*Rr[0][2]
    l[1] = marker_point[i][0]*Rr[1][0]+marker_point[i][1]*Rr[1][1]+marker_point[i][2]*Rr[1][2]
    l[2] = marker_point[i][0]*Rr[2][0]+marker_point[i][1]*Rr[2][1]+marker_point[i][2]*Rr[2][2]
    X.append(l[0])
    Y.append(l[1])
  return (X,Y)

# 获取原点
def getFar(far_point):    
  X_farpoint = []
  Y_farpoint = []
  Rr =    [[2.15966,-0.168739,0.0937677],[-0.168739,-1.13852,1.83758],[-0.0937677,-1.83758,-1.14713]]
  for j in range(len(far_point)):
    l = [ 0,0 ,0]
    l[0] = round(far_point[j][0]*Rr[0][0]+far_point[j][1]*Rr[0][1]+far_point[j][2]*Rr[0][2],2)
    l[1] = round(far_point[j][0]*Rr[1][0]+far_point[j][1]*Rr[1][1]+far_point[j][2]*Rr[1][2],2)
    l[2] = round(far_point[j][0]*Rr[2][0]+far_point[j][1]*Rr[2][1]+far_point[j][2]*Rr[2][2],2)
    X_farpoint.append(l[0])
    Y_farpoint.append(l[1])
  print("X_far",X_farpoint)
  x_farpoint = sum(X_farpoint)/len(X_farpoint)
  y_farpoint = sum(Y_farpoint)/len(Y_farpoint)
  return (x_farpoint,y_farpoint)

##需要拟合的函数func :指定函数的形状
def func(p,x):
    k,b=p
    return k*x+b
##偏差函数：x,y都是列表:这里的x,y更上面的Xi,Yi中是一一对应的
def error(p,x,y):
    return func(p,x)-y
def getleastsq(X,Y):
  Xi=np.array(X)
  Yi=np.array(Y)
  p0=[1,20]
  #把error函数中除了p0以外的参数打包到args中(使用要求)
  Para=leastsq(error,p0,args=(Xi,Yi))
  #读取结果
  k,b=Para[0]
  print(k,b)
  return (k,b)


# 获取真实世界坐标下的点云
def getCloud(k,b,x_farpoint,y_farpoint,point_Rtxtpath,worldplypath):
  cos = (1/(k**2+1))**0.5
  sin = (k**2/(k**2+1))**0.5
  x = abs(b)/(k**2+1)**0.5
  y = abs((y_farpoint*k+x_farpoint)/k)/(1+1/k**2)**0.5


  f = open(point_Rtxtpath)
  lines = f.readlines()

  fw1 = open(worldplypath,"w")
  fw1.write("ply\nformat ascii 1.0\ncomment author: Greg Turk\ncomment object: another cube\n")
  fw1.write("element vertex " + lines[0] )
  fw1.write("property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n")
  fw1.write("end_header\n")

  for i in range(1,len(lines)):
    point = [float(p) for p in lines[i].split(" ")[:3]]
    r = lines[i].split(" ")[3]
    g = lines[i].split(" ")[4]
    b = lines[i].split(" ")[5]
    l = [0,0,0]
    l[0] = cos*point[0]+sin*point[1]+x
    l[1] = -sin*point[0]+cos*point[1]+y
    l[2] = point[2]+4.7
    fw1.write(str(l[0])+" "+str(l[1])+" "+str(l[2])+" "+str(r)+" "+str(g)+" "+str(b))

marker_point,far_point = getData(marker_corner_newpath ,markerAndFarpath)
X,Y = getR(marker_point)
x_farpoint,y_farpoint = getFar(far_point)
k,b = getleastsq(X,Y)
getCloud(k,b,x_farpoint,y_farpoint,point_Rtxtpath,worldplypath)
