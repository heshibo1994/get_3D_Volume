#!/usr/bin/python
# -*- coding: UTF-8 -*-
l1 = [-1.25578, 0.832423]
l2 = [-1.222, 1.2289]


def f(p1,p2):
  return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**0.5

print(58/f(l1,l2))

