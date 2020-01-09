#!/bin/bash

work_space="//home//lab606//auto_exe//TEST//Data2//"


# 中间文件的路径
# 三位重建相机内参
calibration=${work_space}//calibration.yml
# 三位重建输入图像集
imagesFile=${work_space}//images
# 二维码角点检测结果
marker_corner=${work_space}//marker_corner.txt
# 三位重建后每一张图像的信息
images=${work_space}//images.txt
# 三位重建后每一个点的信息
points3D=${work_space}//points3D.txt
# 提取平面所需要的数据  x，y，z
plane=${work_space}//plane.txt
# 点的信息数据  x，y，z，r,g,b
xyzrgb=${work_space}//xyzrgb.txt
# 处理后的二维码信息
marker_corner_new=${work_space}//marker_corner_new.txt
# 描述平面的方程数据和旋转矩阵a,b,c,r
abc=${work_space}//abc.txt
# 参考坐标系下，旋转到与水平面平行的ply点云文件
point_Rply=${work_space}//point_R.ply
# 参考坐标系下，旋转到与水平面平行的txt点云文件
point_Rtxt=${work_space}//point_R.txt
# 参考坐标系下，决定世界坐标系所需要的二维码和角点和原点信息
setting=${work_space}//setting.yml
# 真实世界坐标系下的xyzrgb点云文件
worldply=${work_space}//worldply.ply
# 真实世界坐标系下的xyzrgb立群点滤波点云文件
cloud_filter=${work_space}//cloud_filter.ply
# 真实世界坐标系下的xyzrgb平面切割后点云文件
cloud_cut=${work_space}//cloud_cut.ply
# 完全干净的点云模型，拟合的数据
mba=${work_space}//mba.ply



markerAndFar=${work_space}//markerAndFar.txt
cloud_resize_path=${work_space}//cloud_resize.ply
world_path=${work_space}//world_resize.ply
echo "检测图像中所有二维码角点"
# cd C++/build
# ./detect $calibration $imagesFile $marker_corner
# sleep 1s

# echo "对2d点和3d点进行映射"
# cd ..
# cd ..
# python process_point.py $marker_corner $images $points3D $plane $xyzrgb $marker_corner_new
# sleep 1s

# echo "优化平面方程"
# cd C++/build
# ./abc $plane $abc $xyzrgb $point_Rply $point_Rtxt

# cd ..
# cd ..
python process_pointcloud.py $marker_corner_new $abc $markerAndFar $point_Rtxt $worldply

cd C++/build
./pointcloud $setting $worldply $cloud_filter $cloud_cut  $mba $cloud_resize_path $world_path

