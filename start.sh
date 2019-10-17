#!/bin/bash

work_space="//home//lab606//auto_exe//Data//data4//"
echo "开始检测图像中所有的二维码"
opencv_path=${work_space}opencv.txt
detect_path=${work_space}detect.txt
cd detect_opencv
cd build
./detect ${work_space}mynteye_new_1.yml ${work_space}images $opencv_path $detect_path
sleep 1s
echo "每帧中二维码的位姿:            $opencv_path"
echo "每帧中二维码的底部坐标值:       $detect_path"
echo "结束检测"
echo ""
echo ""


echo "开始提取结算尺度和平面的数据"
K_path=${work_space}K.txt
plane_path=${work_space}plane.txt
colmap_path=${work_space}colmap.txt
points3D_path=${work_space}points3D.txt
cd ..
cd ..
python get_K.py $colmap_path $opencv_path $K_path
python get_plane.py $points3D_path $detect_path $colmap_path ${work_space}relative2d_3d.txt $plane_path
sleep 1s
echo "结算尺度的数据:               $K_path"
echo "结算平面的数据:               $detect_path"
echo "结束提取"
echo ""
echo ""

echo "开始优化K值和平面方程"
kabc_path=${work_space}kabc.txt
cd Ceres
cd build
./kabc $K_path $plane_path $kabc_path
sleep 1s
echo "最优k值和平面方程:            $kabc_path"
echo "结束优化"
echo ""
echo ""


echo "开始增加平面方程"
cd ..
cd ..
cd pcl
cd build
./add_plane //home//lab606//auto_exe//data4//dense//fused.ply //home//lab606//auto_exe//data4//dense//fused_plane.ply -10 10 -1 5


