//#include <QCoreApplication>
#include <dirent.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv.h>
#include <iostream>
#include <highgui.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <string>

using namespace cv;
using namespace std;




ofstream outfile_opencv;
ofstream outfile_detect;
Eigen::Isometry3d detect(cv::Mat image,Mat cameraMatrix,Mat distCoeffs,string name,string opencv_path,string detect_path){
    //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);//检测靶标

    outfile_detect.open(detect_path,ios::app);
    for (int i = 0;i<corners.size();i++){
        outfile_detect<<name<<" "<<corners[i][2]<<" "<<corners[i][3]<<endl;
        //cout<<name<<" "<<corners[i][2]<<" "<<corners[i][3]<<endl;
    }

    

    // cv::Mat markerImage_40,markerImage_41,markerImage_42,markerImage_43;
    // cv::aruco::drawMarker(dictionary,40,750,markerImage_40,1);
    // cv::imshow("markerImage_40", markerImage_40);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    outfile_opencv.open(opencv_path,ios::app);
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);//绘制检测到的靶标的框
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.2, cameraMatrix, distCoeffs, rvecs, tvecs);//求解旋转矩阵rvecs和平移矩阵tvecs
        for(int i =0 ;i<ids.size();i++){
            if (ids[i] == 40){
                outfile_opencv<<name<<" t: "<<tvecs[i][0]<<" "<<tvecs[i][1]<<" "<<tvecs[i][2]<<endl;
                //cout<<name<<" t: "<<tvecs[i][0]<<" "<<tvecs[i][1]<<" "<<tvecs[i][2]<<endl;
            }
        }
    
        cv::Mat Rotate;
        Rodrigues(rvecs[0],Rotate);
        Eigen::Matrix<double,3,3> Rotate_matrix;
        Eigen::Vector3d Translate_matrix;
        cv2eigen(Rotate,Rotate_matrix);
        cv2eigen(tvecs[0],Translate_matrix);
        //cout<<"Rotate_matrix :"<<Rotate_matrix<<endl;
        //cout<<"Translate_matrix :"<<Translate_matrix<<endl;

        T.rotate(Rotate_matrix);
        T.pretranslate(Translate_matrix);
        // draw axis for each marker
        for(int i=0; i<ids.size(); i++)
            cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    }
    outfile_opencv.close();
    outfile_detect.close();

    cv::imshow("out", image);
    //cv::waitKey();
    return T;
}


int main(int argc, char *argv[])

{
 
    cv::FileStorage fs(argv[1],cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+"count not open file :"+argv[1]);
    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"]>>cameraMatrix;
    fs["distortion_coefficients"]>>distCoeffs;


    DIR* dir = opendir(argv[2]);//打开指定目录 
    string opencv_path = string(argv[3]); 
    string detect_path = string(argv[4]); 
    dirent* p = NULL;//定义遍历指针  
    Eigen::Isometry3d cam2marker;
    while((p = readdir(dir)) != NULL)//开始逐个遍历  
    {  
        //这里需要注意，linux平台下一个目录中有"."和".."隐藏文件，需要过滤掉  
        if(p->d_name[0] != '.')//d_name是一个char数组，存放当前遍历到的文件名  
        {  
            string name = string(p->d_name);  
            cv::Mat image = imread(string(argv[2])+"//"+name);
            cam2marker = detect(image,cameraMatrix,distCoeffs,name,opencv_path,detect_path);
            //cout<<"相机在marker下的位姿：\n"<<cam2marker.matrix()<<endl;
            //cv::waitKey(0);
        }  
    }  
    closedir(dir);//关闭指定目录

    return 0;
}
