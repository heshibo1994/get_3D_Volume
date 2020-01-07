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

ofstream outfile_marker_corner;

void detect(cv::Mat image,Mat cameraMatrix,Mat distCoeffs,string name,string Marker_corner){
    //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);//检测靶标
    cout<<name<<endl;
    outfile_marker_corner.open(Marker_corner,ios::app);
    for (int i = 0;i<corners.size();i++){
        outfile_marker_corner<<name<<" "<<ids[i]<<
        " "<<corners[i][0].x<<" "<<corners[i][0].y<<
        " "<<corners[i][1].x<<" "<<corners[i][1].y<<
        " "<<corners[i][2].x<<" "<<corners[i][2].y<<
        " "<<corners[i][3].x<<" "<<corners[i][3].y<<endl;
                cout<<name<<" "<<ids[i]<<
        " "<<corners[i][0].x<<" "<<corners[i][0].y<<
        " "<<corners[i][1].x<<" "<<corners[i][1].y<<
        " "<<corners[i][2].x<<" "<<corners[i][2].y<<
        " "<<corners[i][3].x<<" "<<corners[i][3].y<<endl;
    }
    outfile_marker_corner.close();
}


int main(int argc, char *argv[])

{
    cv::FileStorage fs(argv[1],cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+"count not open file :"+argv[1]);
    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"]>>cameraMatrix;
    fs["distortion_coefficients"]>>distCoeffs;

    DIR* dir = opendir(argv[2]);//打开指定目录 
    string Marker_corner_path = string(argv[3]); 

    dirent* p = NULL;//定义遍历指针  

    while((p = readdir(dir)) != NULL)//开始逐个遍历  
    {  
        //这里需要注意，linux平台下一个目录中有"."和".."隐藏文件，需要过滤掉  
        if(p->d_name[0] != '.')//d_name是一个char数组，存放当前遍历到的文件名  
        {  
            string name = string(p->d_name); 
            cv::Mat image = imread(string(argv[2])+"//"+name);
            detect (image,cameraMatrix,distCoeffs,name,Marker_corner_path);
            //cv::waitKey(0);
        }  
    }  
    closedir(dir);//关闭指定目录

    return 0;
}
