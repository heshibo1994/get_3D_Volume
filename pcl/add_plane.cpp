#include <iostream> 
#include <unistd.h>
#include <string>
#include <math.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>
#include <vector>

using namespace std;


void get_plane(double p1[3],double p2[3],double abcd[4],double A,double B,double C){
    double a = p1[0]-p2[0];
    double b = p1[1]-p2[1];
    double c = p1[2]-p2[2];
    abcd[2] = 1;
    abcd[1] = (C*a-A*c)/(A*b-B*a);
    abcd[0] = -(c+b*abcd[1])/(a);
    abcd[3] = -(p1[0]*abcd[0]+p1[1]*abcd[1]+p1[2]*abcd[2]);
}



int main(int argc, char * argv[])
{



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        pause();
        return -1;
    }
    
    //计算点的数量
    std::cout << " 原始数量: "<< cloud->width * cloud->height<<std::endl;

    //添加水平面方程放在cloud_plane(argv[2])中
    //已知水平面方程，添加Ax+By+Cz+D=0,通过添加点来显示水平面，提前要确定号A,B,C的数值D=1，和xy平面的值域，步长

    //滤波设置。将滤波后的结果放在cloud_filter(argv[4])中
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>sor;
    sor.setInputCloud (cloud);                           //设置待滤波的点云
    sor.setMeanK (50);                               //设置在进行统计时考虑的临近点个数
    sor.setStddevMulThresh (0.1);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
    sor.filter (*cloud_filter);                    //滤波结果存储到cloud_filtered
    std::cout << " 滤波数量: "<< cloud_filter->width * cloud_filter->height<<std::endl;
    pcl::io::savePLYFileBinary(argv[2],*cloud_filter);


    //获取切割平面方程
    //经过两个点，且垂直与水平面的切割面，输入两个点的坐标和水平面方程参数
    //将切割后的结果放在cloud_cut(argv[3])后
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0;i<cloud_filter->points.size();i++){
        // float d0 =  A*cloud->points[i].x+B*cloud->points[i].y+C*cloud->points[i].z+1;//d0>=0
        if (cloud_filter->points[i].y>0.5 && cloud_filter->points[i].y<13 && cloud_filter->points[i].x>-10 && cloud_filter->points[i].x<20 ){
            pcl::PointXYZRGB point ;
            point.x = cloud_filter->points[i].x ;
            point.y = cloud_filter->points[i].y ;
            point.z = cloud_filter->points[i].z ;
            point.r = cloud_filter->points[i].r ;
            point.g = cloud_filter->points[i].g ;
            point.b = cloud_filter->points[i].b ;
            cloud_cut->push_back(point);
    }
    }
    pcl::io::savePLYFileBinary(argv[3],*cloud_cut);
    

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);

    // for (float x = x_min ;x<x_max;x = x+x_delta){
    //     for (float y = y_min ;y<y_max;y = y+x_delta){
    //         float z = -(1 + A*x + B*y)/C;
    //         pcl::PointXYZRGB point ;
    //         point.x = x ;
    //         point.y = y ;
    //         point.z = z ;
    //         point.r = 255 ;
    //         point.g = 0 ;
    //         point.b = 0 ;
    //         cloud_cut->push_back(point);
    //     }
    // }
    // pcl::io::savePLYFileBinary(argv[4],*cloud_cut);



    //获得点的坐标文本
    ofstream outfile_3d;
    outfile_3d.open(argv[4]);
    for (size_t i = 0;i<cloud_cut->points.size();i++){
        pcl::PointXYZRGB point ;
        point.x = cloud_cut->points[i].x ;
        point.y = cloud_cut->points[i].y ;
        point.z = cloud_cut->points[i].z ;
        outfile_3d<<point.x<<" "<<point.y<<" "<<point.z<<endl;
    }  
    //垂直防止求水平面
    // double A = 0.0571989;
    // double B = -0.0187938 ;
    // double C = -0.046316;
    // float A0 = 0.0571989;
    // float B0 = -0.0187938 ;
    // float C0 = -0.046316;
    // float pa1 = -1.9577;
    // float pa2 = -0.795895;
    // float pa3 = 19.5298;
    // float pb1 = -0.124661;
    // float pb2 =  2.20117;
    // float pb3 = 20.4688;
    // float point1 = pa1-pb1;
    // float point2 = pa2-pb2;
    // float point3 = pa3-pb3;
    // float plane1 = 0.0571989;
    // float plane2 = -0.0187938;
    // float plane3 = -0.046316;
    // float k1 = point2*plane3-point3*plane2;
    // float k2 = point3*plane1-point1*plane3;
    // float k3 = point1*plane2-point2*plane1;
    // double d = -k1*pa1-k2*pa2-k3*pa3;
    // double z = -(d+k1*x + k2*y)/k3;
    return 0; 
}
