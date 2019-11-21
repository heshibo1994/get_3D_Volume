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
    //载入点云模型
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        pause();
        return -1;
    }
    
    //计算点的数量
    std::cout << " 原始数量: "<< cloud->width * cloud->height<<std::endl;


    double A = 0.0571989;
    double B = -0.0187938 ;
    double C = -0.046316;


    float A0 = 0.0571989;
    float B0 = -0.0187938 ;
    float C0 = -0.046316;





        float pa1 = -1.9577;
        float pa2 = -0.795895;
        float pa3 = 19.5298;

        float pb1 = -0.124661;
        float pb2 =  2.20117;
        float pb3 = 20.4688;

        float point1 = pa1-pb1;
        float point2 = pa2-pb2;
        float point3 = pa3-pb3;

        float plane1 = 0.0571989;
        float plane2 = -0.0187938;
        float plane3 = -0.046316;

        float k1 = point2*plane3-point3*plane2;
        float k2 = point3*plane1-point1*plane3;
        float k3 = point1*plane2-point2*plane1;

        double d = -k1*pa1-k2*pa2-k3*pa3;







    //cout<<"添加水平面"<<endl;
    //for (float x = atof(argv[3]) ;x<atof(argv[4]);x = x+0.185){
        //for (float y = atof(argv[5]) ;y<atof(argv[6]);y = y+0.102){
    for (float x = -20 ;x<20;x = x+0.185){
        for (float y = -10 ;y<10;y = y+0.102){
            float z = -(1+A0*x + B0*y)/C0;
            pcl::PointXYZRGB point ;
            point.x = x ;
            point.y = y ;
            point.z = z ;
            point.r = 255 ;
            point.g = 0 ;
            point.b = 0 ;
            //cloud->push_back(point);
        }
    }
    pcl::io::savePLYFileBinary(argv[2],*cloud);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(argv[2], *cloud_new) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        pause();
        return -1;
    }    
    // for (size_t i = 0;i<cloud_new->points.size();i++){
    //     cout<<cloud_new->points[i].x<<"    ";
    //     cout<<cloud_new->points[i].y<<"    ";
    //     cout<<cloud_new->points[i].z<<"    "<<endl;
    // }
    //第一个平面
    double p1_1[3] ={1.75459, -1.35326, 8.97379}; 
    double p2_1[3] ={3.75985, -1.35022, 8.76246};
    double abcd_1[4];
    get_plane(p1_1,p2_1,abcd_1,A,B,C);

    double p1_2[3] ={-3.96538,1.28961, 7.50563}; 
    double p2_2[3] ={-3.83836,-0.443925,8.78999};
    double abcd_2[4];
    get_plane(p1_2,p2_2,abcd_2,A,B,C);

    double p1_3[3] ={4.4202, -0.180534, 7.96734}; 
    double p2_3[3] ={4.32761, 1.42498, 6.71566};
    double abcd_3[4];
    get_plane(p1_3,p2_3,abcd_3,A,B,C);
    


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0;i<cloud->points.size();i++){
        float d0 =  A0*cloud->points[i].x+B0*cloud->points[i].y+C0*cloud->points[i].z+1;//d0>=0
        float d1 =  abcd_1[0]*cloud->points[i].x+abcd_1[1]*cloud->points[i].y+abcd_1[2]*cloud->points[i].z+abcd_1[3];//d1<=0
        float d2 =  abcd_2[0]*cloud->points[i].x+abcd_2[1]*cloud->points[i].y+abcd_2[2]*cloud->points[i].z+abcd_2[3];//d2<=0
        float d3 =  abcd_3[0]*cloud->points[i].x+abcd_3[1]*cloud->points[i].y+abcd_3[2]*cloud->points[i].z+abcd_3[3];//d3<=0
        //if ( d0>=0 && d1<=0 && d2<=0 && d3>=0){
            pcl::PointXYZRGB point ;
            point.x = cloud->points[i].x ;
            point.y = cloud->points[i].y ;
            point.z = cloud->points[i].z ;
            point.r = cloud->points[i].r ;
            point.g = cloud->points[i].g ;
            point.b = cloud->points[i].b ;
            cloud2->push_back(point);
        //}
    }
    pcl::io::savePLYFileBinary(argv[3],*cloud2);
    std::cout << " 切割数量: "<< cloud2->width * cloud2->height<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>sor;
    sor.setInputCloud (cloud2);                           //设置待滤波的点云
    sor.setMeanK (50);                               //设置在进行统计时考虑的临近点个数
    sor.setStddevMulThresh (0.1);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
    sor.filter (*cloud3);                    //滤波结果存储到cloud_filtered
    std::cout << " 滤波数量: "<< cloud3->width * cloud3->height<<std::endl;
    pcl::io::savePLYFileBinary(argv[4],*cloud3);


    for (float x = -20 ;x<20;x = x+0.185){
        for (float y = -10 ;y<10;y = y+0.102){
            float z = -(1+A0*x + B0*y)/C0;
            pcl::PointXYZRGB point ;
            point.x = x ;
            point.y = y ;
            point.z = z ;
            point.r = 255 ;
            point.g = 0 ;
            point.b = 0 ;
            cloud3->push_back(point);
        }
    }





    for (double x = -20 ;x<20;x = x+0.185){
        for (double y = -10 ;y<10;y = y+0.102){
            double z = -(d+k1*x + k2*y)/k3;
            pcl::PointXYZRGB point ;
            point.x = x ;
            point.y = y ;
            point.z = z ;
            point.r = 0 ;
            point.g = 255 ;
            point.b = 0 ;
            cloud3->push_back(point);
        }
    }

    pcl::io::savePLYFileBinary(argv[5],*cloud3);



    ofstream outfile_3d;
    outfile_3d.open(argv[6]);
    for (size_t i = 0;i<cloud3->points.size();i++){
            pcl::PointXYZRGB point ;
            point.x = cloud->points[i].x ;
            point.y = cloud->points[i].y ;
            point.z = cloud->points[i].z ;
            outfile_3d<<point.x<<" "<<point.y<<" "<<point.z<<endl;
    }  





    return 0; 
}

