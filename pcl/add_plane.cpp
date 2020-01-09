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
    

 
//   3.38232  -1.45144  0.121435
//  -1.45144  -3.33321  0.586976
// -0.121435 -0.586976  -3.63349



// 0.3800021236950053, 0.92498561393531187, 5.3017912861334739, 4.4147596433917533
//1.4396890792284729, 1.1988688669733951
    // l[0] = cos*point[0]+sin*point[1]+x
    // l[1] = -sin*point[0]+cos*point[1]+y
    // l[2] = point[2]+13.58
//   Rr = [[3.38232,-1.45144,0.121435],
//        [-1.45144,-3.33321,0.586976],
//        [-0.121435,-0.586976,-3.63349]]



    float cos = 0.3800021236950053;
    float sin = 0.92498561393531187;
    float x = -1.4396890792284729;
    float y = 1.0903137888901073;



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dense1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dense2(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0;i<cloud->points.size();i++){
        pcl::PointXYZRGB point1 ;
        pcl::PointXYZRGB point2 ;
        float k = 3.698;
        point1.x = (cloud->points[i].x*3.38232  + cloud->points[i].y*(-1.45144)  + cloud->points[i].z* 0.121435)/k;
        point1.y = (cloud->points[i].x*(-1.45144)  + cloud->points[i].y*(-3.33321)  + cloud->points[i].z* 0.586976)/k;
        point1.z = (cloud->points[i].x*(-0.121435) + cloud->points[i].y*(-0.586976) + cloud->points[i].z* (-3.63349))/k;
        point1.r = cloud->points[i].r ;
        point1.g = cloud->points[i].g ;
        point1.b = cloud->points[i].b ;
        cloud_dense1->push_back(point1);


        point2.x = cos*point1.x+sin*point1.y+x;
        point2.y = -sin*point1.x+cos*point1.y+y;
        point2.z = point1.z+3.672;
        point2.r = cloud->points[i].r ;
        point2.g = cloud->points[i].g ;
        point2.b = cloud->points[i].b ;
        cloud_dense2->push_back(point2);
    } 
    pcl::io::savePLYFileBinary(argv[2],*cloud_dense1);
    pcl::io::savePLYFileBinary(argv[3],*cloud_dense2);







    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dense3(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>sor;
    sor.setInputCloud (cloud_dense2);                           //设置待滤波的点云
    sor.setMeanK (50);                               //设置在进行统计时考虑的临近点个数
    sor.setStddevMulThresh (0.1);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
    sor.filter (*cloud_dense3);                    //滤波结果存储到cloud_filtered
    std::cout << " 滤波数量: "<< cloud_dense3->width * cloud_dense3->height<<std::endl;
    pcl::io::savePLYFileBinary(argv[4],*cloud_dense3);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dense4(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0;i<cloud_dense3->points.size();i++){
        if (cloud_dense3->points[i].x>-4 && 
            cloud_dense3->points[i].x<40 &&
            cloud_dense3->points[i].y>0 && 
            cloud_dense3->points[i].y<24 &&
            cloud_dense3->points[i].z>0){
                pcl::PointXYZRGB point ;
                point.x = cloud_dense3->points[i].x ;
                point.y = cloud_dense3->points[i].y ;
                point.z = cloud_dense3->points[i].z ;
                point.r = cloud_dense3->points[i].r ;
                point.g = cloud_dense3->points[i].g ;
                point.b = cloud_dense3->points[i].b ;
                cloud_dense4->push_back(point);
        }
    }
    std::cout << " 切割滤波数量:=============================================== "<< cloud_dense4->width * cloud_dense4->height<<std::endl;
    pcl::io::savePLYFileBinary(argv[5],*cloud_dense4);




    // //获得点的坐标文本
    // ofstream outfile_3d;
    // outfile_3d.open(argv[4]);
    // for (size_t i = 0;i<cloud_cut->points.size();i++){
    //     pcl::PointXYZRGB point ;
    //     point.x = cloud_cut->points[i].x ;
    //     point.y = cloud_cut->points[i].y ;
    //     point.z = cloud_cut->points[i].z ;
    //     outfile_3d<<point.x<<" "<<point.y<<" "<<point.z<<endl;
    // }  
    return 0; 
}
