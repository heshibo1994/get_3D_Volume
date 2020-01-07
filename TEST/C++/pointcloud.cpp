#include <mba/mba.hpp>
#include <iostream> 
#include <unistd.h>
#include <string>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>
#include <vector>


using namespace std;
int main(int argc, char * argv[])
{

    cv::FileStorage fs(argv[1],cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+"count not open file :"+argv[1]);
    float x_max,x_min;
    float y_max,y_min;
    float K;
    fs["x_max"]>>x_max;
    fs["x_min"]>>x_min;
    fs["y_max"]>>y_max;
    fs["y_min"]>>y_min;
    fs["K"]>>K;


    string cloud_path = string(argv[2]);           //初始点云
    string cloud_filter_path = string(argv[3]);    //离群点后的点云
    string cloud_cut_path = string(argv[4]);       //品面切割后的点云
    string mba_plypath = string(argv[5]);       //mbaply

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(cloud_path, *cloud) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        pause();
        return -1;
    }
    //计算点的数量
    std::cout << " 原始数量: "<< cloud->width * cloud->height<<std::endl;

    //滤波设置。将滤波后的结果放在cloud_filter(argv[3])中
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>sor;
    sor.setInputCloud (cloud);                           //设置待滤波的点云
    sor.setMeanK (50);                               //设置在进行统计时考虑的临近点个数
    sor.setStddevMulThresh (0.1);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
    sor.filter (*cloud_filter);                    //滤波结果存储到cloud_filtered
    std::cout << " 离群点滤波数量: "<< cloud_filter->width * cloud_filter->height<<std::endl;
    pcl::io::savePLYFileBinary(cloud_filter_path,*cloud_filter);


    //获取切割平面方程
    //经过两个点，且垂直与水平面的切割面，输入两个点的坐标和水平面方程参数
    //将切割后的结果放在cloud_cut(argv[4])后
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0;i<cloud_filter->points.size();i++){
        if (cloud_filter->points[i].x>x_min/K && cloud_filter->points[i].x<x_max/K &&cloud_filter->points[i].y>y_min/K && cloud_filter->points[i].y<y_max/K){
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
    std::cout << " 切割滤波数量: "<< cloud_cut->width * cloud_cut->height<<std::endl;
    pcl::io::savePLYFileBinary(cloud_cut_path,*cloud_cut);
    

    



    // //获得点的坐标文本
    // ofstream outfile_3d;
    // outfile_3d.open(mba_path);
    // for (size_t i = 0;i<cloud_cut->points.size();i++){
    //     pcl::PointXYZRGB point ;
    //     point.x = cloud_cut->points[i].x ;
    //     point.y = cloud_cut->points[i].y ;
    //     point.z = cloud_cut->points[i].z ;
    //     outfile_3d<<point.x<<" "<<point.y<<" "<<point.z<<endl;
    // }  

	std::vector<mba::point<2>> coo;
	std::vector<double> val;
    for (size_t i = 0;i<cloud_cut->points.size();i++){
		pcl::PointXYZRGB point ;
		point.x = cloud_cut->points[i].x ;
		point.y = cloud_cut->points[i].y ;
		point.z = cloud_cut->points[i].z ;
		mba::point<2> pt_xy;
		pt_xy[0] = point.x;
		pt_xy[1] = point.y;
		coo.push_back(pt_xy);
		val.push_back(point.z); 
    }

    mba::point<2> lo2 = {x_min,y_min };
	mba::point<2> hi2 = {x_max,y_max };
	mba::index<2> grid2 = { 3,3 };
	mba::MBA<2> interp2(lo2, hi2, grid2, coo, val);

    int num = (x_max-x_min)*(y_max-y_min)*20*20/K/K;

	std::ofstream prez_ply(mba_plypath);
	prez_ply<<"ply\n";
	prez_ply<<"format ascii 1.0\n";
	prez_ply<<"element vertex "<<to_string(num)<<"\n";
	prez_ply<<"property float x\n";
	prez_ply<<"property float y\n";
	prez_ply<<"property float z\n";
	prez_ply<<"property uchar diffuse_red\n";
	prez_ply<<"property uchar diffuse_green\n";
	prez_ply<<"property uchar diffuse_blue\n";
	prez_ply<<"end_header\n";


    double total = 0;
    int k = 0;

	for (double i =x_min/K; i <x_max/K;i=i+0.05){
		for (double  j=y_min/K;j<y_max/K;j=j+0.05){
			mba::point<2> pt_q;
			pt_q[0] = i;
			pt_q[1] = j;
			double pre_z = interp2(pt_q);
			prez_ply<<i<<" "<<j<<" "<<pre_z<<" 255 0 0\n";
            if (pre_z>0){
                total+=pre_z;
                k+=1;
            }
		}
	}
    double Volume =(x_max-x_min)*(y_max-y_min)*total*K/k;
    cout<<"体积: "<<Volume<<" m^3"<<endl;

    return 0; 
}
