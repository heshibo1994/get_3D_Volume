////////////////////////////////////////////////////////////////////////////////
// 2d.cpp                                                                     //
// ------                                                                     //
//                                                                            //
// Interpolates z = sin(x)cos(y) on the interval [-pi, pi] X [-pi, pi] using  //
// 15 evenly-spaced points along the x axis and 15 evenly-spaced points along //
// the y axis.                                                                //
////////////////////////////////////////////////////////////////////////////////

//#include <mlinterp>
#include <mba/mba.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>


using namespace Eigen;

Matrix3d getR(Vector3d v1)
{
    Eigen::Vector3d v2 = Eigen::Vector3d(0, 0, 1);
    double angle  = acos(v1.dot(v2)/(v1.norm()*v2.norm()));
    Eigen::Vector3d axis = v1.normalized().cross(v2.normalized());
    Eigen::AngleAxisd RotationAngleAxis(angle,axis.normalized());                             
    Eigen::Matrix3d R = RotationAngleAxis.toRotationMatrix();
    Eigen::Vector3d v_resize = R*v1;    
    Eigen::Matrix3d Rr = R/v_resize[2];
	return Rr;  
}



//using namespace mlinterp;
using namespace std;

int main() {
	//将所有数据读到vector中
	std::ifstream fdata("data1/points3D.txt");
	vector<vector<double>> XYZRGB;
	std::string xyzrgb; //存储读取的每行数据
	double onePoint;  //存储每行按空格分开的每一个float数据
	while(!fdata.eof()) 
	{	
        std::vector<double> temp;
		getline(fdata, xyzrgb); //一次读取一行数据
		stringstream stringin(xyzrgb); //使用串流实现对string的输入输出操作
        int index = 0;
		while (stringin >> onePoint) {      //按空格一次读取一个数据存入feat_onePoint 
           	temp.push_back(onePoint);
        }  
		XYZRGB.push_back(temp); //存储所有数据
	}
	Eigen::Vector3d v1 = Eigen::Vector3d(-0.0321351, -0.383098, -0.184525); 
	Eigen::Matrix3d Rr = getR(v1);
	cout<<Rr<<endl;
	//点云旋转,生成ply文件	
	std::ofstream fply("data1/points.ply");
	fply<<"ply\n";
	fply<<"format ascii 1.0\n";
	fply<<"element vertex "<<XYZRGB[0][0]<<"\n";
	fply<<"property float x\n";
	fply<<"property float y\n";
	fply<<"property float z\n";
	fply<<"property uchar diffuse_red\n";
	fply<<"property uchar diffuse_green\n";
	fply<<"property uchar diffuse_blue\n";
	fply<<"end_header\n";
	for(int i = 1;i<=XYZRGB[0][0];i++){
		//cout<<i<<endl;
		Eigen::Vector3d xyz = Eigen::Vector3d(XYZRGB[i][1],XYZRGB[i][2],XYZRGB[i][3]); 
		Eigen::Vector3d  new_xyz =Rr * xyz;
		fply<<new_xyz(0,0)<<" "<<new_xyz(1,0)<<" "<<new_xyz(2,0)<<" "<<XYZRGB[i][4]<<" "<<XYZRGB[i][5]<<" "<<XYZRGB[i][6]<<endl;
	}
	//PLY点云过滤
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("data1/points.ply", *cloud) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        pause();
        return -1;
    }
    //计算点的数量
    std::cout << " 原始数量: "<< cloud->width * cloud->height<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_remove(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>sor;
    sor.setInputCloud (cloud);                     //设置待滤波的点云
    sor.setMeanK (50);                             //设置在进行统计时考虑的临近点个数
    sor.setStddevMulThresh (0.1);                  //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
    sor.filter (*cloud_remove);                    //滤波结果存储到cloud_filtered
    std::cout << " 离群点滤波数量: "<< cloud_remove->width * cloud_remove->height<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0;i<cloud_remove->points.size();i++){
		pcl::PointXYZRGB point ;
		point.x = cloud->points[i].x ;
		point.y = cloud->points[i].y ;
		point.z = cloud->points[i].z ;
		point.r = cloud->points[i].r ;
		point.g = cloud->points[i].g ;
		point.b = cloud->points[i].b ;
		if (point.y<12){
			cloud_filter->push_back(point);
    	}
    }
	std::cout << " 几何点滤波数量: "<< cloud_filter->width * cloud_filter->height<<std::endl;
    pcl::io::savePLYFileBinary("data1/cloud_filter.ply",*cloud_filter);
	
	//mba拟合
	std::vector<mba::point<2>> coo;
	std::vector<double> val;
    for (size_t i = 0;i<cloud_filter->points.size();i++){
		pcl::PointXYZRGB point ;
		point.x = cloud_filter->points[i].x ;
		point.y = cloud_filter->points[i].y ;
		point.z = cloud_filter->points[i].z ;
		mba::point<2> pt_xy;
		pt_xy[0] = point.x;
		pt_xy[1] = point.y;
		coo.push_back(pt_xy);
		val.push_back(point.z); 
    }
	mba::point<2> lo2 = {-15,-9 };
	mba::point<2> hi2 = {25,12 };
	mba::index<2> grid2 = { 3,3 };
	mba::MBA<2> interp2(lo2, hi2, grid2, coo, val);

	std::ofstream prez_ply("data1/prez_points.ply");
	prez_ply<<"ply\n";
	prez_ply<<"format ascii 1.0\n";
	prez_ply<<"element vertex "<<XYZRGB[0][0]<<"\n";
	prez_ply<<"property float x\n";
	prez_ply<<"property float y\n";
	prez_ply<<"property float z\n";
	prez_ply<<"property uchar diffuse_red\n";
	prez_ply<<"property uchar diffuse_green\n";
	prez_ply<<"property uchar diffuse_blue\n";
	prez_ply<<"end_header\n";

	for (double i =-14; i <24;i=i+0.05){
		for (double  j=-6;j<11;j=j+0.05){
			mba::point<2> pt_q;
			pt_q[0] = i;
			pt_q[1] = j;
			double pre_z = interp2(pt_q);
			prez_ply<<i<<" "<<j<<" "<<pre_z<<" 0 255 255\n";
		}
	}
 
	return 0;


}