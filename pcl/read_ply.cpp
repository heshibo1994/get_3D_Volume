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

using namespace std;
void CrossProduct(double a[3], double b[3] ,double c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}
double DotProduct(double a[3], double b[3])
{
    double result;
    result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    return result;
}

double Normalize(double v[3])
{
    double result;
    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return result;
}

void RotationMatrix(double angle, double u[3],double rotatinMatrix[3][3])
{
    double norm = Normalize(u);
    u[0] = u[0] / norm;
    u[1] = u[1] / norm;
    u[2] = u[2] / norm;

    rotatinMatrix[0][0] = cos(angle) + u[0] * u[0] * (1 - cos(angle));
    rotatinMatrix[0][1] = u[0] * u[1] * (1 - cos(angle) - u[2] * sin(angle));
    rotatinMatrix[0][2]= u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));

    rotatinMatrix[1][0] = u[2] * sin(angle) + u[0] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][1] = cos(angle) + u[1] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][2] = -u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
    
    rotatinMatrix[2][0] = -u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][1] = u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][2] = cos(angle) + u[2] * u[2] * (1 - cos(angle));
}
void Calculation(double vectorBefore[3], double vectorAfter[3],double rotatinMatrix[3][3] )
{
    cout<<"计算旋转矩阵"<<endl;
    double rotationAxis[3];
    double rotationAngle;
    CrossProduct(vectorBefore, vectorAfter,rotationAxis);
    rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
    RotationMatrix(rotationAngle, rotationAxis,rotatinMatrix);
}

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
  

    ofstream outfile_3d_t;
    outfile_3d_t.open(argv[2]);
    for (size_t i = 0;i<cloud->points.size();i++){
        if (i%1000 == 0){
            pcl::PointXYZRGB point ;
            point.x = cloud->points[i].x ;
            point.y = cloud->points[i].y ;
            point.z = cloud->points[i].z ;
            cout<<point.x<<endl;
            outfile_3d_t<<point.x<<" "<<point.y<<" "<<point.z<<endl;
        }
    }  
    return 0; 
}


