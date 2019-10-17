#include <iostream> 
#include <unistd.h>
#include <string>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>

using namespace std;
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
    std::cout << "Loaded "<< cloud->width * cloud->height<< " data1 points from test_pcd.pcd with the following fields: "<< std::endl;
    // //割
    // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());//创建条件定义对象
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 10.0)));//比较内容

    // // build the filter
    //pcl::ConditionalRemoval<pcl::PointXYZ> condrem;//创建条件滤波器
    // condrem.setCondition(range_cond);              //并用条件定义对象初始化
    // condrem.setInputCloud(cloud);                  //输入点云
    // condrem.setKeepOrganized(true);
    // apply filter
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
    // condrem.filter(*cloud);               //执行滤波
    // pcl::io::savePLYFileBinary("new_earse.ply",*cloud_new);


//-0.7217   14.2773
//-19 3

//-0.0383016 -0.170981 -0.140265 1


    float A = -0.0383016;
    float B = -0.170981 ;
    float C = -0.140265 ;
    for (float x = atof(argv[3]) ;x<atof(argv[4]);x = x+0.185){
        for (float y = atof(argv[5]) ;y<atof(argv[6]);y = y+0.102){
            float z = -(1+A*x + B*y)/C;
            pcl::PointXYZRGB point ;
            point.x = x ;
            point.y = y ;
            point.z = z ;
            point.r = 255 ;
            point.g = 0 ;
            point.b = 0 ;

            //cout<<point.x<<"  "<<point.y<<"  "<<point.z<<endl;
            cloud->push_back(point);
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

    pcl::visualization::CloudViewer viewer("Viewer");
    //viewer.showCloud(cloud);
    //pause();
    return 0; 
}


