#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;
using namespace std;
int main()
{


	Eigen::Vector3d v1 = Eigen::Vector3d(-0.00895437,-0.0432825 ,-0.267926); 



    //Eigen::Vector3d v1 = Eigen::Vector3d(3, 2, 1); 
    Eigen::Vector3d v2 = Eigen::Vector3d(0, 0, 1);
    double angle  = acos(v1.dot(v2)/(v1.norm()*v2.norm()));
    Eigen::Vector3d axis = v1.normalized().cross(v2.normalized());
    // cout<<"v1:"<<v1<<endl;
    // cout<<"v1.:"<<v1.norm()<<endl;
    // cout<<"angle:"<<angle<<endl;
    // cout<<"axis:"<<axis<<endl;
    Eigen::AngleAxisd RotationAngleAxis(angle,axis.normalized());                             
    Eigen::Matrix3d R = RotationAngleAxis.toRotationMatrix();
	cout << "R" << endl << R << endl;    
    Eigen::Vector3d v_resize = R*v1;    
    Eigen::Matrix3d Rr = R/v_resize[2];
    cout << "Rr" << endl << Rr << endl;  
    cout << "result" << endl << Rr*v1 << endl; 
 
}
