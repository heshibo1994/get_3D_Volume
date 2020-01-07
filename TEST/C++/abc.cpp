#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <vector>
#include <string>

using namespace std;
using namespace Eigen;
// 代价函数的计算模型
struct CURVE_FITTING_COST_plane
{
    CURVE_FITTING_COST_plane ( double x, double y, double z  ) : _x ( x ), _y ( y ), _z ( z ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const plane,     // 模型参数，有4维
        T* residual ) const     // 残差
    {
        residual[0] =   plane[0]*T (_x ) + plane[1]*T (_y ) + plane[2]*T (_z ) + plane[3] ; // 0 - (ax+by+cz+d)
        return true;
    }
    const double _x, _y, _z;    // x,y,z数据
};

//  旋转矩阵
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

int main ( int argc, char** argv )
{   
    string plane_path = argv[1];
    std::ifstream fdata(plane_path);
	ofstream outfile_kabc;
    outfile_kabc.open(string(argv[2]));
    double abc[4] = {0.1,0.1,0.2,0};

    int mdata = 0;
	fdata >> mdata;
    double* xdata = new double[mdata];
	double* ydata = new double[mdata];
	double* zdata = new double[mdata];
	for (int i = 0; i < mdata; i++)
	{
		fdata >> xdata[i] >> ydata[i] >> zdata[i];
	}
 
    //构建最小二乘问题
    ceres::Problem problem;
    for ( int i=0; i<mdata; i++ ){
	    problem.AddResidualBlock (new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_plane, 1, 4> ( new CURVE_FITTING_COST_plane ( xdata[i],ydata[i],zdata[i]) ),nullptr,abc);
     // 向问题中添加误差项  使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致 // 核函数，这里不使用，为空                // 待估计参数
    }
    // 配置求解器
    ceres::Solver::Options options2;     // 这里有很多配置项可以填
    options2.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options2.minimizer_progress_to_stdout = true;   // 输出到cout
    ceres::Solver::Summary summary2;                // 
    ceres::Solve ( options2, &problem, &summary2 );  // 开优化信息始优化
    outfile_kabc<<"abc:"<<endl;
    for ( int i=0;i<3;i++ ) {
        outfile_kabc<<abc[i]*1/abc[3]<<endl;
        abc[i] = abc[i]*1/abc[3];
        cout<<abc[i]<<endl;
    }

    
	Eigen::Vector3d v1 = Eigen::Vector3d(abc[0],abc[1],abc[2]); 
	Eigen::Matrix3d Rr = getR(v1);
    outfile_kabc<<"Rr:"<<endl;
	outfile_kabc<<Rr<<endl;
    cout<<Rr<<endl;


    string xyzrgb_path = argv[3];
    std::ifstream fxyzrgb(xyzrgb_path);
    string pointRply_path = argv[4];
    std::ofstream fply(pointRply_path);
    string xyzrgbR_path = argv[5];
    std::ofstream fxyzrgbR(xyzrgbR_path);
    int numxyzrgb = 0;
    fxyzrgb>>numxyzrgb;
	fply<<"ply\n";
	fply<<"format ascii 1.0\n";
	fply<<"element vertex "<<numxyzrgb<<"\n";
	fply<<"property float x\n";
	fply<<"property float y\n";
	fply<<"property float z\n";
	fply<<"property uchar diffuse_red\n";
	fply<<"property uchar diffuse_green\n";
	fply<<"property uchar diffuse_blue\n";
	fply<<"end_header\n";
    fxyzrgbR<<numxyzrgb<<"\n";

    double* x = new double[numxyzrgb];
	double* y = new double[numxyzrgb];
	double* z = new double[numxyzrgb];
    int* r = new int[numxyzrgb];
	int* g = new int[numxyzrgb];
	int* b = new int[numxyzrgb];

	for(int i = 0;i<numxyzrgb;i++){
		fxyzrgb>>x[i]>>y[i]>>z[i]>>r[i]>>g[i]>>b[i];
		Eigen::Vector3d xyz = Eigen::Vector3d(x[i],y[i],z[i]); 
		Eigen::Vector3d  new_xyz =Rr * xyz;
		fply<<new_xyz(0,0)<<" "<<new_xyz(1,0)<<" "<<new_xyz(2,0)<<" "<<r[i]<<" "<<g[i]<<" "<<b[i]<<endl;
        fxyzrgbR<<new_xyz(0,0)<<" "<<new_xyz(1,0)<<" "<<new_xyz(2,0)<<" "<<r[i]<<" "<<g[i]<<" "<<b[i]<<endl;
	}
    return 0;
}

