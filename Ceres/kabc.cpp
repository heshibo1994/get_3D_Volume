#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <fstream>
#include <vector>

using namespace std;
int TIMES = 100;
vector<vector<float>> colmap, opencv;
vector<float> colmap_data1 ,opencv_data1;//dai youhua 
vector<vector<float>> plane_data1;
vector<float> planex_data1 ,planey_data1, planez_data1;//dai youhua 

// 代价函数的计算模型
struct CURVE_FITTING_COST_k
{
    CURVE_FITTING_COST_k ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const k,     // 模型参数，有1维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - ( k[0]*T ( _x ) ); // y-ax
        return true;
    }
    const double _x, _y;    // x,y数据
};

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

void get_data1_k(string K_path){
    std::ifstream infile_feat(K_path); //加载数据文件
	std::string feature; //存储读取的每行数据
	float feat_onePoint;  //存储每行按空格分开的每一个float数据
	while(!infile_feat.eof()) 
	{	
        std::vector<float> temp_colmap,temp_opencv;
		getline(infile_feat, feature); //一次读取一行数据
		stringstream stringin(feature); //使用串流实现对string的输入输出操作
        int index = 0;
		while (stringin >> feat_onePoint) {      //按空格一次读取一个数据存入feat_onePoint 
            if (index<3){temp_colmap.push_back(feat_onePoint);}
            else{temp_opencv.push_back(feat_onePoint);}
        index = index+1;
        }
        
		colmap.push_back(temp_colmap); //存储所有数据
        opencv.push_back(temp_opencv); //存储所有数据
	}

	infile_feat.close();
    for (int i=0;i<1000;i++){
        int p1 = rand() % (colmap.size()-1) ;
        int p2 = rand() % (opencv.size()-1) ;
        //cout<<i<<" "<<p1<<" "<<p2<<" "<<colmap.size()<<" "<<opencv.size()<<endl;
        float c = sqrt((colmap[p1][0] -  colmap[p2][0])*(colmap[p1][0] -  colmap[p2][0]) + (colmap[p1][1] -  colmap[p2][1])*(colmap[p1][1] -  colmap[p2][1])+ (colmap[p1][2] -  colmap[p2][2])*(colmap[p1][2] -  colmap[p2][2]));
        float o = sqrt((opencv[p1][0] -  opencv[p2][0])*(opencv[p1][0] -  opencv[p2][0]) + (opencv[p1][1] -  opencv[p2][1])*(opencv[p1][1] -  opencv[p2][1])+ (opencv[p1][2] -  opencv[p2][2])*(opencv[p1][2] -  opencv[p2][2]));
        
        colmap_data1.push_back(c);
        opencv_data1.push_back(o);
        cout<<"o "<<o<<" c "<<c<<" "<<c/o<<endl;
        //cout<<"over"<<endl;
    }
}

void get_data1_plane(string plane_path){
    std::ifstream infile_feat(plane_path); //加载数据文件
	std::string feature; //存储读取的每行数据
	float feat_onePoint;  //存储每行按空格分开的每一个float数据
	while(!infile_feat.eof()) 
	{	
        std::vector<float> temp_plane;
		getline(infile_feat, feature); //一次读取一行数据
		stringstream stringin(feature); //使用串流实现对string的输入输出操作
		int index = 0;
        while (stringin >> feat_onePoint) {      //按空格一次读取一个数据存入feat_onePoint 
            if (index == 0 ){planex_data1.push_back(feat_onePoint);}
            else if (index == 1 ){planey_data1.push_back(feat_onePoint);}
            else if (index == 2 ){planez_data1.push_back(feat_onePoint);}
        index = index+1;
        }
	}
	infile_feat.close();

}
int main ( int argc, char** argv )
{   
    //将abc写进kabc.txt
    ofstream outfile_kabc;
    outfile_kabc.open(string(argv[3]) ,ios::app);

    double k[1] = {0};
    string K_path = argv[1];
    get_data1_k(K_path);

    // 构建最小二乘问题
    ceres::Problem problem;
    for ( int i=0; i<colmap_data1.size(); i++ ){
	problem.AddResidualBlock (new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_k, 1, 1> ( new CURVE_FITTING_COST_k ( opencv_data1[i], colmap_data1[i] )),nullptr,k);
     // 向问题中添加误差项  使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致 // 核函数，这里不使用，为空                // 待估计参数

    }
    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout
    ceres::Solver::Summary summary;                // 优化信息
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    // 输出结果
    //cout<<summary.BriefReport() <<endl;
    //cout<<"estimated k = ";
    for ( auto a:k ) {
        //cout<<a<<" "<<endl;
        outfile_kabc<<a<<endl;
        }



    double plane[4] = {0.1,0.1,0.2,0};
    string plane_path = argv[2];
    get_data1_plane(plane_path);
    //构建最小二乘问题
    ceres::Problem problem2;

    for ( int i=0; i<planex_data1.size(); i++ ){
	    problem2.AddResidualBlock (new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_plane, 1, 4> ( new CURVE_FITTING_COST_plane ( planex_data1[i], planey_data1[i],planez_data1[i]) ),nullptr,plane);
     // 向问题中添加误差项  使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致 // 核函数，这里不使用，为空                // 待估计参数

    }
    // 配置求解器
    ceres::Solver::Options options2;     // 这里有很多配置项可以填
    options2.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options2.minimizer_progress_to_stdout = true;   // 输出到cout
    ceres::Solver::Summary summary2;                // 
    ceres::Solve ( options2, &problem2, &summary2 );  // 开优化信息始优化
    // 输出结果
    //cout<<summary2.BriefReport() <<endl;
    //cout<<summary2.FullReport() <<endl;
    //cout<<"estimated plane = ";
    for ( auto a:plane ) {
        //cout<<a*1/plane[3]<<" ";
        outfile_kabc<<a*1/plane[3]<<endl;
        }
    return 0;
}

