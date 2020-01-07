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


//using namespace mlinterp;
using namespace std;

int main() {
	//std::array<int, 10> arr = { 1,2,3,4,5,6,7,8,9,0 };
	//std::for_each(arr.begin(), arr.end(), [](int& i) {i++; });
	//for (auto i : arr) { std::cout << i << " "; }
	
	// Coordinates of data points.
	std::vector<mba::point<2>> coo = {
		{0.0, 0.0},
		{0.0, -1.0},
		{1.0, 0.0},
		{1.0, 1.0},
		{0.4, 0.4},
		{0.6, 0.6}
	};

	// Data values.
	std::vector<double> val = {
		0.2, 0.0, 0.0, -0.2, -1.0, 1.0
	};

	// Bounding box containing the data points.
	mba::point<2> lo = { -0.1, -0.1 };
	mba::point<2> hi = { 1.1,  1.1 };

	// Initial grid size.
	mba::index<2> grid = { 3, 3 };

	// Algorithm setup.
	mba::MBA<2> interp(lo, hi, grid, coo, val);

	// Get interpolated value at arbitrary location.
	double w = interp(mba::point<2>{0.99, 0.99});
	cout<<w<<endl;


	///////////////////////////////////////////////////////////////
	///////////////////my test//////////////////////////////////////
	////////////////////////////////////////////////////////////
	coo.clear();
	val.clear();
	int mdata = 0, ndata;
	std::ifstream fdata("data/1.txt");

	fdata >> mdata;

	fdata >> ndata;
	cout<<mdata<<endl;
	double* xdata = new double[mdata];
	double* ydata = new double[mdata];
	double* zdata = new double[mdata];
	for (int i = 0; i < mdata; i++)
	{
		fdata >> xdata[i] >> ydata[i] >> zdata[i];
		mba::point<2> pt_xy;
		pt_xy[0] = xdata[i];
		pt_xy[1] = ydata[i];
		coo.push_back(pt_xy);
		val.push_back(zdata[i]);
		// cout<< zdata[i]<<endl;
	}

	// Bounding box containing the data points.
	// mba::point<2> lo2 = { -2.5, -7 };
	// mba::point<2> hi2 = {  5.5,  -1 };
	// mba::point<2> lo2 = { -10, -10 };
	// mba::point<2> hi2 = {  20,  20 };
	mba::point<2> lo2 = {-15,-9 };
	mba::point<2> hi2 = {25,12 };

	// Initial grid size.
	mba::index<2> grid2 = { 3,3 };

	// Algorithm setup.
	//mba::MBA<2> interp2(lo2, hi2, grid2, coo, val,1,0.1);
	mba::MBA<2> interp2(lo2, hi2, grid2, coo, val);
	// Get interpolated value at arbitrary location.
	double w2 = interp2(mba::point<2>{0.0, -5});


	int mquery = 0,nquery;
	std::ifstream fquery("data/1.txt");
	std::ofstream oerror("data/error_1.txt");
	fquery >> mquery;
	fquery >> nquery;

	double* xq = new double[mquery];
	double* yq = new double[mquery];
	double* zq = new double[mquery];
	//double* zq2 = new double[mquery];
	double total = 0;
	int k = 0;
	for (int i = 0; i < mquery; i++)
	{
		fquery >> xq[i] >> yq[i] >> zq[i];
		mba::point<2> pt_q;
		pt_q[0] = xq[i];
		pt_q[1] = yq[i];
		double w3 = interp2(pt_q);
		double error = w3 - zq[i];

		//oerror <<"x: " <<xq[i]<<"   y: " <<yq[i]<<"   z: " <<zq[i]<<"   z_test: " <<w3<<"   error： " << error <<"   percent： " <<percent<<endl;
		oerror<<xq[i]<<" "<<yq[i]<<" "<<zq[i]<<" "<<xq[i]<<" "<<yq[i]<<" "<<w3<<"   "<<zq[i]-w3<<endl;

	}


	std::ofstream opre_z("data/pre_z.txt");
	for (double i =-10; i <20;i=i+0.05){
		for (double  j=0.5;j<13;j=j+0.05){
			mba::point<2> pt_q;
			pt_q[0] = i;
			pt_q[1] = j;
			double pre_z = interp2(pt_q);
			opre_z<<i<<" "<<j<<" "<<pre_z<<endl;
		}
	}

}
