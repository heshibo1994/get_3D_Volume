#include <iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp> 
#include<opencv2/xfeatures2d.hpp>
#include<opencv2/core/core.hpp>
 
using namespace cv;  
using namespace std;
using namespace cv::xfeatures2d;//只有加上这句命名空间，SiftFeatureDetector and SiftFeatureExtractor才可以使用
 
int main()
{
	//Create SIFT class pointer
	Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
	//SiftFeatureDetector siftDetector;
	//Loading images
	Mat img_1 = imread("341.jpg");
	Mat img_2 = imread("705.jpg");
	if (!img_1.data || !img_2.data)
	{
		cout << "Reading picture error！" << endl;
		return false;
	}
	//Detect the keypoints
	double t0 = getTickCount();//当前滴答数
	vector<KeyPoint> keypoints_1, keypoints_2;
	f2d->detect(img_1, keypoints_1);
	f2d->detect(img_2, keypoints_2);
	cout << "The keypoints number of img1 is:" << keypoints_1.size() << endl;
	cout << "The keypoints number of img2 is:" << keypoints_2.size() << endl;
	//Calculate descriptors (feature vectors)
	Mat descriptors_1, descriptors_2;
	f2d->compute(img_1, keypoints_1, descriptors_1);
	f2d->compute(img_2, keypoints_2, descriptors_2);
	double freq = getTickFrequency();
	double tt = ((double)getTickCount() - t0) / freq;
	cout << "Extract SIFT Time:" <<tt<<"ms"<< endl;
	//画关键点
	Mat img_keypoints_1, img_keypoints_2;
	drawKeypoints(img_1,keypoints_1,img_keypoints_1,Scalar::all(-1),0);
	drawKeypoints(img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), 0);
	//imshow("img_keypoints_1",img_keypoints_1);
	//imshow("img_keypoints_2",img_keypoints_2);
 
	//Matching descriptor vector using BFMatcher
	BFMatcher matcher;
	vector<DMatch> matches;
	matcher.match(descriptors_1, descriptors_2, matches);
	cout << "The number of match:" << matches.size()<<endl;
	//绘制匹配出的关键点
	Mat img_matches;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);
	//imshow("Match image",img_matches);
	//计算匹配结果中距离最大和距离最小值
	double min_dist = matches[0].distance, max_dist = matches[0].distance;
	for (int m = 0; m < matches.size(); m++)
	{
		if (matches[m].distance<min_dist)
		{
			min_dist = matches[m].distance;
		}
		if (matches[m].distance>max_dist)
		{
			max_dist = matches[m].distance;
		}	
	}
	cout << "min dist=" << min_dist << endl;
	cout << "max dist=" << max_dist << endl;
	//筛选出较好的匹配点
	vector<DMatch> goodMatches;
	for (int m = 0; m < matches.size(); m++)
	{
		if (matches[m].distance < 0.6*max_dist)
		{
			goodMatches.push_back(matches[m]);
		}
	}
	cout << "The number of good matches:" <<goodMatches.size()<< endl;
	//画出匹配结果
	Mat img_out;
	//红色连接的是匹配的特征点数，绿色连接的是未匹配的特征点数
	//matchColor – Color of matches (lines and connected keypoints). If matchColor==Scalar::all(-1) , the color is generated randomly.
	//singlePointColor – Color of single keypoints(circles), which means that keypoints do not have the matches.If singlePointColor == Scalar::all(-1), the color is generated randomly.
	//CV_RGB(0, 255, 0)存储顺序为R-G-B,表示绿色
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, goodMatches, img_out, Scalar::all(-1), CV_RGB(0, 0, 255), Mat(), 2);
	imshow("good Matches",img_out);
    //RANSAC匹配过程
	vector<DMatch> m_Matches;
	m_Matches = goodMatches;
	int ptCount = goodMatches.size();
	if (ptCount < 100)
	{
		cout << "Don't find enough match points" << endl;
		return 0;
	}
 
	//坐标转换为float类型
	vector <KeyPoint> RAN_KP1, RAN_KP2;
	//size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		RAN_KP1.push_back(keypoints_1[goodMatches[i].queryIdx]);
		RAN_KP2.push_back(keypoints_2[goodMatches[i].trainIdx]);
		//RAN_KP1是要存储img01中能与img02匹配的点
		//goodMatches存储了这些匹配点对的img01和img02的索引值
	}
	//坐标变换
	vector <Point2f> p01, p02;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		p01.push_back(RAN_KP1[i].pt);
		p02.push_back(RAN_KP2[i].pt);
	}
	/*vector <Point2f> img1_corners(4);
	img1_corners[0] = Point(0,0);
	img1_corners[1] = Point(img_1.cols,0);
	img1_corners[2] = Point(img_1.cols, img_1.rows);
	img1_corners[3] = Point(0, img_1.rows);
	vector <Point2f> img2_corners(4);*/
	////求转换矩阵
	//Mat m_homography;
	//vector<uchar> m;
	//m_homography = findHomography(p01, p02, RANSAC);//寻找匹配图像
	//求基础矩阵 Fundamental,3*3的基础矩阵
	vector<uchar> RansacStatus;
	Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, FM_RANSAC);
	//重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
	vector <KeyPoint> RR_KP1, RR_KP2;
	vector <DMatch> RR_matches;
	int index = 0;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			RR_KP1.push_back(RAN_KP1[i]);
			RR_KP2.push_back(RAN_KP2[i]);
			m_Matches[i].queryIdx = index;
			m_Matches[i].trainIdx = index;
			RR_matches.push_back(m_Matches[i]);
			index++;
		}
	}
	cout << "RANSAC后匹配点数" <<RR_matches.size()<< endl;
	Mat img_RR_matches;
	drawMatches(img_1, RR_KP1, img_2, RR_KP2, RR_matches, img_RR_matches);
	imshow("After RANSAC",img_RR_matches);
	//等待任意按键按下
	waitKey(0);
}
