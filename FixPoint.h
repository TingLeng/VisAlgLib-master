#pragma once
#include"FindLine.h"
#include "VisionLibrary.h"
#include "Caliper.h"
#include<stdlib.h>
#include <iostream>
#include <string>
#include "cxcore.hpp"
#include "cv.hpp"
using namespace System;
using namespace std;
using namespace cv;
using namespace System::Drawing;
using namespace System::Drawing::Imaging;
using namespace System::Runtime::InteropServices;
namespace VisionLibrary
{
	System::Drawing::Bitmap^  ConverToBitmap(cv::Mat * src);
	int ConvertBitmapToMat(System::Drawing::Bitmap^ bmpImg, cv::Mat& cvImg);
	public ref class FixPoint
	{
	public:
		//ref struct detectedPoint 
		//{
			int x;  //Ѱ�ҵ��ĵ�x����
			int y;
		//};
		//ref struct middlePoint
		//{
			int x1;  //�е�1��x����
			int y1;
			int x2; //�е�2��x����
			int y2;
		//};
		//detectedPoint XY;// Ѱ�ҵ��ĵ�
		//middlePoint midPoint;//�е�
		double posOfRealWorld_X, posOfRealWorld_Y;
		double angleToHorizon;
		double midPos_X, midPos_Y;
		float aa1,bb1,tt1,aa2,bb2,tt2; //����ϵ�任����[aa1,bb1,tt1;aa2,bb2,tt2];
		int t1_x, t1_y, t2_x, t2_y, t3_x, t3_y;
		FixPoint() {};
		~FixPoint() {};
		int getFixPoint(System::Drawing::Bitmap^ via, int pointAX, int pointAY, int pointBx, int PointBy,int pointCX, int pointCY, int pointDx, int PointDy, System::Drawing::Bitmap^ %vis);
		int getFixPoint(System::String^ strPath, int pointAX, int pointAY, int pointBx, int PointBy,int pointCX, int pointCY, int pointDx, int PointDy, System::Drawing::Bitmap^ %vis);
		int getLineA(Mat img, int pointAX, int pointAY, int pointBx, int PointBy,FindLine::Line & lineA);
		int getLineB(Mat img, int pointAX, int pointAY, int pointBx, int PointBy, FindLine::Line & lineB);
		void getAngleToHorizon();
		void transForm(double x1, double y1, double  x2, double y2, double x3, double y3, double a1, double b1, double a2, double b2, double a3,double b3);
		void pictureToWorld(double a1, double b1, double t1, double a2, double b2, double t2, double XX, double YY);
		void getRealMid(double center1_x,double center1_y,double center2_x,double center2_y);
		void diffGetPoint(System::String^ s1, System::String^ s2, System::String^ s3, System::String^ s4);
		Mat maxConnectedCompents(Mat &img, cv::Point &centerPoint);
	private:

	};


}