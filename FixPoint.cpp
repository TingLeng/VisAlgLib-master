#include"FixPoint.h"
#include"FindLine.h"
#include "VisionLibrary.h"
#include "Caliper.h"
#include<stdlib.h>
#include <iostream>
#include <string>
#include "cxcore.hpp"
#include "cv.hpp"
#include<math.h>
using namespace System;
using namespace std;
using namespace cv;
using namespace System::Drawing;
using namespace System::Drawing::Imaging;
using namespace System::Runtime::InteropServices;

namespace VisionLibrary
{
	Mat FixPoint::maxConnectedCompents(Mat &img, cv::Point &centerPoint)
	{
		vector<vector<cv::Point>> contours;
		findContours(img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		double maxArea = 0;
		//vector<Point> maxContour;
		int maxNum = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			double area = contourArea(contours[i]);
			if (area > maxArea)
			{
				maxArea = area;
				maxNum = i;
				//maxContour = contours[i];
			}
		}
		Mat dst(img.size(), CV_8UC1, Scalar::all(0));
		Rect maxRect = boundingRect(contours[maxNum]);
		drawContours(dst, contours, maxNum, Scalar(255), -1, 8);
		rectangle(dst, maxRect, Scalar(255));
		int pickNum = 0;
		double pickX = 0;
		double pickY = 0;
		for (int j = maxRect.x; j < (maxRect.x + maxRect.width); j++)
		{
			for (int k = maxRect.y; k < (maxRect.y + maxRect.height); k++)
			{
				if (dst.at<uchar>(cv::Point(j, k)))
				{
					pickNum++;
					pickX += j;
					pickY += k;
				}
			}
		}
		centerPoint.x = cvRound(pickX / pickNum);
		centerPoint.y = cvRound(pickY / pickNum);

		return dst;
	}
	//从路径中读取先后标定的图片，通过差异寻找点。
	void FixPoint::diffGetPoint(System::String^ s1, System::String^ s2,
		System::String^ s3, System::String^ s4)
	{
		IntPtr ptrPath1 = Marshal::StringToHGlobalAnsi(s1);
		char* pPath1 = static_cast<char*>(ptrPath1.ToPointer());
		IntPtr ptrPath2 = Marshal::StringToHGlobalAnsi(s2);
		char* pPath2 = static_cast<char*>(ptrPath2.ToPointer());
		IntPtr ptrPath3 = Marshal::StringToHGlobalAnsi(s3);
		char* pPath3 = static_cast<char*>(ptrPath3.ToPointer());
		IntPtr ptrPath4 = Marshal::StringToHGlobalAnsi(s4);
		char* pPath4 = static_cast<char*>(ptrPath4.ToPointer());
		Mat src1 = imread(pPath1, CV_LOAD_IMAGE_GRAYSCALE);
		Mat src2 = imread(pPath2,CV_LOAD_IMAGE_GRAYSCALE);
		Mat src3 = imread(pPath3,CV_LOAD_IMAGE_GRAYSCALE);
		Mat src4 = imread(pPath4,CV_LOAD_IMAGE_GRAYSCALE);
		Mat ds1_2, ds2_3, ds3_4;
		absdiff(src1, src2, ds1_2);
		absdiff(src2, src3, ds2_3);
		absdiff(src3, src4, ds3_4);
		threshold(ds1_2, ds1_2, 40, 255, THRESH_BINARY);
		threshold(ds2_3, ds2_3, 40, 255, THRESH_BINARY);
		threshold(ds3_4, ds3_4, 40, 255, THRESH_BINARY);
		Mat element = getStructuringElement(MORPH_ELLIPSE, cv::Size(11, 11));
		morphologyEx(ds1_2, ds1_2, MORPH_CLOSE, element);
		morphologyEx(ds2_3, ds2_3, MORPH_CLOSE, element);
		morphologyEx(ds3_4, ds3_4, MORPH_CLOSE, element);
		Mat S1, S2, S3;
		cv::Point t1, t2, t3;
		S1 = maxConnectedCompents(ds1_2, t1);
		S2 = maxConnectedCompents(ds2_3, t2);
		S3 = maxConnectedCompents(ds3_4, t3);
		t1_x = t1.x;
		t1_y = t1.y;
		t2_x = t2.x;
		t2_y = t2.y;
		t3_x = t3.x;
		t3_y = t3.y;
	/*	Mat grayImg1;
		Mat grayImg2;
		Mat grayImg3;
		Mat grayImg4;
		cvtColor(src1, grayImg1, COLOR_BGR2GRAY);
		cvtColor(src2, grayImg2, COLOR_BGR2GRAY);
		cvtColor(src3, grayImg3, COLOR_BGR2GRAY);
		cvtColor(src4, grayImg4, COLOR_BGR2GRAY);*/
	}
	
	
	
	//根据图像中三个点位置及其在物理坐标系的位置，求得图像坐标到实际物理坐标系转换的矩阵。
	//其中，(x1,y1),(x2,y2),(x3,y3)为图像坐标系下的点的坐标，（a1,b1）,(a2,b2),(a3,b3)为物理坐标中的点的坐标。
    void FixPoint::transForm(double x1, double y1, double  x2, double y2, double x3, double y3, double a1, double b1, double a2, double b2, double a3, double b3)
	{
		Mat warp_mat(2, 3, CV_32FC1);
		Point2f strTri[3];
		strTri[0] = Point2f(x1, y1);
		strTri[1] = Point2f(x2, y2);
		strTri[2] = Point2f(x3, y3);
		Point2f dstTri[3];                                                                                                                                                                                                                                                                                                                                                                                                               
		dstTri[0] = Point2f(a1, b1);
		dstTri[1] = Point2f(a2, b2);
		dstTri[2] = Point2f(a3, b3);
		warp_mat = getAffineTransform(strTri, dstTri);

		aa1 = warp_mat.at<double>(0,0);
		bb1 = warp_mat.at<double>(0, 1);
		tt1 = warp_mat.at<double>(0, 2);
		aa2 = warp_mat.at<double>(1, 0);
		bb2 = warp_mat.at<double>(1, 1);
		tt2 = warp_mat.at<double>(1, 2);
	}
	/***********
	getRealMid在求得两个图像坐标中的点，并转换为实际物理坐标后，求得检测目标的中心点
	****************/
	void FixPoint::getRealMid(double center1_x, double center1_y, double center2_x, double center2_y)
	{
		midPos_X = (center1_x + center2_x) / 2;
	    midPos_Y=(center1_y+center2_y)/2;
	}
	/*******
	pictureToWorld将图片中的坐标转换到实际物理坐标中。a1,a2,t1,a2,b2,t2为该镜头中求得的变换坐标系。
	*******/
	void FixPoint::pictureToWorld(double a1, double b1, double t1, double a2, double b2, double t2,double XX,double YY)
	{
		posOfRealWorld_X = XX*a1 + YY*b1 + t1;
		posOfRealWorld_Y = XX*a2 + YY*b2 + t2;
	}
	//getAngleToHorizon求取图像中交点坐标与寻找到线段中点连线的方位角，取其与x轴夹角较小的为所求角
	void FixPoint::getAngleToHorizon() 
	{
		//double angle1 = acos((midPoint.x1 - XY.x)/ sqrt((midPoint.x1 - XY.x)*(midPoint.x1 - XY.x)+ (midPoint.y1 - XY.y)*(midPoint.y1 - XY.y)))*180/CV_PI;
		//double angle2 = acos((midPoint.x2 - XY.x) / sqrt((midPoint.x2 - XY.x)*(midPoint.x2 - XY.x) + (midPoint.y2 - XY.y)*(midPoint.y2 - XY.y)))*180/CV_PI;
		double angle1 = atan2(y1 - y, x1 - x) * 180 / CV_PI;
		double angle2 = atan2(y2 - y, x2 - x) * 180 / CV_PI;
		double angle_1 = abs(angle1);
		double angle_2= abs(angle2);
		if (angle_1 > 90)
			angle_1 = 180-angle_1;
		if (angle_2 > 90)
			angle_2 = 180-angle_2;
		if (angle1 >= angle2)
			angleToHorizon = angle2;
		else
			angleToHorizon = angle1;
	}
	int FixPoint::getFixPoint(System::String^ strPath, int pointAX, int pointAY, int pointBx, int PointBy,
		int pointCX, int pointCY, int pointDx, int PointDy, System::Drawing::Bitmap^ %vis)
	{
		int rc = 0;
		IntPtr ptrPath = Marshal::StringToHGlobalAnsi(strPath);
		char* pPath = static_cast<char*>(ptrPath.ToPointer());
		cv::Mat srcImg;
		cv::Mat grayImg;
		cv::Mat cloneImg;
		try
		{
			srcImg = imread(pPath, CV_LOAD_IMAGE_COLOR);
			if (srcImg.empty())
			{
			rc = __LINE__;
			printf("can't load image: line=%d\n", rc);
			/*int j = ConvertBitmapToMat(via, srcImg);
			if (j)
			{
				rc = __LINE__;*/
				return rc;
			}
			FindLine flA, flB;
			if (rc == 0)
			{
				//cloneImg = srcImg.clone();

				cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);
				adaptiveThreshold(grayImg, grayImg, 255,
					ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 0);
				flA = FindLine(Point2f(pointAX, pointAY), Point2f(pointBx, PointBy));
				flA.bIsPairMode = false;
				flA.eMajorPolarity = ANY;
				flA.fAcceptFitError = 1.5;
				flA.fSearchDir = 90;
				flA.nProjectionLength = 30;
				flA.nNumOfCaliper = 20;
				flA.nNumToIgnoreAllowed = 5;
				flA.nSearchLenght = 40;

				flB = FindLine(Point2f(pointCX, pointCY), Point2f(pointDx, PointDy));
				flB.bIsPairMode = false;
				flB.eMajorPolarity = ANY;
				flB.fAcceptFitError = 1.5;
				flB.fSearchDir = 90;
				flB.nProjectionLength = 20;
				flB.nNumOfCaliper = 20;
				flB.nNumToIgnoreAllowed = 5;
				flB.nNumToIgnoreAllowed = 40;
			}
			FindLine::LineSeg lsA, lsB;
			FindLine::Line lineA, lineB;
			int rc1 = flA.run(grayImg, PoseXYR(), lsA, lineA);
			int rc2 = flB.run(grayImg, PoseXYR(), lsB, lineB);
			if (flA.Success() && flB.Success())
			{
				line(srcImg, cv::Point(cvRound(lsA.start.x), cvRound(lsA.start.y)), cv::Point(cvRound(lsA.end.x), cvRound(lsA.end.y)), Scalar(0, 0, 255), 3, 8);
				line(srcImg, cv::Point(cvRound(lsB.start.x), cvRound(lsB.start.y)), cv::Point(cvRound(lsB.end.x), cvRound(lsB.end.y)), Scalar(0, 0, 255), 3, 8);
				int X = cvRound((lineA.ny*lineA.ny*(lineA.y - lineB.y) + lineA.nx*lineB.ny*lineA.x - lineB.nx*lineA.ny*lineB.x) / (lineA.nx*lineB.ny - lineB.nx*lineA.ny));
				int Y = cvRound((lineA.nx*lineB.nx*(lineA.x - lineB.x) + lineB.nx*lineA.ny*lineA.y - lineA.nx*lineB.ny*lineB.y) / (-lineA.nx*lineB.ny + lineB.nx*lineA.ny));
				circle(srcImg, cv::Point(X, Y), 4, Scalar(0, 0, 255), -1, 8);
				x = X;
				y = Y;
				x1 = (cvRound(lsA.start.x) + cvRound(lsA.end.x)) / 2;
				y1 = (cvRound(lsA.start.y) + cvRound(lsA.end.y)) / 2;
				x2 = (cvRound(lsB.start.x) + cvRound(lsB.end.x)) / 2;
				y2 = (cvRound(lsB.start.y) + cvRound(lsB.end.y)) / 2;
			}
			else
			{
				rc = __LINE__;
			}


		}
		catch (cv::Exception & exc)
		{
			rc = __LINE__;
			throw std::exception("can't load image");
		}
		vis = ConverToBitmap(&srcImg);
		return rc;
	}

	int FixPoint::getFixPoint(System::Drawing::Bitmap^ via,int pointAX, int pointAY, int pointBx, int PointBy,
			int pointCX, int pointCY, int pointDx, int PointDy, System::Drawing::Bitmap^ %vis)
	{
		int rc = 0;
		/*IntPtr ptrPath = Marshal::StringToHGlobalAnsi(strPath);
		char* pPath = static_cast<char*>(ptrPath.ToPointer());*/
		cv::Mat srcImg;
		cv::Mat grayImg;
		cv::Mat cloneImg;
		try
		{
		/*	srcImg = imread(pPath, CV_LOAD_IMAGE_COLOR);
			if (srcImg.empty())
			{
				rc = __LINE__;
				printf("can't load image: line=%d\n", rc);*/
			int j = ConvertBitmapToMat(via, srcImg);
		//	std::cout << "Channels of srcImg: " << srcImg.channels()<<std::endl;
		//	cv::imwrite("F:\\TEST.bmp", srcImg);
			if(j)
			{
				rc = __LINE__;
				return rc;
			}
			//cloneImg = srcImg.clone();
			
			if (via->PixelFormat == System::Drawing::Imaging::PixelFormat::Format8bppIndexed)	// 灰度图像
			{
				grayImg = srcImg.clone();
			}
			else if (via->PixelFormat == System::Drawing::Imaging::PixelFormat::Format24bppRgb)	// 彩色图像
			{
				cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
			}
			FindLine flA, flB;
			if (rc == 0)
			{
				//cloneImg = srcImg.clone();

				//cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
				
				cv::adaptiveThreshold(grayImg, grayImg, 255,
					cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0);
				flA = FindLine(cv::Point2f(pointAX, pointAY), cv::Point2f(pointBx, PointBy));
				flA.bIsPairMode = false;
				flA.eMajorPolarity = ANY;
				flA.fAcceptFitError = 1.5;
				flA.fSearchDir = 90;
				flA.nProjectionLength = 30; //原先30
				flA.nNumOfCaliper = 20;  //原先20
				flA.nNumToIgnoreAllowed = 10;
				flA.nSearchLenght = 40; //原先40

				flB = FindLine(Point2f(pointCX, pointCY), Point2f(pointDx, PointDy));
				flB.bIsPairMode = false;
				flB.eMajorPolarity = ANY;
				flB.fAcceptFitError = 1.5;
				flB.fSearchDir = 90;
				flB.nProjectionLength = 20;  //原先20
				flB.nNumOfCaliper =20;   //原先20
				//flB.nNumToIgnoreAllowed = 5;
				flB.nNumToIgnoreAllowed = 20;
				flB.nSearchLenght = 40; //原先40
			}
			FindLine::LineSeg lsA, lsB;
			FindLine::Line lineA, lineB;
		    int rc1 = flA.run(grayImg, PoseXYR(), lsA, lineA);
	        int rc2 = flB.run(grayImg, PoseXYR(), lsB, lineB);
			if (flA.Success() && flB.Success())
			{
				if	(via->PixelFormat == System::Drawing::Imaging::PixelFormat::Format8bppIndexed)	// 灰度图像
				{
					cv::line(srcImg, cv::Point(cvRound(lsA.start.x), cvRound(lsA.start.y)), cv::Point(cvRound(lsA.end.x), cvRound(lsA.end.y)), Scalar(255), 3, 8);
					cv::line(srcImg, cv::Point(cvRound(lsB.start.x), cvRound(lsB.start.y)), cv::Point(cvRound(lsB.end.x), cvRound(lsB.end.y)), Scalar(255), 3, 8);
					int X = cvRound((lineA.ny*lineA.ny*(lineA.y - lineB.y) + lineA.nx*lineB.ny*lineA.x - lineB.nx*lineA.ny*lineB.x) / (lineA.nx*lineB.ny - lineB.nx*lineA.ny));
					int Y = cvRound((lineA.nx*lineB.nx*(lineA.x - lineB.x) + lineB.nx*lineA.ny*lineA.y - lineA.nx*lineB.ny*lineB.y) / (-lineA.nx*lineB.ny + lineB.nx*lineA.ny));
					cv::circle(srcImg, cv::Point(X, Y), 8, Scalar(255), -1, 8);
					x = X;
				    y = Y;
					x1 = (cvRound(lsA.start.x) + cvRound(lsA.end.x)) / 2;
					y1 = (cvRound(lsA.start.y) + cvRound(lsA.end.y)) / 2;
					x2 = (cvRound(lsB.start.x) + cvRound(lsB.end.x)) / 2;
				    y2 = (cvRound(lsB.start.y) + cvRound(lsB.end.y)) / 2;

				}
				else if (via->PixelFormat == System::Drawing::Imaging::PixelFormat::Format24bppRgb)	// 彩色图像
				{
				cv::line(srcImg, cv::Point(cvRound(lsA.start.x), cvRound(lsA.start.y)), cv::Point(cvRound(lsA.end.x), cvRound(lsA.end.y)), Scalar(0, 0, 255), 3, 8);
				cv::line(srcImg, cv::Point(cvRound(lsB.start.x), cvRound(lsB.start.y)), cv::Point(cvRound(lsB.end.x), cvRound(lsB.end.y)), Scalar(0, 0, 255), 3, 8);
				int X = cvRound((lineA.ny*lineA.ny*(lineA.y - lineB.y) + lineA.nx*lineB.ny*lineA.x - lineB.nx*lineA.ny*lineB.x) / (lineA.nx*lineB.ny - lineB.nx*lineA.ny));
				int Y = cvRound((lineA.nx*lineB.nx*(lineA.x - lineB.x) + lineB.nx*lineA.ny*lineA.y - lineA.nx*lineB.ny*lineB.y) / (-lineA.nx*lineB.ny + lineB.nx*lineA.ny));
				cv::circle(srcImg, cv::Point(X,Y), 8, Scalar(0, 0, 255), -1, 8);
				x = X;
				y = Y;
				x1 = (cvRound(lsA.start.x) + cvRound(lsA.end.x)) / 2;
				y1 = (cvRound(lsA.start.y) + cvRound(lsA.end.y)) / 2;
				x2 = (cvRound(lsB.start.x) + cvRound(lsB.end.x)) / 2;
				y2 = (cvRound(lsB.start.y) + cvRound(lsB.end.y)) / 2;
				}
			}
			else 
			{
				rc = __LINE__;
			}
	

		}
		catch (cv::Exception & exc)
		{
			rc = __LINE__;
			throw std::exception("can't load image");
		}
		vis = ConverToBitmap(&srcImg);
		return rc;
  }
	System::Drawing::Bitmap ^ ConverToBitmap(cv::Mat * src)
	{
		// bitmap 初始化
		System::Drawing::Imaging::PixelFormat dstFormat = System::Drawing::Imaging::PixelFormat::Format24bppRgb;
		switch (src->channels())
		{
		case 1:
			dstFormat = System::Drawing::Imaging::PixelFormat::Format8bppIndexed;
			break;
		case 3:
			dstFormat = System::Drawing::Imaging::PixelFormat::Format24bppRgb;
			break;
		default:
			throw __LINE__;
			break;
		}
		System::Drawing::Bitmap ^dst = gcnew System::Drawing::Bitmap(src->cols, src->rows, dstFormat);

		if (src->channels() == 1)
		{
			ColorPalette^ colorPalette = dst->Palette;
			for (int i = 0; i < 256; i++)
			{
				colorPalette->Entries[i] = Color::FromArgb(i, i, i);
			}
			dst->Palette = colorPalette;
		}

		// 获取 bitmap 数据指针  
		System::Drawing::Imaging::BitmapData ^data = dst->LockBits(
			*(gcnew System::Drawing::Rectangle(0, 0, dst->Width, dst->Height)),
			System::Drawing::Imaging::ImageLockMode::ReadWrite,
			dstFormat
		);
		// 复制图像数据  
		for (int i = 0; i<src->rows; i++)
			memcpy((char*)data->Scan0.ToPointer() + i * data->Stride, src->data + i * src->step, src->cols * src->elemSize());

		// 解除 bitmap 数据保护  
		dst->UnlockBits(data);
		return dst;
	}
	int ConvertBitmapToMat(System::Drawing::Bitmap^ bmpImg, cv::Mat& cvImg)
	{
		int retVal = 0;

		//锁定Bitmap数据
		System::Drawing::Imaging::BitmapData^ bmpData = bmpImg->LockBits(
			System::Drawing::Rectangle(0, 0, bmpImg->Width, bmpImg->Height),
			System::Drawing::Imaging::ImageLockMode::ReadWrite, bmpImg->PixelFormat);

		//若cvImg非空，则清空原有数据
		if (!cvImg.empty())
		{
			cvImg.release();
		}

		//将 bmpImg 的数据指针复制到 cvImg 中，不拷贝数据
		if (bmpImg->PixelFormat == System::Drawing::Imaging::PixelFormat::Format8bppIndexed)	// 灰度图像
		{
			cvImg = cv::Mat(bmpImg->Height, bmpImg->Width, CV_8UC1, (char*)bmpData->Scan0.ToPointer());
		}
		else if (bmpImg->PixelFormat == System::Drawing::Imaging::PixelFormat::Format24bppRgb)	// 彩色图像
		{
			cvImg = cv::Mat(bmpImg->Height, bmpImg->Width, CV_8UC3, (char*)bmpData->Scan0.ToPointer());
		}

		//解锁Bitmap数据
		bmpImg->UnlockBits(bmpData);

		return (retVal);
	}
	
	int FixPoint::getLineA(Mat img, int pointAX, int pointAY, int pointBx, int PointBy, FindLine::Line & lineA)
	{
		int rc = 0;
		FindLine flA;
		flA = FindLine(Point2f(pointAX, pointAY), Point2f(pointBx, PointBy));
		flA.bIsPairMode = false;
		flA.eMajorPolarity = ANY;
		flA.fAcceptFitError = 1.5;
		flA.fSearchDir = 0;
		flA.nProjectionLength = 30;
		flA.nNumOfCaliper = 20;
		flA.nNumToIgnoreAllowed = 5;
		flA.nSearchLenght = 40;
		FindLine::LineSeg lsA;
	//	FindLine::Line lineA;
	    rc = flA.run(img, PoseXYR(), lsA, lineA);
		if (!flA.Success())
		{
			rc = __LINE__;
		}
		return rc;
	}
	int FixPoint::getLineB(Mat img, int pointAX, int pointAY, int pointBx, int PointBy, FindLine::Line & lineB)
	{
		int rc = 0;
		FindLine flB;
		flB = FindLine(Point2f(pointAX, pointAY), Point2f(pointBx, PointBy));
		flB.bIsPairMode = false;
		flB.eMajorPolarity = ANY;
		flB.fAcceptFitError = 1.5;
		flB.fSearchDir = 0;
		flB.nProjectionLength = 30;
		flB.nNumOfCaliper = 20;
		flB.nNumToIgnoreAllowed = 5;
		flB.nSearchLenght = 40;
		FindLine::LineSeg lsB;
		//FindLine::Line lineB;
		rc = flB.run(img, PoseXYR(), lsB, lineB);
		if (!flB.Success())
		{
			rc = __LINE__;
		}
		return rc;
	}


}