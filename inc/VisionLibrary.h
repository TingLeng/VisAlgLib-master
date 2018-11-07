#pragma once

#include "stdio.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"


#ifndef DLLEXPORT
	#ifdef _USRDLL
		#define DLLEXPORT _declspec(dllexport)
	#else
		#define DLLEXPORT _declspec(dllimport)
	#endif
#endif

#ifndef OK
#define OK 0
#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif


#ifndef INOUT
#define INOUT
#endif




namespace VisionLibrary
{
	typedef enum _PolarityEnum
	{
		DARK_TO_LIGHT,
		LIGHT_TO_DARK,
		ANY
	}PolarityEnum;

	class PoseXYR
	{
	public:
		float x;
		float y;
		float rz;

		PoseXYR() :x(0.0), y(0.0), rz(0.0) {}
		PoseXYR(float _x, float _y, float _rz) :x(_x), y(_y), rz(_rz) {}
		PoseXYR(float _x, float _y) : x(_x), y(_y), rz(0.0) {}

		cv::Point2f pointGlobalToLocal(cv::Point2f src)
		{
			float _x = src.x - x;
			float _y = src.y - y;

			double m11 = cos(-rz*CV_PI / 180);
			double m21 = sin(-rz*CV_PI / 180);
			double m12 = -m21;
			double m22 = m11;

			return cv::Point2f(m11*_x + m12*_y, m21*_x + m22*_y);

		}

		cv::Point2f pointLocalToGlobal(cv::Point2f src)
		{
			double m11 = cos(rz*CV_PI / 180);
			double m21 = sin(rz*CV_PI / 180);
			double m12 = -m21;
			double m22 = m11;

			return cv::Point2f(m11*src.x + m12*src.y + x, m21*src.x + m22*src.y + y);
		}

		cv::Point2f vectorGlobalToLocal(cv::Point2f src)
		{
			cv::Point2f pt1 = pointGlobalToLocal(cv::Point2f(0.0, 0.0));
			cv::Point2f pt2 = pointGlobalToLocal(src);
			return cv::Point2f(pt2.x-pt1.x, pt2.y-pt1.y);
		}

		cv::Point2f vectorLocalToGlobal(cv::Point2f src)
		{
			cv::Point2f pt1 = pointLocalToGlobal(cv::Point2f(0.0, 0.0));
			cv::Point2f pt2 = pointLocalToGlobal(src);
			return cv::Point2f(pt2.x - pt1.x, pt2.y - pt1.y);
		}


	};




}

