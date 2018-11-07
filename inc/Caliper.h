#pragma once

#include <vector>
#include <iostream>
#include <memory>
#include "cv.h"
#include "VisionLibrary.h"

namespace VisionLibrary
{
	class __declspec(dllexport) Caliper
	{
	public:
		class CaliperResult
		{
		public:

			float fPos; // 边缘点在投影信号上的坐标
			float x;    // 单边模式时，表示边的坐标。双边模式时：双边的中心点坐标。描述在图像坐标系中
			float y;

			float fAngle0;//所找到边的角度（边的切向量 ，描述在图像坐标系中）
			float fAngle1;//双边模式时，第二个边角度

			int edge0;//单边模式边的编号，或双边模式第一个边编号。 满足 abs(梯度)>=阈值 的点会有一个编号。
			int edge1;//双边模式第二个边编号

			float fContrast0;// 单边模式对比度，双边模式第一个边对比度
			float fContrast1;//双边模式第二个边对比度

			float fPos0;
			float fPos1;

			float x0;//第一个边的坐标
			float y0;
			float x1;//第二个边的坐标，仅双边模式有效
			float y1;

			int fPairWidth;//实际测量值,仅双边模式


			CaliperResult()
				:/*bValid(false),*/
				fPos(-1),x(-1), y(-1), 
				fAngle0(0), fAngle1(0),
				edge0(-1), edge1(-1), 
				fContrast0(0), fContrast1(0), 
				fPos0(-1), fPos1(-1), 
				x0(0), y0(0), 
				x1(0), y1(0), 
				fPairWidth(0)
			{

			}

		};

	public:
		Caliper();


		// center,在训练图像中,caliper的中心坐标
		Caliper(cv::Point2f center, float fSearchDir);

		// center，caliper的中心坐标（在训练图像中）
		// ref,参考物的位置与姿态（在训练图像中）
		Caliper(PoseXYR _ref, cv::Point2f center, float fSearchDir);

		~Caliper()
		{

		}

		// ref,参考物的位置姿态（当前图像）
		// result，结果存储区
		// nCapacity,输入result容量，输出实际找到的结果个数
		int run(IN cv::Mat img, IN PoseXYR _ref, OUT CaliperResult *result, INOUT int *nCapacity);


		int run(IN cv::Mat img, IN PoseXYR _ref = PoseXYR());



		cv::Mat getProfile();
		cv::Mat getImage();

		bool sucess();

		int getResults(std::vector<CaliperResult> &result);

		int getResult(CaliperResult &result);

		void getROI(cv::Point2f* roi);

		//// 获取上次运行时中心在图像里的坐标
		//cv::Point2f getLastCenter();

		// 获取上次运行时搜索方向，描述在图像坐标系中
		//float getLastSearchDir();



		cv::Point2f center; // 搜索区域的中心点，描述在图像坐标系中（在训练图像中）
		int nSearchLength;  // 搜索的长度（沿着搜索方向，中心点左右各一半），单位像素
		int nProjectionLength;//投影长度，单位像素
		float fSearchDir;     //搜索方向，单位度，在图像坐标系中
		float fProjectionAngleFrom;// 投影角度最小值，以搜索方向在图像中旋转90度后状态为零点
		float fProjectionAngleTo;// 投影角度最大值
		float fProjectionAngleStep;//投影角度步距


		bool bIsPairMode; //是否为双边模式
		PolarityEnum eMajorPolarity; // 边的极性
		PolarityEnum eSecondaryPolarity;
		float fPairWidthMin;// 双边模式，距离最小值，单位像素
		float fPairWidthMax;


		float fContrastThresh;;// 对比度阈值，单位灰阶
		float fFilterHalfWindow; //滤波窗口大小（半宽）

		PoseXYR m_ref;// 参考物体位置姿态（在训练图像中）




	private:
		cv::Mat unwarpImage(cv::Mat src, PoseXYR m_ref, float fProjectionAngle/*, cv::Point2f roi[4]*/);

		cv::Point2f lastCenter;
		float lastSearchDir;

		cv::Mat m_profile;

		cv::Mat m_image;

		std::vector<CaliperResult> m_result;

		void parabolaInterp(float l, float c, float r, float* maxx, float* maxy);



	};
}

