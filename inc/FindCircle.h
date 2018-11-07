#pragma once
#include <vector>
#include "cxcore.hpp"
#include "cv.hpp"
#include "VisionLibrary.h"
#include "Caliper.h"

namespace VisionLibrary
{


	class DLLEXPORT FindCircle
	{
	public:

		// 每个caliper可能搜索出多个结果(CaliperResult)
		// 找直线时只允许每个caliper出一个结果
		// 该枚举指定了从多个CaliperResult挑选的原则
		enum ChooseCaliperResult
		{
			STRONGEST,// 挑梯度最强的
			FIRST,    // 挑沿着搜索方向第一个出现的
			LAST      // 挑沿着搜索方向最后一个出现的
		};

	public:

		// 点法式理想直线参数（无限长）
		struct Circle
		{
			double x; 
			double y;
			double r;
		};

		// 线段（长度有限）
		struct CircleSeg
		{
			double x;
			double y;
			double startAngle;
			double endAngle;
		};

		// 找直线算法输出的Caliper
		// 找直线算法除了输出找到的线与线段，还将其每个Caliper的结果输出，以便于诊断、界面显示等
		// 该结构体以CaliperResult为基础，增加了一些相关内容
		struct FindCircleCaliperResult
		{
			bool bValid; // caliper结果是否有效。
			bool bUsed;  // 是否用于直线拟合（粗大点为false)
			float x;     // 在图像中的精确坐标
			float y;

			cv::Point2f nominalPos; // 名义坐标，用于界面显示
			float fSearchDir;       // 搜索方向，用于界面显示

			Caliper::CaliperResult detailedCaliperResult;

			FindCircleCaliperResult()
				:bValid(false), bUsed(false), x(0), y(0), nominalPos(cv::Point2f()), fSearchDir(NAN)
			{

			}

		};



	public:
		// ptStart：起点，ptEnd：终点
		FindCircle(cv::Point2f ptStart, cv::Point2f ptEnd);

		// _ref：参考物在训练图像中的位置
		// ptStart：起点在训练图像中的位置
		// ptEnd：终点在训练图像中的位置
		FindCircle(PoseXYR _ref, cv::Point2f ptStart, cv::Point2f ptEnd);

		~FindCircle();

		// 运行函数
		// img，当前图像
		// _ref 参考物在当前图像中的位置
		// seg 线段输出
		// line 线输出
		int run(IN cv::Mat img, IN PoseXYR _ref, OUT CircleSeg &seg, OUT Circle& line,
			OUT FindCircleCaliperResult* pCaliperResult = NULL, INOUT int nCapaicy = 0);

		cv::Point2f ptNominalCenter;
		double fNominalRadius;
		double fStartAngle;
		double fEndAngle;

		int nNumOfCaliper;     // caliper的个数 
		int nSearchLenght;     // 搜索长度，单位像素。根据目标线段在图像内可能的偏移量设定。
		int nProjectionLength; // 投影长度，单位像素
		float fSearchDir;      // 搜索方向，单位度，在图像坐标系中
		bool bIsPairMode;      // 是否为双边模式
		PolarityEnum eMajorPolarity; // 边的极性
		PolarityEnum eSecondaryPolarity;
		float fPairWidthMin;// 双边模式，距离最小值，单位像素
		float fPairWidthMax;


		float fContrastThresh;;// 对比度阈值，单位灰阶
		float fFilterHalfWindow; //滤波窗口大小（半宽）

		PoseXYR ref; // 参考坐标系 

		int nNumToIgnoreAllowed; // 允许忽略的caliper个数

		float fAcceptFitError;

		ChooseCaliperResult m_eChooseStrategy;


	private:




	};

}

