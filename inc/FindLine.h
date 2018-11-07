#pragma once


#include "core.hpp"
#include "highgui.hpp"
#include "video.hpp"

#include "VisionLibrary.h"
#include "Caliper.h"


namespace VisionLibrary
{


class DLLEXPORT FindLine
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
	struct Line
	{
		double nx; //法向量
		double ny;
		double x;//点
		double y;
	};

	// 线段（长度有限）
	struct LineSeg
	{
		cv::Point2f	start;
		cv::Point2f end;
	};

	// 找直线算法输出的Caliper
	// 找直线算法除了输出找到的线与线段，还将其每个Caliper的结果输出，以便于诊断、界面显示等
	// 该结构体以CaliperResult为基础，增加了一些相关内容
	struct FindLineCaliperResult
	{
		int idx;
		bool bValid; // caliper结果是否有效。
		bool bUsed;  // 是否用于直线拟合（粗大点为false)
		float x;     // 在图像中的精确坐标
		float y;

		//cv::Point2f nominalPos; // 名义坐标，用于界面显示
		//float fSearchDir;       // 搜索方向，用于界面显示

		cv::Point2f roi[4];

		Caliper::CaliperResult detailedCaliperResult;

		FindLineCaliperResult()
			:idx(-1), bValid(false), bUsed(false), x(0), y(0)/*, nominalPos(cv::Point2f()), fSearchDir(NAN)*/
		{

		}

	};



public:

	FindLine();

	FindLine& operator=(const FindLine& other);

	

	// ptStart：起点，ptEnd：终点
	FindLine(cv::Point2f ptStart, cv::Point2f ptEnd);

	FindLine(LineSeg ls);

	// _ref：参考物在训练图像中的位置
	// ptStart：起点在训练图像中的位置
	// ptEnd：终点在训练图像中的位置
	FindLine(PoseXYR _ref, cv::Point2f ptStart, cv::Point2f ptEnd);



	~FindLine();

	// 运行函数
	// img，当前图像
	// _ref 参考物在当前图像中的位置
	// seg 线段输出
	// line 线输出
	int run(IN cv::Mat img, IN PoseXYR _ref, OUT LineSeg &seg, OUT Line& line/* ,
		OUT FindLineCaliperResult* pCaliperResult = NULL, INOUT int nCapaicy = 0 */);

	void getCaliperResult(std::vector<FindLineCaliperResult> & calipers);

	cv::Mat getProfile(int idx);

	cv::Mat getCaliperImage(int idx);

	bool Success();

	cv::Point2i ptStart;   // 线段名义起点，描述在图像坐标系下 
	cv::Point2i ptEnd;     // 线段名义终点，描述在图像坐标系下
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

	std::vector<FindLineCaliperResult> m_caliperResults;
	bool m_bSuc;


	std::vector<Caliper> vCalipers;

};

}

