#pragma once


#include <map>
#include <vector>
#include <string>
#include <memory>
//#include "CVIM.h"
//#include "CV4A_if.h"
//#include "CV4A_tc.h"

#include "cxcore.hpp"
#include "cv.hpp"
#include "highgui.h"


using namespace std;



////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	���Ա任��ʽ </summary>
///
/// <remarks>	, 2017/5/15. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum _TransformationType
{
	RIGID,
	SIMILAR,
	AFFINE
}TransformationType;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	��ֵ�� </summary>
///
/// <remarks>	, 2017/5/15. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum _InterpolationMethod
{
	BILINEAR,
	BICUBIC
}InterpolationMethod;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	����Inspector�������������ּ����</summary>
///
/// <remarks>	, 2017/5/15. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////



class __declspec(dllexport) Inspector
{
public:

	Inspector();

	Inspector(cv::Mat imgInitial, const vector<vector<cv::Point>> &vROIs,
		const vector<vector<cv::Point>> &vIgnoreROIs,
		InterpolationMethod im, float fOffset, float fScale);

	virtual ~Inspector();

	void DoTrain();

	void FeedTrain(cv::Mat img, cv::Mat &HomMat2D);

    void Compare(cv::Mat img,cv::Mat &HomMat2D, vector<cv::Mat>& vAbsDiffImage, vector<cv::Mat>& vDefectImage, vector<cv::Mat>& vIdealImage);

	void UnTrain();

	cv::Mat GetMeanImage(int idx)
	{
		return m_vRefImage[idx];
	}

	cv::Mat GetSigmaImage(int idx)
	{
		return m_vThresholdImage[idx];
	}



public:
	
	int GetImageCount();

	
protected:
	cv::Mat m_imgInitial;

	

	vector<vector<cv::Point> > m_vROIs;
	vector<vector<cv::Point> > m_vIgnoreROIs;
	vector<cv::Rect> m_vRoiBoudingBoxes;
	vector<cv::Mat> m_vBoudingBoxMask;

	vector<cv::Mat> m_vMaps;//[roi id]

	InterpolationMethod m_eInterpolationMethod;
	

	float m_fOffset;
	float m_fScale;

	vector< vector < cv::Mat> > m_vStatisticalImages;//[roi id][statistical image id]
	vector<cv::Mat> m_vRefImage; //[roi id]
	vector<cv::Mat> m_vThresholdImage; //[roi id]


	int m_nTrainImageCount;
};



////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	ͨ��ê�������AnchorPosInspector�� </summary>
///
/// <remarks>	, 2017/5/15. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

class __declspec(dllexport) AnchorPosInspector:public Inspector
{
public:

	AnchorPosInspector();

	//AnchorPosInspector(const AnchorPosInspector& api);



	AnchorPosInspector(
		const vector<cv::Point2f> &vAnchorPos,
		TransformationType tt,
		cv::Mat imgInitial,
		const vector<vector<cv::Point>>& vROIs,
		const vector<vector<cv::Point>> &vIgnoreROIs,
		InterpolationMethod im, 
		float fOffset, 
		float fScale);

	void FeedTrain(cv::Mat img, const vector<cv::Point2f> &vAnchorPos);

	void Compare(cv::Mat img, const vector<cv::Point2f> &vAnchorPos, 
		vector<cv::Mat>& vAbsDiffImage, vector<cv::Mat>& vDefectImage,
		vector<cv::Mat>& vIdealImage);

	virtual ~AnchorPosInspector();
public:
	void getTransformation(const vector<cv::Point2f> & vPoint1, const vector<cv::Point2f> & vPoint2, cv::Mat &HomMat2D);
	static void getRigidTransform(const vector<cv::Point2f> & vPoint1, const vector<cv::Point2f> & vPoint2, cv::Mat &HomMat2D);
	static void getSimilarTransform(const vector<cv::Point2f> & vPoint1, const vector<cv::Point2f> & vPoint2, cv::Mat &HomMat2D);
	static void getAffineTransform(vector<cv::Point2f> & vPoint1, vector<cv::Point2f> & vPoint2, cv::Mat &HomMat2D);
private:
	vector<cv::Point2f> m_vInitialAnchorPos;
	TransformationType m_eTransformType;
};



//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
///// <summary>	pmƥ�����㷨 </summary>
/////
///// <remarks>	, 2017/6/27. </remarks>
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//class PMInspect:public Inspector
//{
//public:
//	PMInspect(cv::Mat imgInitial,vector<cv::Rect> &vROIs,cv::Rect &vPROI,
//		InterpolationMethod im, float fOffset, float fScale );
//	PMInspect(cv::Mat imgInitial,vector<cv::Rect> &vROIs,cv::Rect &vPROI,
//		InterpolationMethod im, float fOffset, float fScale,
//		IN float fCoarseGrand,
//		IN float fFineGrand,
//		IN float fScoreThresh
//	    );
//	virtual ~PMInspect();
//	int GetTransformationFromPM(IN cv::Mat TrainImg,OUT cv::Mat & Transform,IN float fCoarseGrand,
//		IN float fFineGrand ,IN float fScoreThresh ,IN float * vDofRangeStart,IN float * vDofRangeEnd);
//	cv::Point2f Getm_vPROICenter()const;
//	cv::Mat GetPatternImg()const;
//
//public:
//
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//	/// <summary>	��һ�׶Σ�ѵ���׶� </summary>
//	///
//	/// <remarks>	, 2017/7/3. </remarks>
//	///
//	/// <param name="TrainImg">	ѵ����ͼƬMat </param>
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void TrainPhase(IN cv::Mat TrainImg);
//
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//	/// <summary>	�ڶ��׶Σ�ͳ�ƽ׶�. </summary>
//	///
//	/// <remarks>	, 2017/7/3. </remarks>
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void SatisticalPhase();
//
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//	/// <summary>	�����׶Σ��ȶԲ��Խ׶� </summary>
//	///
//	/// <remarks>	, 2017/7/3. </remarks>
//	///
//	/// <param name="TestImg">	����ͼƬ��Mat </param>
//	/// <param name="defects">	ȱ��ͼƬ�ļ��� </param>
//	///
//	/// <returns>	True if it succeeds, false if it fails. </returns>
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	bool ComparePhase( IN cv::Mat TestImg,OUT cv::Mat & defect);
//private:
//
//	/// <summary>	pmƥ���㷨�Ĳ��� </summary>
//	float m_fCoarseGrand;
//	float m_fFineGrand;
//	float m_fScoreThresh;
//	int m_nInfoCode;
//	void * m_model;
//	/// <summary>	pmƥ��ģ����� </summary>
//	cv::Rect m_vPROI;
//	cv::Mat m_imgPattern;
//};