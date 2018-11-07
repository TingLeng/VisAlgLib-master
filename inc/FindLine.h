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

	// ÿ��caliper����������������(CaliperResult)
	// ��ֱ��ʱֻ����ÿ��caliper��һ�����
	// ��ö��ָ���˴Ӷ��CaliperResult��ѡ��ԭ��
	enum ChooseCaliperResult
	{
		STRONGEST,// ���ݶ���ǿ��
		FIRST,    // ���������������һ�����ֵ�
		LAST      // �����������������һ�����ֵ�
	};

public:

	// �㷨ʽ����ֱ�߲��������޳���
	struct Line
	{
		double nx; //������
		double ny;
		double x;//��
		double y;
	};

	// �߶Σ��������ޣ�
	struct LineSeg
	{
		cv::Point2f	start;
		cv::Point2f end;
	};

	// ��ֱ���㷨�����Caliper
	// ��ֱ���㷨��������ҵ��������߶Σ�������ÿ��Caliper�Ľ��������Ա�����ϡ�������ʾ��
	// �ýṹ����CaliperResultΪ������������һЩ�������
	struct FindLineCaliperResult
	{
		int idx;
		bool bValid; // caliper����Ƿ���Ч��
		bool bUsed;  // �Ƿ�����ֱ����ϣ��ִ��Ϊfalse)
		float x;     // ��ͼ���еľ�ȷ����
		float y;

		//cv::Point2f nominalPos; // �������꣬���ڽ�����ʾ
		//float fSearchDir;       // �����������ڽ�����ʾ

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

	

	// ptStart����㣬ptEnd���յ�
	FindLine(cv::Point2f ptStart, cv::Point2f ptEnd);

	FindLine(LineSeg ls);

	// _ref���ο�����ѵ��ͼ���е�λ��
	// ptStart�������ѵ��ͼ���е�λ��
	// ptEnd���յ���ѵ��ͼ���е�λ��
	FindLine(PoseXYR _ref, cv::Point2f ptStart, cv::Point2f ptEnd);



	~FindLine();

	// ���к���
	// img����ǰͼ��
	// _ref �ο����ڵ�ǰͼ���е�λ��
	// seg �߶����
	// line �����
	int run(IN cv::Mat img, IN PoseXYR _ref, OUT LineSeg &seg, OUT Line& line/* ,
		OUT FindLineCaliperResult* pCaliperResult = NULL, INOUT int nCapaicy = 0 */);

	void getCaliperResult(std::vector<FindLineCaliperResult> & calipers);

	cv::Mat getProfile(int idx);

	cv::Mat getCaliperImage(int idx);

	bool Success();

	cv::Point2i ptStart;   // �߶�������㣬������ͼ������ϵ�� 
	cv::Point2i ptEnd;     // �߶������յ㣬������ͼ������ϵ��
	int nNumOfCaliper;     // caliper�ĸ��� 
	int nSearchLenght;     // �������ȣ���λ���ء�����Ŀ���߶���ͼ���ڿ��ܵ�ƫ�����趨��
	int nProjectionLength; // ͶӰ���ȣ���λ����
	float fSearchDir;      // �������򣬵�λ�ȣ���ͼ������ϵ��
	bool bIsPairMode;      // �Ƿ�Ϊ˫��ģʽ
	PolarityEnum eMajorPolarity; // �ߵļ���
	PolarityEnum eSecondaryPolarity;
	float fPairWidthMin;// ˫��ģʽ��������Сֵ����λ����
	float fPairWidthMax;


	float fContrastThresh;;// �Աȶ���ֵ����λ�ҽ�
	float fFilterHalfWindow; //�˲����ڴ�С�����

	PoseXYR ref; // �ο�����ϵ 

	int nNumToIgnoreAllowed; // ������Ե�caliper����

	float fAcceptFitError;

	ChooseCaliperResult m_eChooseStrategy;


private:

	std::vector<FindLineCaliperResult> m_caliperResults;
	bool m_bSuc;


	std::vector<Caliper> vCalipers;

};

}

