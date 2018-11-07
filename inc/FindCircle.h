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
		struct Circle
		{
			double x; 
			double y;
			double r;
		};

		// �߶Σ��������ޣ�
		struct CircleSeg
		{
			double x;
			double y;
			double startAngle;
			double endAngle;
		};

		// ��ֱ���㷨�����Caliper
		// ��ֱ���㷨��������ҵ��������߶Σ�������ÿ��Caliper�Ľ��������Ա�����ϡ�������ʾ��
		// �ýṹ����CaliperResultΪ������������һЩ�������
		struct FindCircleCaliperResult
		{
			bool bValid; // caliper����Ƿ���Ч��
			bool bUsed;  // �Ƿ�����ֱ����ϣ��ִ��Ϊfalse)
			float x;     // ��ͼ���еľ�ȷ����
			float y;

			cv::Point2f nominalPos; // �������꣬���ڽ�����ʾ
			float fSearchDir;       // �����������ڽ�����ʾ

			Caliper::CaliperResult detailedCaliperResult;

			FindCircleCaliperResult()
				:bValid(false), bUsed(false), x(0), y(0), nominalPos(cv::Point2f()), fSearchDir(NAN)
			{

			}

		};



	public:
		// ptStart����㣬ptEnd���յ�
		FindCircle(cv::Point2f ptStart, cv::Point2f ptEnd);

		// _ref���ο�����ѵ��ͼ���е�λ��
		// ptStart�������ѵ��ͼ���е�λ��
		// ptEnd���յ���ѵ��ͼ���е�λ��
		FindCircle(PoseXYR _ref, cv::Point2f ptStart, cv::Point2f ptEnd);

		~FindCircle();

		// ���к���
		// img����ǰͼ��
		// _ref �ο����ڵ�ǰͼ���е�λ��
		// seg �߶����
		// line �����
		int run(IN cv::Mat img, IN PoseXYR _ref, OUT CircleSeg &seg, OUT Circle& line,
			OUT FindCircleCaliperResult* pCaliperResult = NULL, INOUT int nCapaicy = 0);

		cv::Point2f ptNominalCenter;
		double fNominalRadius;
		double fStartAngle;
		double fEndAngle;

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




	};

}

