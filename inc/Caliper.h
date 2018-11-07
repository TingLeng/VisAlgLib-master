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

			float fPos; // ��Ե����ͶӰ�ź��ϵ�����
			float x;    // ����ģʽʱ����ʾ�ߵ����ꡣ˫��ģʽʱ��˫�ߵ����ĵ����ꡣ������ͼ������ϵ��
			float y;

			float fAngle0;//���ҵ��ߵĽǶȣ��ߵ������� ��������ͼ������ϵ�У�
			float fAngle1;//˫��ģʽʱ���ڶ����߽Ƕ�

			int edge0;//����ģʽ�ߵı�ţ���˫��ģʽ��һ���߱�š� ���� abs(�ݶ�)>=��ֵ �ĵ����һ����š�
			int edge1;//˫��ģʽ�ڶ����߱��

			float fContrast0;// ����ģʽ�Աȶȣ�˫��ģʽ��һ���߶Աȶ�
			float fContrast1;//˫��ģʽ�ڶ����߶Աȶ�

			float fPos0;
			float fPos1;

			float x0;//��һ���ߵ�����
			float y0;
			float x1;//�ڶ����ߵ����꣬��˫��ģʽ��Ч
			float y1;

			int fPairWidth;//ʵ�ʲ���ֵ,��˫��ģʽ


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


		// center,��ѵ��ͼ����,caliper����������
		Caliper(cv::Point2f center, float fSearchDir);

		// center��caliper���������꣨��ѵ��ͼ���У�
		// ref,�ο����λ������̬����ѵ��ͼ���У�
		Caliper(PoseXYR _ref, cv::Point2f center, float fSearchDir);

		~Caliper()
		{

		}

		// ref,�ο����λ����̬����ǰͼ��
		// result������洢��
		// nCapacity,����result���������ʵ���ҵ��Ľ������
		int run(IN cv::Mat img, IN PoseXYR _ref, OUT CaliperResult *result, INOUT int *nCapacity);


		int run(IN cv::Mat img, IN PoseXYR _ref = PoseXYR());



		cv::Mat getProfile();
		cv::Mat getImage();

		bool sucess();

		int getResults(std::vector<CaliperResult> &result);

		int getResult(CaliperResult &result);

		void getROI(cv::Point2f* roi);

		//// ��ȡ�ϴ�����ʱ������ͼ���������
		//cv::Point2f getLastCenter();

		// ��ȡ�ϴ�����ʱ��������������ͼ������ϵ��
		//float getLastSearchDir();



		cv::Point2f center; // ������������ĵ㣬������ͼ������ϵ�У���ѵ��ͼ���У�
		int nSearchLength;  // �����ĳ��ȣ����������������ĵ����Ҹ�һ�룩����λ����
		int nProjectionLength;//ͶӰ���ȣ���λ����
		float fSearchDir;     //�������򣬵�λ�ȣ���ͼ������ϵ��
		float fProjectionAngleFrom;// ͶӰ�Ƕ���Сֵ��������������ͼ������ת90�Ⱥ�״̬Ϊ���
		float fProjectionAngleTo;// ͶӰ�Ƕ����ֵ
		float fProjectionAngleStep;//ͶӰ�ǶȲ���


		bool bIsPairMode; //�Ƿ�Ϊ˫��ģʽ
		PolarityEnum eMajorPolarity; // �ߵļ���
		PolarityEnum eSecondaryPolarity;
		float fPairWidthMin;// ˫��ģʽ��������Сֵ����λ����
		float fPairWidthMax;


		float fContrastThresh;;// �Աȶ���ֵ����λ�ҽ�
		float fFilterHalfWindow; //�˲����ڴ�С�����

		PoseXYR m_ref;// �ο�����λ����̬����ѵ��ͼ���У�




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

