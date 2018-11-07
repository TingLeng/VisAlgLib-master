#include <fstream>
#include "core.hpp"  
#include "imgcodecs.hpp" 
#include "imgproc.hpp"

#include "filter.h"

#include "Caliper.h"



namespace VisionLibrary
{

	Caliper::Caliper(cv::Point2f center, float fSearchDir)
		:center(center), nSearchLength(30), nProjectionLength(10), fSearchDir(fSearchDir),
		fProjectionAngleFrom(-10.0), fProjectionAngleTo(10.0), fProjectionAngleStep(1.0),
		bIsPairMode(false), eMajorPolarity(PolarityEnum::ANY),
		eSecondaryPolarity(PolarityEnum::ANY), fPairWidthMin(5), fPairWidthMax(60),
		fContrastThresh(10), fFilterHalfWindow(3.0), m_ref(PoseXYR(0,0)), lastCenter(center), 
		lastSearchDir(fSearchDir)
	{
	}

	Caliper::Caliper(PoseXYR _ref, cv::Point2f center, float fSearchDir)
		:center(center), nSearchLength(30), nProjectionLength(10), fSearchDir(fSearchDir),
		fProjectionAngleFrom(-10.0), fProjectionAngleTo(10.0), fProjectionAngleStep(1.0),
		bIsPairMode(false), eMajorPolarity(PolarityEnum::ANY),
		eSecondaryPolarity(PolarityEnum::ANY), fPairWidthMin(5), fPairWidthMax(60),
		fContrastThresh(10), fFilterHalfWindow(3.0), m_ref(_ref), lastCenter(center),
		lastSearchDir(fSearchDir)
	{
	}






	Caliper::Caliper()
		:center(cv::Point(0, 0)), nSearchLength(30), nProjectionLength(10), fSearchDir(fSearchDir),
		fProjectionAngleFrom(-10.0), fProjectionAngleTo(10.0), fProjectionAngleStep(1.0),
		bIsPairMode(false), eMajorPolarity(PolarityEnum::ANY),
		eSecondaryPolarity(PolarityEnum::ANY), fPairWidthMin(5), fPairWidthMax(60),
		fContrastThresh(10), fFilterHalfWindow(3.0), m_ref(PoseXYR(0, 0)), lastCenter(cv::Point(0, 0)),
		lastSearchDir(0)
	{

	}



	int Caliper::run(IN cv::Mat img, IN PoseXYR _ref)
	{
		static int nRunCount = 0;
		int rc = OK;
		int i, j;
		//int nFoundResult = 0;


		m_result.clear();


		if (rc == OK)
		{
			if (img.channels() != 1)
			{
				rc = __LINE__;
			}
		}

		//////////////////////////////////////////////////////////////////////////
		// 确定在当前图像上，搜索原点和搜索方向
		//////////////////////////////////////////////////////////////////////////
		cv::Point2f p = m_ref.pointGlobalToLocal(center);
		lastCenter = _ref.pointLocalToGlobal(p);
		cv::Point2f v = m_ref.vectorGlobalToLocal(cv::Point2f(cos(fSearchDir*CV_PI / 180.0), sin(fSearchDir*CV_PI / 180.0)));
		cv::Point2f cv = _ref.vectorLocalToGlobal(v);
		lastSearchDir = atan2(cv.y, cv.x) * 180 / CV_PI;



		if (rc == OK)
		{
			if (fProjectionAngleFrom > fProjectionAngleTo)
			{
				std::swap(fProjectionAngleFrom, fProjectionAngleTo);
			}
		}

		//////////////////////////////////////////////////////////////////////////
		// 按每个投影角度投影，结果存放于prj，每行对应一个投影角度
		// 每个投影角度对应的ROI存储起始地址 roiForEachAngle[angleIdx*4]
		//////////////////////////////////////////////////////////////////////////
		int nProjectionAngleSteps = floor((fProjectionAngleTo - fProjectionAngleFrom) / fProjectionAngleStep) + 1;
		cv::Mat prj(nProjectionAngleSteps, nSearchLength, CV_32FC1);
		//std::vector<cv::Point2f> roiForEachAngle(nProjectionAngleSteps * 4);
		if (rc == OK)
		{
			for (int idx = 0; idx < nProjectionAngleSteps; idx++)
			{
				float a = fProjectionAngleFrom + fProjectionAngleStep * idx;

				cv::Mat unwarp = unwarpImage(img, _ref, a/*, &roiForEachAngle[idx*4]*/);

				/*cv::imwrite("./test.bmp", unwarp);*/

				m_image = unwarp;

				cv::reduce(unwarp, prj.row(idx), 0, cv::ReduceTypes::REDUCE_AVG, CV_32F);

				// filter 
				if (rc == OK)
				{
					gaussianFilter1D<float, float>(prj.ptr<float>(idx),
						prj.cols, prj.ptr<float>(idx), prj.step, fFilterHalfWindow / 3.0);
				}
			}
		}

		//////////////////////////////////////////////////////////////////////////
		// calculate diff 
		//////////////////////////////////////////////////////////////////////////
		cv::Mat diff;
		if (rc == OK)
		{
			diff = cv::Mat::zeros(prj.rows, prj.cols, prj.type());
			for (int idx = 0; idx < diff.rows; idx++)
			{
				float* src = prj.ptr<float>(idx);
				float* dst = diff.ptr<float>(idx);

				for (size_t i = 1; i < diff.cols - 1; i++)
				{
					dst[i] = (src[i + 1] - src[i - 1]) / 2.0;
				}
			}
		}
		//////////////////////////////////////////////////////////////////////////
		// maximum diff among different projection angles
		//////////////////////////////////////////////////////////////////////////
		cv::Mat vDiffOfIndex;
		cv::Mat vAngleOfIndex;
		cv::Mat vAngleIdxOfIndex;
		if (rc == OK)
		{
			vDiffOfIndex = cv::Mat::zeros(1, diff.cols, diff.type());
			vAngleOfIndex = cv::Mat::zeros(vDiffOfIndex.size(), CV_32FC1);
			vAngleIdxOfIndex = cv::Mat::zeros(vDiffOfIndex.size(), CV_32SC1);

			float* pMaxDiff = vDiffOfIndex.ptr<float>();
			float* pMaxDiffAngle = vAngleOfIndex.ptr<float>();
			int * pMaxDiffAngleIdx = vAngleIdxOfIndex.ptr<int>();

			for (int angleIdx = 0; angleIdx < diff.rows; angleIdx++)
			{
				float* src = diff.ptr<float>(angleIdx);
				for (int col = 0; col < diff.cols; col++)
				{
					if (fabs(src[col]) > fabs(pMaxDiff[col]))
					{
						pMaxDiff[col] = src[col];
						pMaxDiffAngle[col] = angleIdx * fProjectionAngleStep + fProjectionAngleFrom;
						pMaxDiffAngleIdx[col] = angleIdx;
					}
				}
			}
		}

		if (rc == OK)
		{
			m_profile = vDiffOfIndex.clone();
		}

		//if (rc == OK)
		//{
		//	matContrastProfile = maxDiff;
		//}





//#ifdef _DEBUG
//		if (rc == OK)
//		{
//			float* p = maxDiff.ptr<float>();
//			std::ofstream ofs("./test.txt", std::ios::trunc);
//			for (size_t i = 0; i < diff.cols; i++)
//			{
//				ofs << p[i] << std::endl;
//			}
//			ofs.close();
//		}
//#endif


		// find local maximum/minimum
		std::vector<int> lmm;
		if (rc == OK)
		{
			float* pMaxDiff = vDiffOfIndex.ptr<float>();
			float* pMaxDiffAngle = vAngleOfIndex.ptr<float>();
			for (int idx = 1; idx < vDiffOfIndex.cols - 1; idx++)
			{
				if (fabs(pMaxDiff[idx]) >= fabs(pMaxDiff[idx - 1]) &&
					fabs(pMaxDiff[idx]) > fabs(pMaxDiff[idx + 1]))
				{
					lmm.push_back(idx);
				}
			}
		}

		if (rc == OK && !bIsPairMode)
		{
			float* pMaxDiff = vDiffOfIndex.ptr<float>();
			float* pMaxDiffAngle = vAngleOfIndex.ptr<float>();
			int * pMaxDiffAngleIdx = vAngleIdxOfIndex.ptr<int>();
			for (int lmmidx = 0; lmmidx < lmm.size(); lmmidx++) // loop over all local maximum/minimum
			{
				int index = lmm[lmmidx];
				float cdiff = pMaxDiff[index];
				float angle = pMaxDiffAngle[index];
				int angleIndex = pMaxDiffAngleIdx[index];
				float* pDiffAtAngle = diff.ptr<float>(angleIndex);

				if (cdiff >= fContrastThresh && eMajorPolarity == DARK_TO_LIGHT ||
					cdiff <= -fContrastThresh && eMajorPolarity == LIGHT_TO_DARK ||
					fabs(cdiff) >= fContrastThresh && eMajorPolarity == ANY)
				{


					CaliperResult r;
					//r.bValid = true;
					r.edge0 = lmmidx;
					r.fContrast0 = fabs(pMaxDiff[lmm[lmmidx]]);
					r.fAngle0 = lastSearchDir + 90 + angle;

					float fMaxTmp;
					float fNothong;
					parabolaInterp(pDiffAtAngle[index - 1], pDiffAtAngle[index], pDiffAtAngle[index + 1], &fMaxTmp, &fNothong);

					r.fPos = index + fMaxTmp;
					r.fPos0 = index + fMaxTmp;
					r.x = lastCenter.x + (r.fPos - nSearchLength / 2.0) * cos(lastSearchDir*CV_PI / 180);
					r.y = lastCenter.y + (r.fPos - nSearchLength / 2.0) * sin(lastSearchDir*CV_PI / 180);
					r.x0 = r.x;
					r.y0 = r.y;
					//r.matProfile = maxDiff;


					//int angleIdx = pMaxDiffAngleIdx[lmm[lmmidx]];
					//for (int i=0; i<4; i++)
					//{
					//	r.roiPoly[i].x = roiForEachAngle[angleIdx * 4 + i].x;
					//	r.roiPoly[i].y = roiForEachAngle[angleIdx * 4 + i].y;
					//}
					
					//r.ptNominalPos = lastCenter;
					//r.fSearchDir = lastSearchDir;

					//if (nFoundResult < *nCapacity)
					//{
					//	result[nFoundResult++] = r;
					//}

					m_result.push_back(r);
				}
			}
		}

		if (rc == OK && bIsPairMode)
		{
			float* pMaxDiff = vDiffOfIndex.ptr<float>();
			float* pMaxDiffAngle = vAngleOfIndex.ptr<float>();
			//int * pMaxDiffAngleIdx = maxDiffAngleIdx.ptr<int>();

			for (int lmmidx1 = 0; lmmidx1 < lmm.size(); lmmidx1++)
			{
				if (pMaxDiff[lmm[lmmidx1]] >= fContrastThresh && eMajorPolarity == DARK_TO_LIGHT ||
					pMaxDiff[lmm[lmmidx1]] <= -fContrastThresh && eMajorPolarity == LIGHT_TO_DARK ||
					fabs(pMaxDiff[lmm[lmmidx1]]) >= fContrastThresh && eMajorPolarity == ANY)
				{
					for (int lmmidx2 = lmmidx1 + 1; lmmidx2 < lmm.size(); lmmidx2++)
					{
						float dis = fabs(lmm[lmmidx2] - lmm[lmmidx1]);

						if (dis > fPairWidthMin && dis < fPairWidthMax)
						{
							if (pMaxDiff[lmm[lmmidx2]] >= fContrastThresh && eSecondaryPolarity == DARK_TO_LIGHT ||
								pMaxDiff[lmm[lmmidx2]] <= -fContrastThresh && eSecondaryPolarity == LIGHT_TO_DARK ||
								fabs(pMaxDiff[lmm[lmmidx2]]) >= fContrastThresh && eSecondaryPolarity == ANY)
							{
								CaliperResult r;
								//r.bValid = true;
								r.edge0 = lmmidx1;
								r.edge1 = lmmidx2;
								r.fContrast0 = fabs(pMaxDiff[lmm[lmmidx1]]);
								r.fContrast1 = fabs(pMaxDiff[lmm[lmmidx2]]);
								r.fAngle0 = lastSearchDir + 90 + pMaxDiffAngle[lmm[lmmidx1]];
								r.fAngle1 = lastSearchDir + 90 + pMaxDiffAngle[lmm[lmmidx2]];

								r.fPos0 = lmm[lmmidx1];
								r.fPos1 = lmm[lmmidx2];

								r.fPos = (r.fPos0 + r.fPos1) / 2.0;

								r.x0 = lastCenter.x + (r.fPos0 - nSearchLength / 2.0) * cos(lastSearchDir*CV_PI / 180);
								r.y0 = lastCenter.y + (r.fPos0 - nSearchLength / 2.0) * sin(lastSearchDir*CV_PI / 180);
								r.x1 = lastCenter.x + (r.fPos1 - nSearchLength / 2.0) * cos(lastSearchDir*CV_PI / 180);
								r.y1 = lastCenter.y + (r.fPos1 - nSearchLength / 2.0) * sin(lastSearchDir*CV_PI / 180);

								r.x = (r.x0 + r.x1) / 2.0;
								r.y = (r.y0 + r.y1) / 2.0;

								r.fPairWidth = fabs(r.fPos0 - r.fPos1);

								//r.matProfile = maxDiff;

								//int angleIdx = pMaxDiffAngleIdx[(lmm[lmmidx1] + lmm[lmmidx2])/2];
								//for (int i = 0; i < 4; i++)
								//{
								//	r.roiPoly[i].x = roiForEachAngle[angleIdx * 4 + i].x;
								//	r.roiPoly[i].y = roiForEachAngle[angleIdx * 4 + i].y;
								//}

								//r.ptNominalPos = lastCenter;
								//r.fSearchDir = lastSearchDir;

								//if (nFoundResult < *nCapacity)
								//{
								//	result[nFoundResult++] = r;
								//}

								m_result.push_back(r);
							}
						}
					}
				}
			}
		}



		//*nCapacity = nFoundResult;

		return rc;
	}

	int Caliper::run(IN cv::Mat img, IN PoseXYR _ref,
		OUT CaliperResult *result, INOUT int *nCapacity)
	{
		int rc = OK;
		rc = run(img, _ref);
		*nCapacity = (std::min)(*nCapacity, (int)m_result.size());
		for (int i=0; i<*nCapacity; i++)
		{
			result[i] = m_result[i];
		}
		return rc;
	}

	cv::Mat Caliper::getProfile()
	{
		return m_profile;
	}

	cv::Mat Caliper::getImage()
	{
		return m_image;
	}

	bool Caliper::sucess()
	{
		return m_result.size() > 0;
	}

	int Caliper::getResults(std::vector<CaliperResult> &result)
	{
		result = m_result;
		return result.size();
	}

	int Caliper::getResult(CaliperResult &result)
	{
		int idx = -1;
		float fMaxContrast = -1;
		for (size_t i = 0; i < m_result.size(); i++)
		{
			if (m_result[i].fContrast0 > fMaxContrast)
			{
				fMaxContrast = m_result[i].fContrast0;
				idx = i;
			}
		}

		if (idx >= 0)
		{
			result = m_result[idx];
			return OK;
		}

		return __LINE__;
	}

	void Caliper::getROI(cv::Point2f* roi)
	{
		float halfSearchLen = nSearchLength / 2.0;
		float halfPrjLen = nProjectionLength / 2.0;
		float vx = cos(lastSearchDir * CV_PI / 180.0);
		float vy = sin(lastSearchDir * CV_PI / 180.0);

		float nx = -vy;
		float ny = vx;

		roi[0].x = lastCenter.x + vx * -halfSearchLen + nx * halfPrjLen;
		roi[0].y = lastCenter.y + vy * -halfSearchLen + ny * halfPrjLen;

		roi[1].x = lastCenter.x + vx * halfSearchLen + nx * halfPrjLen;
		roi[1].y = lastCenter.y + vy * halfSearchLen + ny * halfPrjLen;

		roi[2].x = lastCenter.x + vx * halfSearchLen + nx * -halfPrjLen;
		roi[2].y = lastCenter.y + vy * halfSearchLen + ny * -halfPrjLen;


		roi[3].x = lastCenter.x + vx * -halfSearchLen + nx * -halfPrjLen;
		roi[3].y = lastCenter.y + vy * -halfSearchLen + ny * -halfPrjLen;
	}

	//cv::Point2f Caliper::getLastCenter()
	//{
	//	return lastCenter;
	//}

	

	//float VisionLibrary::Caliper::getLastSearchDir()
	//{
	//	return lastSearchDir;
	//}

	cv::Mat Caliper::unwarpImage(cv::Mat src, PoseXYR _ref, float fProjectionAngle/*, cv::Point2f roi[4]*/)
	{
		assert(nProjectionLength > 0 && nSearchLength > 0);

		cv::Point2f p = m_ref.pointGlobalToLocal(center);
		cv::Point2f curCenter = _ref.pointLocalToGlobal(p);

		cv::Point2f v = m_ref.vectorGlobalToLocal(cv::Point2f(cos(fSearchDir*CV_PI / 180.0), sin(fSearchDir*CV_PI / 180.0)));
		cv::Point2f cv = _ref.vectorLocalToGlobal(v);


		double fCurSearchDir = atan2(cv.y, cv.x) * 180 / CV_PI;


		cv::Mat dst(nProjectionLength, nSearchLength, src.type());


		cv::Mat map(nProjectionLength, nSearchLength, CV_32FC2);

		double m11 = cos(fCurSearchDir*CV_PI / 180);
		double m21 = sin(fCurSearchDir*CV_PI / 180);

		double m12 = cos((fCurSearchDir + 90 + fProjectionAngle) * CV_PI / 180);
		double m22 = sin((fCurSearchDir + 90 + fProjectionAngle) * CV_PI / 180);


		for (int i = 0; i < nProjectionLength; i++)
		{
			float *p = map.ptr<float>(i);

			float y = i - nProjectionLength / 2.0;
			for (int j = 0; j < nSearchLength; j++)
			{



				float x = j - nSearchLength / 2.0;

				float nx = m11 * x + m12 * y + curCenter.x;
				float ny = m21 * x + m22 * y + curCenter.y;

				p[2 * j] = nx;
				p[2 * j + 1] = ny;

				/*if (i == 0)
				{
					if (j == 0)
					{
						roi[0].x = nx;
						roi[0].y = ny;
					}
					if (j== nSearchLength-1)
					{
						roi[1].x = nx;
						roi[1].y = ny;
					}
				}

				if (i == nProjectionLength-1)
				{
					if (j == 0)
					{
						roi[3].x = nx;
						roi[3].y = ny;
					}
					if (j == nSearchLength - 1)
					{
						roi[2].x = nx;
						roi[2].y = ny;
					}
				}*/
			}
		}

		cv::remap(src, dst, map, cv::Mat(), cv::INTER_LINEAR);

		return dst;
	}

	void Caliper::parabolaInterp(float l, float c, float r, float* maxx, float* maxy)
	{
		assert(c >= l && c >= r || c <= l && c <= r);
		*maxx = (r - l) / (4 * c - 2 * (l + r));
		*maxy = c + (r - l)*(r - l) / (16 * c - 8 * (l + r));
	}

}

