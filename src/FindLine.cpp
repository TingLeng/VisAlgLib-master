#include "FindLine.h"
#include "CVRANSAC.h"
#include "CVRANSAC_LineParamEstimator.h"

namespace VisionLibrary
{

template <class T>
void fitLineLeastSquare(const std::vector<T> &data, std::vector<double> &parameters)
{
	double meanX, meanY, nx, ny, norm;
	double covMat11, covMat12, covMat21, covMat22; // The entries of the symmetric covariance matrix
	int i, dataSize = (int)data.size();

	parameters.clear();
	if (data.size()<2)
		return;

	meanX = meanY = 0.0;
	covMat11 = covMat12 = covMat21 = covMat22 = 0;
	for (i = 0; i<dataSize; i++) 
	{
		meanX += data[i].x;
		meanY += data[i].y;

		covMat11 += data[i].x * data[i].x;
		covMat12 += data[i].x * data[i].y;
		covMat22 += data[i].y * data[i].y;
	}

	meanX /= dataSize;
	meanY /= dataSize;

	covMat11 -= dataSize*meanX*meanX;
	covMat12 -= dataSize*meanX*meanY;
	covMat22 -= dataSize*meanY*meanY;
	covMat21 = covMat12;

	if (covMat11<1e-12)
	{
		nx = 1.0;
		ny = 0.0;
	}
	else 
	{
		//lamda1 is the largest eigen-value of the covariance matrix 
		//and is used to compute the eigne-vector corresponding to the smallest
		//eigenvalue, which isn't computed explicitly.
		double lamda1 = (covMat11 + covMat22 + sqrt((covMat11 - covMat22)*(covMat11 - covMat22) + 4 * covMat12*covMat12)) / 2.0;
		nx = -covMat12;
		ny = lamda1 - covMat22;
		norm = sqrt(nx*nx + ny*ny);
		nx /= norm;
		ny /= norm;
	}
	parameters.push_back(nx);
	parameters.push_back(ny);
	parameters.push_back(meanX);
	parameters.push_back(meanY);
}

FindLine::FindLine()
	:FindLine(PoseXYR(), cv::Point(0, 0), cv::Point(0, 0))
{

}


FindLine::FindLine(cv::Point2f _ptStart, cv::Point2f _ptEnd)
	:FindLine(PoseXYR(), _ptStart, _ptEnd)
{
	
}

FindLine::FindLine(PoseXYR _ref, cv::Point2f _ptStart, cv::Point2f _ptEnd)
	:ptStart(_ptStart), ptEnd(_ptEnd), nNumOfCaliper(20), nSearchLenght(40),
	nProjectionLength(20), fSearchDir(0), bIsPairMode(false), eMajorPolarity(ANY),
	eSecondaryPolarity(ANY), fPairWidthMax(20), fPairWidthMin(10), fContrastThresh(10),
	fFilterHalfWindow(1.0), ref(_ref), nNumToIgnoreAllowed(0), fAcceptFitError(1.5),
	m_eChooseStrategy(STRONGEST)
{
	float dx = _ptEnd.x - _ptStart.x;
	float dy = _ptEnd.y - _ptStart.y;
	float dis = sqrt(dx*dx + dy*dy);
	nNumOfCaliper = dis / nProjectionLength;
	float fLineAngle = atan2(dy, dx) * 180 / CV_PI;
	fSearchDir = fLineAngle + 90;
}

FindLine::FindLine(LineSeg ls)
	:FindLine(PoseXYR(), ls.start, ls.end)
{
	
}


VisionLibrary::FindLine& FindLine::operator=(const FindLine& other)
{
	if (this != &other)
	{
		this->ref = other.ref;
		this->ptStart = other.ptStart;
		this->ptEnd = other.ptEnd;
	}

	return *this;
}

FindLine::~FindLine()
{
}


int FindLine::run(IN cv::Mat img, IN PoseXYR _ref, OUT LineSeg &seg, OUT Line& l/*,
	OUT FindLineCaliperResult* pCaliperResult, INOUT int nCapaicy*/)
{
	int rc = OK;
	int idx = 0;
	
	vCalipers.clear();
	m_caliperResults.clear();
	m_caliperResults.reserve(nNumOfCaliper);
	m_bSuc = false;

	// generate calipers
	if (rc == OK)
	{
		float vx = ptEnd.x - ptStart.x;
		float vy = ptEnd.y - ptStart.y;
		float totalLength = sqrt(vx*vx + vy*vy);
		vx = vx / totalLength;
		vy = vy / totalLength;


		for (idx = 0; idx < nNumOfCaliper; idx++)
		{
			float x = ptStart.x + (totalLength / nNumOfCaliper / 2.0 + idx*totalLength / nNumOfCaliper)*vx;
			float y = ptStart.y + (totalLength / nNumOfCaliper / 2.0 + idx*totalLength / nNumOfCaliper)*vy;
			Caliper c(ref, cv::Point2f(x, y), fSearchDir);
			c.bIsPairMode = bIsPairMode;
			c.eMajorPolarity = eMajorPolarity;
			c.eSecondaryPolarity = eSecondaryPolarity;
			c.fContrastThresh = fContrastThresh;
			c.fFilterHalfWindow = fFilterHalfWindow;
			c.fPairWidthMax = fPairWidthMax;
			c.fPairWidthMin = fPairWidthMin;
			c.nSearchLength = nSearchLenght;
			c.nProjectionLength = nProjectionLength;
			c.fProjectionAngleFrom = 0;
			c.fProjectionAngleTo = 0;

			vCalipers.push_back(c);
		}
	}
	

	std::vector<FindLineCaliperResult> vValidResults;
	std::vector<FindLineCaliperResult> vInvalidResults;
	if (rc == OK)
	{
		for (idx = 0; idx < nNumOfCaliper; idx++)
		{
			FindLineCaliperResult fr;
			
			int nNumResult = 10;
			Caliper::CaliperResult r[10];

			if (OK != vCalipers[idx].run(img, _ref, r, &nNumResult) || nNumResult < 1)
			{
				
				vCalipers[idx].getROI(fr.roi);
				fr.idx = idx;
				fr.bValid = false;
				fr.bUsed = false;

				vInvalidResults.push_back(fr);

				continue;
			}


			vCalipers[idx].getROI(fr.roi);
			fr.bValid = true;
			fr.bUsed = false;

			float fMaxContrast = -1;
			switch (m_eChooseStrategy)
			{
			case VisionLibrary::FindLine::STRONGEST:

				for (int k = 0; k < nNumResult; k++)
				{
					float con = r[k].fContrast0 + r[k].fContrast1;
					if (con > fMaxContrast)
					{
						fMaxContrast = con;
						fr.detailedCaliperResult = r[k];
					}
				}

				break;
			case VisionLibrary::FindLine::FIRST:
				fr.detailedCaliperResult = r[0];
				break;
			case VisionLibrary::FindLine::LAST:
				fr.detailedCaliperResult = r[nNumResult - 1];
				break;
			default:
				break;
			}

			fr.x = fr.detailedCaliperResult.x;
			fr.y = fr.detailedCaliperResult.y;
			vValidResults.push_back(fr);
		}
	}
	
	std::vector<double> line;
	std::vector<FindLineCaliperResult> inliers, outliers;
	if (rc == OK)
	{
		CVRANSAC_LineParamEstimator<FindLineCaliperResult, double> le(fAcceptFitError);

		std::vector<bool> vLimitRange(4, false);
		std::vector<double> vFrom(4);
		std::vector<double> vTo(4);
		

		m_bSuc = true;
		if (true != CVRANSAC<FindLineCaliperResult, double>::compute(
			vValidResults, 
			&le,
			nNumOfCaliper - nNumToIgnoreAllowed, 
			0.99, 
			line, 
			vLimitRange,
			vFrom, 
			vTo, 
			inliers, 
			outliers))
		{
			m_bSuc = false;
		}
	}

	if (rc == OK)
	{
		for (size_t idx = 0; idx < inliers.size(); idx++)
		{
			inliers[idx].bUsed = true;
		}

		m_caliperResults.insert(m_caliperResults.end(), vInvalidResults.begin(), vInvalidResults.end());
		m_caliperResults.insert(m_caliperResults.end(), inliers.begin(), inliers.end());
		m_caliperResults.insert(m_caliperResults.end(), outliers.begin(), outliers.end());
	}

	if (rc == OK && m_bSuc)
	{
		l.nx = line[0];
		l.ny = line[1];
		l.x  = line[2];
		l.y  = line[3];

		seg.start.x = l.ny * l.ny * vCalipers[0].center.x +
			l.nx*l.nx * l.x - 
			l.nx*l.ny*vCalipers[0].center.y +
			l.nx*l.ny*l.y;

		seg.start.y = l.nx*l.nx*vCalipers[0].center.y +
			l.ny*l.ny*l.y -
			l.nx*l.ny* vCalipers[0].center.x +
			l.nx*l.ny*l.x;

		seg.end.x = l.ny * l.ny * vCalipers[nNumOfCaliper-1].center.x +
			l.nx*l.nx * l.x -
			l.nx*l.ny*vCalipers[nNumOfCaliper-1].center.y +
			l.nx*l.ny*l.y;

		seg.end.y = l.nx*l.nx*vCalipers[nNumOfCaliper-1].center.y +
			l.ny*l.ny*l.y -
			l.nx*l.ny* vCalipers[nNumOfCaliper-1].center.x +
			l.nx*l.ny*l.x;
	}



	return rc;


}

void FindLine::getCaliperResult(std::vector<FindLineCaliperResult> & calipers)
{
	calipers = m_caliperResults;
}

cv::Mat FindLine::getProfile(int idx)
{
	if (idx < 0 || idx > vCalipers.size())
	{
		return cv::Mat();
	}

	return vCalipers[idx].getProfile();
}

cv::Mat FindLine::getCaliperImage(int idx)
{
	if (idx < 0 || idx > vCalipers.size())
	{
		return cv::Mat();
	}

	return vCalipers[idx].getImage();

}

bool FindLine::Success()
{
	return m_bSuc;
}

}
