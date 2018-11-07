
#include "Inspector.h"





Inspector::Inspector(cv::Mat imgInitial, const vector<vector<cv::Point>> &vROIs,
	const vector<vector<cv::Point>> &vIgnoreROIs,
	InterpolationMethod im, float fOffset, float fScale) :
	m_imgInitial(imgInitial),
	m_vROIs(vROIs),
	m_vIgnoreROIs(vIgnoreROIs),
	m_eInterpolationMethod(im),
	m_fOffset(fOffset),
	m_fScale(fScale),
	m_nTrainImageCount(0)
{
	m_vRoiBoudingBoxes.resize(vROIs.size());
	m_vBoudingBoxMask.resize(vROIs.size());

	cv::Mat imgTotalMask = cv::Mat::zeros(imgInitial.size(), imgInitial.type());
	if (true)
	{
		for (size_t roiid = 0; roiid < m_vROIs.size(); roiid++)
		{
			cv::fillPoly(imgTotalMask, 
				std::vector< std::vector<cv::Point> >(1, m_vROIs[roiid]), 
				cv::Scalar(255, 255, 255, 255));
		}

		for (size_t roiid = 0; roiid < m_vIgnoreROIs.size(); roiid++)
		{
			cv::fillPoly(imgTotalMask, 
				std::vector< std::vector<cv::Point> >(1, m_vIgnoreROIs[roiid]), 
				cv::Scalar(0, 0, 0, 0));
		}
	}


	for (size_t roiid = 0; roiid < vROIs.size(); roiid++)
	{
		cv::Rect boudingBox = cv::boundingRect(vROIs[roiid]);
		m_vRoiBoudingBoxes[roiid] = boudingBox;
		//m_vBoudingBoxMask[roiid].create(boudingBox.height, boudingBox.width, imgInitial.type());
		m_vBoudingBoxMask[roiid] = imgTotalMask(boudingBox).clone();

		//std::vector< std::vector<cv::Point> > polyInBoundingBox(1, vROIs[roiid]);
		//for (size_t tt = 0; tt < polyInBoundingBox[0].size(); tt++)
		//{
		//	polyInBoundingBox[0][tt].x -= boudingBox.x;
		//	polyInBoundingBox[0][tt].y -= boudingBox.y;
		//}
		//cv::fillPoly(m_vBoudingBoxMask[roiid], polyInBoundingBox, cv::Scalar(255, 255, 255, 255));

		cv::Mat debug = m_vBoudingBoxMask[roiid];

		// prepare map for remap
		cv::Mat map(boudingBox.height, boudingBox.width, CV_32FC2);
		for (int row = 0; row < map.rows; row++)
		{
			for (int col = 0; col < map.cols; col++)
			{
				map.at<cv::Vec2f>(row, col) = cv::Vec2f(float(col) + boudingBox.x, float(row) + boudingBox.y);
			}
		}
		m_vMaps.push_back(map);
		
		// prepare 
		m_vStatisticalImages.push_back(vector<cv::Mat>());

		m_vRefImage.push_back(cv::Mat::zeros(boudingBox.height, boudingBox.width, CV_32FC(imgInitial.channels())));
		m_vThresholdImage.push_back(cv::Mat::zeros(boudingBox.height, boudingBox.width, CV_32FC(imgInitial.channels())));
	}
}

Inspector::Inspector()
{

}

void Inspector::DoTrain()
{
	for (size_t roiid = 0; roiid < m_vROIs.size(); roiid++)
	{
		for (int r = 0; r < m_vRoiBoudingBoxes[roiid].height; r++)
		{
			for (int c = 0; c < m_vRoiBoudingBoxes[roiid].width; c++)
			{
				cv::Mat arr(1, m_vStatisticalImages[roiid].size(), m_vStatisticalImages[roiid][0].type());
				for (size_t sid = 0; sid < m_vStatisticalImages[roiid].size(); sid++)
				{
					cv::Mat m = m_vStatisticalImages[roiid][sid].row(r).col(c);
					m.copyTo(arr.col(sid));
				}
				cv::Mat mean, sigma;
				cv::meanStdDev(arr, mean, sigma);

				mean.reshape(m_imgInitial.channels()).copyTo(m_vRefImage[roiid].row(r).col(c));
				sigma.reshape(m_imgInitial.channels()).copyTo(m_vThresholdImage[roiid].row(r).col(c));
			}
		}

		cv::Mat imgMean = m_vRefImage[roiid];
		cv::Mat imgSigma = m_vThresholdImage[roiid];

		//cv::namedWindow("mean", 1);
		//cv::imshow("mean", m_vRefImage[roiid] / 255);

		//cv::namedWindow("sigma", 1);
		//cv::imshow("sigma", m_vThresholdImage[roiid] / 255);
		//cv::waitKey(0);
	}
}

 void Inspector::UnTrain()
 {
	 m_nTrainImageCount = 0;
	 for (size_t roiid = 0; roiid < m_vROIs.size(); roiid++)
	 {
		 for (size_t imgid = 0; imgid < m_vStatisticalImages[roiid].size(); imgid++)
		 {
			 m_vStatisticalImages[roiid][imgid].release();
		 }
		 m_vStatisticalImages[roiid].clear();
	 }
 }

 void Inspector::Compare( cv::Mat img,cv::Mat &HomMat2D, vector<cv::Mat>& vMultipleOfSigma, vector<cv::Mat>& vDefectImage, vector<cv::Mat>& vIdealImage)
 {
	 assert(img.type() == m_imgInitial.type());

	 vMultipleOfSigma.clear();
	 vDefectImage.clear();
	 vIdealImage.clear();

	 char buf[256];

	 for (size_t roiid = 0; roiid < m_vROIs.size(); roiid++)
	 {


		 int64 st = cv::getTickCount();

		 cv::Mat imgROI;
		 if (true)
		 {
			 imgROI.create(m_vRoiBoudingBoxes[roiid].height, m_vRoiBoudingBoxes[roiid].width, img.type());
			 cv::Mat map(m_vRoiBoudingBoxes[roiid].height, m_vRoiBoudingBoxes[roiid].width, CV_32FC2);
			 cv::transform(m_vMaps[roiid], map, HomMat2D);
			 cv::remap(img, imgROI, map, cv::Mat(), cv::INTER_LINEAR, 0);

			 //sprintf_s(buf, "Inspection Ori %d", roiid);
			 //cv::namedWindow(buf, 1);
			 //cv::imshow(buf, imgROI);

			 //cv::imwrite("C:\\Test\\1\\1.bmp", imgROI);
		 }

		 

		 cv::Mat imgMean = m_vRefImage[roiid];
		 cv::Mat imgCenterDis;
		 if (true)
		 {
			 cv::subtract(imgROI, imgMean, imgCenterDis, cv::Mat(), imgMean.type());
			 imgCenterDis = cv::abs(imgCenterDis);


			 cv::Mat imgMean8U;
			 imgMean.convertTo(imgMean8U, CV_8U);
			 //sprintf_s(buf, "mean %d", roiid);
			 //cv::namedWindow(buf, 1);
			 //cv::imshow(buf, imgMean8U);

			 //sprintf_s(buf, "sigma %d", roiid);
			 //cv::namedWindow(buf, 1);
			 //cv::imshow(buf, m_vThresholdImage[roiid]);

			 //sprintf_s(buf, "imgCenterDis %d", roiid);
			 //cv::namedWindow(buf, 1);
			 //cv::imshow(buf, imgCenterDis);
		 }



		 
		 cv::Mat thresh;
		 cv::Mat diffToThresh;
		 cv::Mat defectImage;
		 if (true)
		 {
			 thresh = m_vThresholdImage[roiid] * m_fScale + m_fOffset;
			 cv::subtract(imgCenterDis, thresh, diffToThresh, cv::Mat(), thresh.type());
			 cv::threshold(diffToThresh, defectImage, 0, 255, CV_THRESH_BINARY);
			 
			 defectImage.convertTo(defectImage, CV_8U);

			 cv::bitwise_and(m_vBoudingBoxMask[roiid], defectImage, defectImage);

			 //sprintf_s(buf, "defectImage %d", roiid);
			 //cv::namedWindow(buf, 1);
			 //cv::imshow(buf, defectImage);
		 }


		 vDefectImage.push_back(defectImage);
		 vIdealImage.push_back(imgMean);


		 //cv::waitKey(30);
		 
		 int64 et = cv::getTickCount();
		 printf("Inspect ROI %d, time=%f ms\n", roiid, ((double)(et - st)) / cv::getTickFrequency() * 1000);
 	 }
 }

 void Inspector::FeedTrain( cv::Mat img, cv::Mat &HomMat2D )
 {
	 assert(img.type() == m_imgInitial.type());

	 for (size_t roiid = 0; roiid < m_vROIs.size(); roiid++)
	 {
		 cv::Mat imgROI(m_vRoiBoudingBoxes[roiid].height, m_vRoiBoudingBoxes[roiid].width, img.type());
		 cv::Mat map(m_vRoiBoudingBoxes[roiid].height, m_vRoiBoudingBoxes[roiid].width, CV_32FC2);

		 cv::transform(m_vMaps[roiid], map, HomMat2D);

		 cv::remap(img, imgROI, map, cv::Mat(), cv::INTER_LINEAR, 0);

		 m_vStatisticalImages[roiid].push_back(imgROI);

		 //cv::namedWindow("feadtrain");
		 //cv::imshow("feadtrain", imgROI);
		 //cv::waitKey(0);
	 }

	 m_nTrainImageCount++;
 }

 int Inspector::GetImageCount()
 {
    return m_nTrainImageCount;
 }

 Inspector::~Inspector()
 {

 }



 AnchorPosInspector::AnchorPosInspector( const vector<cv::Point2f> &vAnchorPos,TransformationType tt,
	    cv::Mat imgInitial, const vector<vector<cv::Point>>& vROIs, 
	 const vector<vector<cv::Point>>& vIgnoreROIs,
	 InterpolationMethod im, float fOffset, float fScale ):
	 m_vInitialAnchorPos(vAnchorPos),
	 m_eTransformType(tt),
	 Inspector(imgInitial,vROIs, vIgnoreROIs, im,fOffset,fScale)
 {

 }

 //AnchorPosInspector::AnchorPosInspector(const AnchorPosInspector& api)
	// :Inspector(api.m_imgInitial, api.m_vROIs, api.m_eInterpolationMethod, api.m_fOffset, api.m_fScale),
	// m_vInitialAnchorPos(api.m_vInitialAnchorPos),
	// m_eTransformType(api.m_eTransformType),
	// m_vRefImage(api.m_vRefImage),
	// m_vThresholdImage(api)
 //{
	// 

 //}

 AnchorPosInspector::AnchorPosInspector()
 {

 }

 void AnchorPosInspector::FeedTrain(cv::Mat img, const vector<cv::Point2f> &vAnchorPos)
 {
	 cv::Mat homMat2D;
	 getTransformation(m_vInitialAnchorPos, vAnchorPos, homMat2D);

	 
	 Inspector::FeedTrain(img, homMat2D);
 }

 void AnchorPosInspector::Compare(cv::Mat img, const vector<cv::Point2f> &vAnchorPos, 
	 vector<cv::Mat>& vAbsDiffImage, vector<cv::Mat>& vDefectImage, vector<cv::Mat>& vIdealImage)
 {
	 cv::Mat homMat2D;
	 getTransformation(m_vInitialAnchorPos, vAnchorPos, homMat2D);

	 Inspector::Compare(img, homMat2D, vAbsDiffImage, vDefectImage, vIdealImage);
 }

 void AnchorPosInspector::getTransformation(const vector<cv::Point2f> & vPoint1, 
	 const vector<cv::Point2f> & vPoint2, cv::Mat &HomMat2D)
 {
	 cv::Mat ret;
	 switch (m_eTransformType)
	 {
	 case AFFINE:
		 //ret = getAffineTransform(vPoint1, vPoint2);
		 break;
	 case SIMILAR:
		 getSimilarTransform(vPoint1, vPoint2, HomMat2D);
		 break;
	 case RIGID:
		 getRigidTransform(vPoint1, vPoint2, HomMat2D);
		 break;
	 default:
		 break;
	 } 
 }

 void AnchorPosInspector::getAffineTransform( vector<cv::Point2f> & x, vector<cv::Point2f> & y, cv::Mat &HomMat2D )
 {

 }

 void AnchorPosInspector::getRigidTransform( const vector<cv::Point2f> & x, const vector<cv::Point2f> & y, cv::Mat &HomMat2D )
 {
	 size_t i;
	 size_t nPointCount = x.size();
	 float sigmax2 = 0;

	 //cv::Mat A, B, X;
	 assert(x.size() == y.size());

	 cv::Point2f center1(0.0, 0.0);
	 cv::Point2f center2(0.0, 0.0);

	 for (i = 0; i < nPointCount; i++)
	 {
		 center1.x += x[i].x;
		 center1.y += x[i].y;

		 center2.x += y[i].x;
		 center2.y += y[i].y;
	 }

	 center1.x /= x.size();
	 center1.y /= x.size();
	 center2.x /= x.size();
	 center2.y /= x.size();

	 vector<cv::Point2f> xc = x;
	 vector<cv::Point2f> yc = y;
	 for (i = 0; i < nPointCount; i++)
	 {
		 xc[i].x -= center1.x;
		 xc[i].y -= center1.y;

		 yc[i].x -= center2.x;
		 yc[i].y -= center2.y;

		 sigmax2 += xc[i].x *xc[i].x + xc[i].y * xc[i].y;
	 }

	 sigmax2 /= nPointCount;

	 cv::Mat Y, XT, matCovXY;
	 Y.create(2, nPointCount, CV_32FC1);
	 XT.create(nPointCount, 2, CV_32FC1);

	 float* ptr0 = Y.ptr<float>(0);
	 float* ptr1 = Y.ptr<float>(1);
	 for (i = 0; i < nPointCount; i++)
	 {
		 ptr0[i] = yc[i].x;
		 ptr1[i] = yc[i].y;

		 float *ptr = XT.ptr<float>(i);
		 ptr[0] = xc[i].x;
		 ptr[1] = xc[i].y;
	 }

	 matCovXY = (Y * XT) / nPointCount;

	 cv::Mat d, u, vt;
	 cv::SVDecomp(matCovXY, d, u, vt);

	 // prepare D
	 cv::Mat D = cv::Mat::eye(2, 2, CV_32FC1);
	 cv::Mat tmp = D.diag(0);
	 tmp.at<float>(0) = d.at<float>(0);
	 tmp.at<float>(1) = d.at<float>(1);

	 // prepare S
	 cv::Mat S = cv::Mat::eye(2, 2, CV_32FC1);
	 double det = cv::determinant(u) * cv::determinant(vt);
	 if (det < 0)
	 {
		 S.at<float>(1, 1) = -1.0;
	 }

	 // R
	 cv::Mat R = u*S*vt;

	 // c (scale)
	 cv::Scalar scale = cv::trace(D * S) / sigmax2;
	 double c = scale.val[0];

	 // cR
	 cv::Mat cR = c * R;

	 // t
	 cv::Mat center1Mat(2, 1, CV_32FC1, &center1);
	 cv::Mat center2Mat(2, 1, CV_32FC1, &center2);
	 cv::Mat translation = center2Mat - R * center1Mat;

	 HomMat2D.create(2, 3, CV_32FC1);
	 HomMat2D.at<float>(0, 0) = R.at<float>(0, 0);
	 HomMat2D.at<float>(0, 1) = R.at<float>(0, 1);
	 HomMat2D.at<float>(1, 0) = R.at<float>(1, 0);
	 HomMat2D.at<float>(1, 1) = R.at<float>(1, 1);
	 HomMat2D.at<float>(0, 2) = translation.at<float>(0, 0);
	 HomMat2D.at<float>(1, 2) = translation.at<float>(1, 0);
 }

 void AnchorPosInspector::getSimilarTransform( const vector<cv::Point2f> & x, const vector<cv::Point2f> & y, cv::Mat &HomMat2D )
 {
	 size_t i;
	 size_t nPointCount = x.size();
	 float sigmax2 = 0;

	 //cv::Mat A, B, X;
	 assert(x.size() == y.size());

	 cv::Point2f center1(0.0, 0.0);
	 cv::Point2f center2(0.0, 0.0);

	 for(i = 0; i < nPointCount; i++)
	 {
		 center1.x += x[i].x;
		 center1.y += x[i].y;

		 center2.x += y[i].x;
		 center2.y += y[i].y;
	 }

	 center1.x /= x.size();
	 center1.y /= x.size();
	 center2.x /= x.size();
	 center2.y /= x.size();

	 vector<cv::Point2f> xc = x;
	 vector<cv::Point2f> yc = y;
	 for (i = 0; i < nPointCount; i++)
	 {
		 xc[i].x -= center1.x;
		 xc[i].y -= center1.y;

		 yc[i].x -= center2.x;
		 yc[i].y -= center2.y;

		 sigmax2 += xc[i].x *xc[i].x + xc[i].y * xc[i].y;
	 }

	 sigmax2 /= nPointCount;


	 cv::Mat Y, XT, matCovXY;
	 Y.create(2, nPointCount, CV_32FC1);
	 XT.create(nPointCount, 2, CV_32FC1);

	 float* ptr0 = Y.ptr<float>(0);
	 float* ptr1 = Y.ptr<float>(1);
	 for (i = 0; i < nPointCount; i++)
	 {
		 ptr0[i] = yc[i].x;
		 ptr1[i] = yc[i].y;

		 float *ptr = XT.ptr<float>(i);
		 ptr[0] = xc[i].x;
		 ptr[1] = xc[i].y;
	 }

	 matCovXY = (Y * XT) / nPointCount;

	 cv::Mat d, u, vt;
	 cv::SVDecomp(matCovXY, d, u, vt);

	 // prepare D
	 cv::Mat D = cv::Mat::eye(2, 2, CV_32FC1);
	 cv::Mat tmp = D.diag(0);
	 tmp.at<float>(0) = d.at<float>(0);
	 tmp.at<float>(1) = d.at<float>(1);

	 // prepare S
	 cv::Mat S = cv::Mat::eye(2, 2, CV_32FC1);
	 //float det = cv::determinant(matCovXY);
	 double det = cv::determinant(u) * cv::determinant(vt);
	 if (det < 0)
	 {
		 S.at<float>(1, 1) = -1.0;
	 }

	 // R
	 cv::Mat R = u*S*vt;

	 // c (scale)
	 cv::Scalar scale = cv::trace(D * S) / sigmax2;
	 double c = scale.val[0];

	 // cR
	 cv::Mat cR = c * R;

	 // t
	 cv::Mat center1Mat(2, 1, CV_32FC1, &center1);
	 cv::Mat center2Mat(2, 1, CV_32FC1, &center2);
	 cv::Mat translation = center2Mat - cR * center1Mat;

	 HomMat2D.create(2, 3, CV_32FC1);
	 HomMat2D.at<float>(0, 0) = cR.at<float>(0, 0);
	 HomMat2D.at<float>(0, 1) = cR.at<float>(0, 1);
	 HomMat2D.at<float>(1, 0) = cR.at<float>(1, 0);
	 HomMat2D.at<float>(1, 1) = cR.at<float>(1, 1);
	 HomMat2D.at<float>(0, 2) = translation.at<float>(0, 0);
	 HomMat2D.at<float>(1, 2) = translation.at<float>(1, 0);
 }

 AnchorPosInspector::~AnchorPosInspector()
 {

 }




 //PMInspect::PMInspect(cv::Mat imgInitial,vector<cv::Rect> &vROIs,cv::Rect &vPROI,
	// InterpolationMethod im, float fOffset, float fScale)
	// :Inspector(imgInitial,vROIs,im,fOffset,fScale),
	// m_vPROI(vPROI),
	// m_imgPattern(imgInitial(vPROI)),
	// m_model(NULL),
	// m_nInfoCode(0),
	// m_fCoarseGrand(6),
 //    m_fFineGrand(2),
 //    m_fScoreThresh(0.5)
 //{
	// int rc = OK;
	//
	// CV4A_PM_ALG_MODE_ENUM mode = CV4A_PM_QuickAndAccurate;//PM的不同匹配模式

	// //把CV::Mat的模板格式转换成CV4A_IMAGE格式的模板
	// cv::Mat GrayPatternImg;
	// cv::cvtColor(GetPatternImg(),GrayPatternImg,CV_RGB2GRAY);
	// CV4A_IMAGE  pattern ;
	// if ((GrayPatternImg.type() == CV_8U) && (! GrayPatternImg.empty()) && (GrayPatternImg.rows > 2)  && (GrayPatternImg.cols > 2))
	// {
	//	 pattern.imgHeight = GrayPatternImg.rows;
	//	 pattern.imgWidth = GrayPatternImg.cols;
	//	 pattern.imgType = CV4A_IMG_DATA_TYPE_8U;
	//	 pattern.isInitialized = 1;
	//	 pattern.pImg.pCharImgData = GrayPatternImg.data;
	//	 pattern.sizePixel = 1;
	//	 //pattern.channel = 1;
	//	 pattern.widthStep = GrayPatternImg.step;
	// }

	// SMEE_BOOL bTrainSuc;
	// rc = CV4A_pm_train(&pattern, NULL, mode, m_fCoarseGrand, m_fFineGrand, &m_model, &bTrainSuc,&m_nInfoCode);//进行模板训练
	// 	 
 //}

 //PMInspect::PMInspect( cv::Mat imgInitial,vector<cv::Rect> &vROIs,cv::Rect &vPROI, InterpolationMethod im, float fOffset, float fScale,IN float fCoarseGrand , IN float fFineGrand ,IN float fScoreThresh  )
	// :Inspector(imgInitial,vROIs,im,fOffset,fScale),
	//  m_vPROI(vPROI),
	//  m_imgPattern(imgInitial(vPROI)),
	//  m_model(NULL),
	//  m_nInfoCode(0),
	//  m_fCoarseGrand(fCoarseGrand),
	//  m_fFineGrand(fFineGrand),
	//  m_fScoreThresh(fScoreThresh)
 //{
	// int rc = OK;
	//
	// CV4A_PM_ALG_MODE_ENUM mode = CV4A_PM_QuickAndAccurate;//PM的不同匹配模式

	// //把CV::Mat的模板格式转换成CV4A_IMAGE格式的模板
	// cv::Mat GrayPatternImg;
	// cv::cvtColor(GetPatternImg(),GrayPatternImg,CV_RGB2GRAY);
	// CV4A_IMAGE  pattern ;
	// if ((GrayPatternImg.type() == CV_8U) && (! GrayPatternImg.empty()) && (GrayPatternImg.rows > 2)  && (GrayPatternImg.cols > 2))
	// {
	//	 pattern.imgHeight = GrayPatternImg.rows;
	//	 pattern.imgWidth = GrayPatternImg.cols;
	//	 pattern.imgType = CV4A_IMG_DATA_TYPE_8U;
	//	 pattern.isInitialized = 1;
	//	 pattern.pImg.pCharImgData = GrayPatternImg.data;
	//	 pattern.sizePixel = 1;
	//	 //pattern.channel = 1;
	//	 pattern.widthStep = GrayPatternImg.step;
	// }

	// 
	// SMEE_BOOL bTrainSuc;
	// rc = CV4A_pm_train(&pattern, NULL, mode, fCoarseGrand, fFineGrand, &m_model, &bTrainSuc,&m_nInfoCode);//进行模板训练
	// 
	// 
 //}

 //PMInspect::~PMInspect()
 //{					
	//if (m_model)
	//{
	//	CV4A_pm_release_model(&m_model);
	//}
 //}

 //void PMInspect::TrainPhase(IN cv::Mat TrainImg )
 //{
	// //用模板GetPatternImg()去匹配到训练图片
	// cv::Mat Transform;
	// float vDofRangeStart[4] = {-0.005f, 0.0f, 1.0f, 0.97f};
	// float vDofRangeEnd[4]   = {0.005f,  0.0f, 1.0f, 1.03f};
	// int rc = GetTransformationFromPM(TrainImg,Transform,m_fCoarseGrand,m_fFineGrand,m_fScoreThresh,vDofRangeStart,vDofRangeEnd);

	// if (rc != OK)
	// {
	//	 return;
	// }
	// //匹配到之后进行训练
	// FeedTrain(TrainImg,Transform);
 //}

 //int PMInspect::GetTransformationFromPM(IN cv::Mat TrainImg,IN cv::Mat & Transform,IN float fCoarseGrand,IN  float fFineGrand , 
	//                                    IN float fScoreThresh ,IN float * vDofRangeStart,IN float * vDofRangeEnd )
 //{
	// int rc = OK;

	// //把CV::Mat的将被匹配的图片格式转换成CV4A_IMAGE格式的将被匹配的图片
	// CV4A_IMAGE img ;
	// cv::Mat GrayTrainImg;
	// cv::cvtColor(TrainImg,GrayTrainImg,CV_RGB2GRAY);
	// if ((GrayTrainImg.depth() == CV_8U) && (! GrayTrainImg.empty()) && (GrayTrainImg.rows > 2)  && (GrayTrainImg.cols > 2))
	// {
	//	 img.imgHeight = GrayTrainImg.rows;
	//	 img.imgWidth = GrayTrainImg.cols;
	//	 img.imgType = CV4A_IMG_DATA_TYPE_8U;
	//	 img.isInitialized = 1;
	//	 img.pImg.pCharImgData = GrayTrainImg.data;
	//	 img.sizePixel = 1;
	//	 //img.channel = 1;
	//	 img.widthStep = GrayTrainImg.step;
	// }

	// //匹配
	// float fContrastThresh = 2;
	// int nNumToFind        = 1;
	// SMEE_BOOL bIgnorePolarity = SMEE_FALSE;//设置是否考虑极性
	// float fTimeLimit = 0.500;
	// float fMaxElasticity = 0;
	// CV4A_RECT_STRUCT roi;
	// roi.x = 0;
	// roi.y = 0;
	// roi.width = img.imgWidth;
	// roi.height = img.imgHeight;
	// CV4A_PM_MATCH_RESULT_STRUCT vResults[10];
	// int nResultCount = 10;
	// SMEE_BOOL bMatchSuc = SMEE_FALSE;
	// rc = CV4A_pm_match(m_model, &img, roi, fScoreThresh, fContrastThresh, nNumToFind, bIgnorePolarity, fTimeLimit, fMaxElasticity, vDofRangeStart, vDofRangeEnd, vResults, &nResultCount, &bMatchSuc,&m_nInfoCode);//进行模板匹配

	// //计算匹配到的ROI区域到初始图片中ROI区域的
	// if(rc == OK)
	// {
	//	 cv::Point2f RectCenter(Getm_vPROICenter());
	//	 Transform.create(2,3,CV_32FC1);
	//	 Transform.at<float>(0,0) = cos(vResults[0].rotation);
	//	 Transform.at<float>(0,1) = sin(vResults[0].rotation);
	//	 Transform.at<float>(1,0) = -sin(vResults[0].rotation);
	//	 Transform.at<float>(1,1) = cos(vResults[0].rotation);
	//	 Transform.at<float>(0,2) = vResults[0].x - cos(vResults[0].rotation) * RectCenter.x - sin(vResults[0].rotation) * RectCenter.y;
	//	 Transform.at<float>(1,2) = vResults[0].y + sin(vResults[0].rotation) * RectCenter.x - cos(vResults[0].rotation) * RectCenter.y;
	// }

	// return !bMatchSuc;
 //}

 //void PMInspect::SatisticalPhase()
 //{
 //    DoTrain();
 //}

 //
 //bool PMInspect::ComparePhase(IN cv::Mat TestImg,OUT cv::Mat & defect)
 //{
	// int rc = OK;
	// if (rc == OK)
	// {
	//	 cv::Mat Transform;
	//	 float vDofRangeStart[4] = {-0.005f, 0.0f, 1.0f, 0.97f};
	//	 float vDofRangeEnd[4]   = {0.005f,  0.0f, 1.0f, 1.03f};
	//	 GetTransformationFromPM(TestImg,Transform,m_fCoarseGrand,m_fFineGrand,m_fScoreThresh,vDofRangeStart,vDofRangeEnd);
	//	 vector<cv::Mat> AbsDiffImage;
	//	 Compare(TestImg,Transform, AbsDiffImage);

	//	 cv::threshold(AbsDiffImage[0],AbsDiffImage[0], 3, 255, CV_THRESH_BINARY );
	//	 cv::namedWindow("result");
	//	 cv::imshow("result", AbsDiffImage[0]);
	//	// cv::imwrite("D:\\Test_TransformImage\\匹配测试\\kingFinger\\result.bmp",AbsDiffImage[0]);
	//	 cv::waitKey(0);

	//	 vector<cv::Point> rectangle;
	//	 
	//	 int Pixelnum =0;
	//	 for (int i = 0;i<AbsDiffImage[0].rows;i++)
	//	 {
	//		 for(int j = 0;j<AbsDiffImage[0].cols;j++)
	//		 {
	//			 if(255 == AbsDiffImage[0].at<cv::Vec3f>(i,j)[0])
	//			 {
	//				 cv::Point P(i,j);
	//				 rectangle.push_back(P);
	//				 Pixelnum++;
	//			 } 
	//		 }
	//	 }

	//	 if (Pixelnum > 200)
	//	 {
	//		 cv::rectangle(TestImg,rectangle.front(),rectangle.back(),cv::Scalar(255,0,0));
	//		 cv::namedWindow("UnqualifiedImg",0);
	//		 cv::imshow("UnqualifiedImg", TestImg);
	//		 //cv::imwrite("D:\\Test_TransformImage\\匹配测试\\kingFinger\\UnqualifiedImg.bmp",TestImg);
	//		 cv::waitKey(0);
	//		 return false;
	//	 }

	//	 return true;
	// }
	// return false;
 //}

 //cv::Point2f PMInspect::Getm_vPROICenter() const
 //{
	// float CenterX = m_vPROI.x + (float)(m_vPROI.width - 1)/2;
	// float CenterY = m_vPROI.y + (float)(m_vPROI.height - 1)/2;
	// cv::Point2f RectCenter(CenterX,CenterY);
	// return RectCenter;
 //}

 //cv::Mat PMInspect::GetPatternImg() const
 //{
 //    return m_imgPattern;
 //}
