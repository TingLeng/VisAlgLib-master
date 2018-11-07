#include"VisionLibrary.h"
#include"FindLine.h"
#include"FixPoint.h"
//#include<opencv2/core/core.hpp>
//#include<opencv2\highgui\highgui.hpp>
//#include<opencv2\imgproc\imgproc.hpp>
#include<iostream>

using namespace cv;
using namespace VisionLibrary;

int main()
{
	int rc = 0;
	
	//Mat src = imread("F:\\PackingBox\\ª“÷Ω∞Â550\\ª“÷Ω∞Â1\\5.bmp", CV_LOAD_IMAGE_COLOR);
	//if (src.empty())
	//{
	//	std::cout << "Wrong to open the image" << std::endl;
	//	return -1;
	//	rc = __LINE__;
	//}
	//Mat grayImg;
	//cvtColor(src, grayImg,CV_BGR2GRAY);
	//adaptiveThreshold(grayImg, grayImg, 255,
	//	ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,7, 0);
	//imshow("Better Image", grayImg);
	//FindLine flA, flB;
	//if (rc == 0)
	//{
	//	flA = FindLine(Point2f(1893,671), Point2f(1897,810));
	//	flA.bIsPairMode = false;
	//	flA.eMajorPolarity = ANY;
	//	flA.fAcceptFitError = 1.5;
	//	flA.fSearchDir = 0;
	//	flA.nProjectionLength = 30;
	//	flA.nNumOfCaliper = 20;
	//	flA.nNumToIgnoreAllowed = 5;
	//	flA.nSearchLenght = 40;

	//	flB = FindLine(Point2f(1614,924), Point2f(1833,921));
	//	flB.bIsPairMode = false;
	//	flB.eMajorPolarity = ANY;
	//	flB.fAcceptFitError = 1.5;
	//	flB.fSearchDir = 90;
	//	flB.nProjectionLength = 20;
	//	flB.nNumOfCaliper = 20;
	//	flB.nNumToIgnoreAllowed = 5;
	//	flB.nNumToIgnoreAllowed = 40;
	//}

	//FindLine::LineSeg lsA, lsB;
	//FindLine::Line lineA, lineB;
	//std::vector<FindLine::FindLineCaliperResult> vCaliperResultA, vCaliperResultB;

	//
	//if (rc == 0)
	//{
	//	int rc1 = flA.run(grayImg, PoseXYR(), lsA, lineA);
	//    int rc2 = flB.run(grayImg, PoseXYR(), lsB, lineB);

	//if (flA.Success())
	//	{
	//	std::cout << lineA.nx << " "<<lineA.ny<<" " << lineA.x <<" "<< lineA.y << std::endl;
	//	line(src, cv::Point(cvRound(lsA.start.x),cvRound(lsA.start.y)), cv::Point(cvRound(lsA.end.x),cvRound(lsA.end.y)), Scalar(0, 0, 255), 4, 8);
	//	imwrite("F:/FindLine.jpg", src);
	//	}
	//if (flB.Success())
	//{
	//	std::cout << lineB.nx <<" "<< lineB.ny <<" "<< lineB.x <<" "<< lineB.y << std::endl;
	//	line(src, cv::Point(cvRound(lsB.start.x), cvRound(lsB.start.y)), cv::Point(cvRound(lsB.end.x), cvRound(lsB.end.y)), Scalar(0, 0, 255), 4, 8);
	//	
	//}
	//imwrite("F:/FindLine.jpg", src);
	//	std::cout << "rc1: " << rc1 << " " << "rc2: " << rc2 <<std:: endl;
	//	std::cout << flA.Success() << std::endl;
	//	std::cout << flB.Success() << std::endl;
	//
	//if (rc1 != 0 || rc2 != 0 || !flA.Success() || !flB.Success())
	//	{
	//		rc = __LINE__;
	//	}
	//	
	//	if (rc == 0)
	//	{
	//		flA.getCaliperResult(vCaliperResultA);
	//		flB.getCaliperResult(vCaliperResultB);
 //		}

	//}
	//float cx = 0;
	//float cy = 0;
	//float innD = 0;
	//if (rc == 0)
	//{
	//	 cx = (lsA.start.x + lsA.end.x) / 2.0;
	//	 cy = (lsA.start.y + lsA.end.y) / 2.0;
	//	 innD = fabs(lineB.x*lineB.nx + lineB.y*lineB.ny - cx*lineB.nx - cy*lineB.ny);
	//}
	//std::cout << "rc:" << rc << std::endl;
	//std::cout << lsA.start << lsA.end<< std::endl;
	//std::cout << lsB.start << lsB.end << std::endl;
	//double X = (lineA.ny*lineA.ny*(lineA.y - lineB.y) + lineA.nx*lineB.ny*lineA.x - lineB.nx*lineA.ny*lineB.x) / (lineA.nx*lineB.ny - lineB.nx*lineA.ny);
	//double Y = (lineA.nx*lineB.nx*(lineA.x - lineB.x) + lineB.nx*lineA.ny*lineA.y - lineA.nx*lineB.ny*lineB.y) / (-lineA.nx*lineB.ny + lineB.nx*lineA.ny);
	//std::cout << cvRound(X) << " " << cvRound(Y) << std::endl;

	//circle(src,cv:: Point(cvRound(X), cvRound(Y)), 8, Scalar(0, 0, 255), -1, 8);

	//imwrite("F:/FindLine.jpg", src);
	int t1 = getTickCount();
	int t2;
	FixPoint^ LA = gcnew FixPoint();
	System::Drawing::Bitmap^ a;
	System::Drawing::Bitmap^ via;
	//Mat src = imread("F:\\PackingBox\\ª“÷Ω∞Â550\\ª“÷Ω∞Â1\\5.bmp", CV_LOAD_IMAGE_COLOR);
	//via = ConverToBitmap(&src);
	int i = LA->getFixPoint("F:\\PackingBox\\ª“÷Ω∞Â550\\ª“÷Ω∞Â1\\5.bmp", 1893,671,1897,810,1614,924,1833,921, a);
	cout << i << endl;
	cout << "XY: " << LA->x << " " << LA->y << endl;
	cout << "Middle Point: " << LA->x1 << " " << LA->y1<<" " <<LA->x2<<" "<<LA->y2<< endl;
	 LA->getAngleToHorizon();
	cout << LA->angleToHorizon << endl;
	//LA->transForm(1, 1, 1, 5, 5, 4, 60, 60, 60, 64, 64, 3);
	//cout << LA->aa1 << " "<<LA->bb1<<" " << LA->tt1 << endl;
	//cout << LA->aa2 << " "<<LA->bb2 <<" "<< LA->tt2 << endl;
	/*LA->transForm(1023, 974, 1040, 623, 1435, 631,179, 178.5, 154.5, 179, 153.5, 205);
	cout << LA->aa1 << endl;
	cout << LA->bb1 << endl;
	cout << LA->tt1 << endl;
	cout << LA->aa2 << endl;
	cout << LA->bb2 << endl;
	cout << LA->tt2 << endl;
	LA->pictureToWorld(LA->aa1, LA->bb1, LA->tt1, LA->aa2, LA->bb2, LA->tt2, LA->x, LA->y);
	cout << LA->posOfRealWorld_X << endl;
	cout << LA->posOfRealWorld_Y << endl;*/
	LA->diffGetPoint("C:\\Users\\Administrator\\Desktop\\tupian\\1.bmp", "C:\\Users\\Administrator\\Desktop\\tupian\\2.bmp",
		"C:\\Users\\Administrator\\Desktop\\tupian\\3.bmp", "C:\\Users\\Administrator\\Desktop\\tupian\\4.bmp");
	cout << LA->t1_x << " " << LA->t1_y << endl;
	cout << LA->t2_x << " " << LA->t2_y << endl;
	cout << LA->t3_x << " " << LA->t3_y << endl;
	
	LA->transForm(LA->t1_x, LA->t1_y, LA->t2_x, LA->t2_y, LA->t3_x, LA->t3_y, 179, 178.5, 154.5, 179, 153.5, 205);
	cout << LA->aa1 << endl;
	cout << LA->bb1 << endl;
	cout << LA->tt1 << endl;
	cout << LA->aa2 << endl;
	cout << LA->bb2 << endl;
	cout << LA->tt2 << endl;
	LA->pictureToWorld(LA->aa1, LA->bb1, LA->tt1, LA->aa2, LA->bb2, LA->tt2, LA->x, LA->y);
	cout << LA->posOfRealWorld_X << endl;
	cout << LA->posOfRealWorld_Y << endl; 

	t2 = getTickCount();
	printf("TIME IS : %f \r\n", (t2 - t1) / getTickFrequency());
	Mat IMG;
	int j = ConvertBitmapToMat(a, IMG);
	imwrite("F:/FindLine.jpg", IMG);
	imshow("HH", IMG);
	waitKey(0);
	return 0;
}