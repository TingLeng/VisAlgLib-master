/*******************************************************************************
* Copyright (C) 2011, 上海微电子装备有限公司
* All rights reserved.
* 产品号   : SSB225, SSB500, SSB300, SLD500
* 所属组件 : CV
* 模块名称 : CV
* 文件名称 : CVRANSAC_LineParamEstimator.h
* 概要描述 : CVRANSAC_LineParamEstimator定义
* 历史记录 :
* 版本    日  期    作  者  内容
*
* V1.0  2015-08-18  tuq  添加文件头部注释
* 
* 
******************************************************************************/

#ifndef _LINE_PARAM_ESTIMATOR_H_
#define _LINE_PARAM_ESTIMATOR_H_

#include "CVRANSAC_ParameterEstimator.h"

/**
* This class estimates the parameters of 2D lines.
* A 2D line is represented as: (*) dot(n,p-a)=0 
*                              where n is the line normal (|n| = 1) and 'a' is a 
*                              point on the line. 
* All points 'p' which satisfy equation (*) are on the line.
*
* The reason for choosing this line parametrization is that it can represent
* all lines, including vertical and horizontal, unlike the slope intercept (y=ax+b)
* parametrization.
*
* Author: Ziv Yaniv
*/

template<class T, class S>
class CVRANSAC_LineParamEstimator : public CVRANSAC_ParameterEstimator<T, S> 
{
public:
	CVRANSAC_LineParamEstimator(double delta)
		:CVRANSAC_ParameterEstimator(2, 4), deltaSquared(delta*delta)
	{
	}

	/**
	* Compute the line defined by the given data points.
	* @param data A vector containing two 2D points.
	* @param This vector is cleared and then filled with the computed parameters.
	*        The parameters of the line passing through these points [n_x,n_y,a_x,a_y]
	*        where ||(n_x,ny)|| = 1.
	*        If the vector contains less than two points then the resulting parameters
	*        vector is empty (size = 0).
	*/
	virtual void estimate(const std::vector<const T *> &data, std::vector<S> &parameters)
	{
		parameters.clear();
		if (data.size() < this->minForEstimate)
			return;
		double nx = data[1]->y - data[0]->y;
		double ny = data[0]->x - data[1]->x;
		double norm = sqrt(nx*nx + ny*ny);

		parameters.push_back(nx / norm);
		parameters.push_back(ny / norm);
		parameters.push_back(data[0]->x);
		parameters.push_back(data[0]->y);
	}

	/**
	* Compute a least squares estimate of the line defined by the given points.
	* This implementation is of an orthogonal least squares error.
	*
	* @param data The line should minimize the least squares error to these points.
	* @param parameters This vector is cleared and then filled with the computed parameters.
	*                   Fill this vector with the computed line parameters [n_x,n_y,a_x,a_y]
	*                   where ||(n_x,ny)|| = 1.
	*                   If the vector contains less than two points then the resulting parameters
	*                   vector is empty (size = 0).
	*/
	virtual void leastSquaresEstimate(const std::vector<const T *> &data, std::vector<S> &parameters)
	{
		double meanX, meanY, nx, ny, norm;
		double covMat11, covMat12, covMat21, covMat22; // The entries of the symmetric covarinace matrix
		int i, dataSize = (int)data.size();

		parameters.clear();
		if (data.size() < this->minForEstimate)
			return;

		meanX = meanY = 0.0;
		covMat11 = covMat12 = covMat21 = covMat22 = 0;
		for (i = 0; i < dataSize; i++)
		{
			meanX += data[i]->x;
			meanY += data[i]->y;

			covMat11 += data[i]->x * data[i]->x;
			covMat12 += data[i]->x * data[i]->y;
			covMat22 += data[i]->y * data[i]->y;
		}

		meanX /= dataSize;
		meanY /= dataSize;

		covMat11 -= dataSize*meanX*meanX;
		covMat12 -= dataSize*meanX*meanY;
		covMat22 -= dataSize*meanY*meanY;
		covMat21 = covMat12;

		if (covMat11 < 1e-12) 
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

	/**
	* Return true if the distance between the line defined by the parameters and the
	* given point is smaller than 'delta' (see constructor).
	* @param parameters The line parameters [n_x,n_y,a_x,a_y].
	* @param data Check that the distance between this point and the line is smaller than 'delta'.
	*/
	virtual int agree(const std::vector<S> &parameters, const T &data)
	{
		double signedDistance = parameters[0] * (data.x - parameters[2]) + parameters[1] * (data.y - parameters[3]);
		if (((signedDistance*signedDistance) < this->deltaSquared))
		{
			return 1;
		}
		return 0;
	}

	

	/**
	* Test the class methods, output to specified stream.
	*/
	//static void debugTest(std::ostream &out);

private:
	double deltaSquared; //given line L and point P, if dist(L,P)^2 < m_delta^2 then the point is on the line
};

#endif //_LINE_PARAM_ESTIMATOR_H_
