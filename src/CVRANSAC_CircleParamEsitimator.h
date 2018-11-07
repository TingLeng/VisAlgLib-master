/*******************************************************************************
* Copyright (C) 2011, 上海微电子装备有限公司
* All rights reserved.
* 产品号   : SSB225, SSB500, SSB300, SLD500
* 所属组件 : CV
* 模块名称 : CV
* 文件名称 : CVRANSAC_CircleParamEsitimator.h
* 概要描述 : CVRANSAC_CircleParamEsitimator定义
* 历史记录 :
* 版本    日  期    作  者  内容
*
* V1.0  2015-08-18  tuq  添加文件头部注释
* 
* 
******************************************************************************/
#ifndef _CVRANSAC_CIRCLE_PARAM_ESTIMATOR_H
#define _CVRANSAC_CIRCLE_PARAM_ESTIMATOR_H

#include "CVRANSAC_ParameterEstimator.h"

template <class T, class S>
class CVRANSAC_CircleParamEsitimator: public CVRANSAC_ParameterEstimator<T, S> 
{
public:
	CVRANSAC_CircleParamEsitimator(double delta)
		:CVRANSAC_ParameterEstimator(3, 3),
		deltaSquared(delta*delta)
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

		if (data.size() < minForEstimate)
		{
			return;
		}

		double x, y, r;

		CVRANSAC_CircleParamEsitimator::FitCircleLeastSquare(data, &x, &y, &r);

		parameters.push_back(x);
		parameters.push_back(y);
		parameters.push_back(r);
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
		parameters.clear();

		if (data.size() < minForEstimate)
		{
			return;
		}

		double x, y, r;

		CVRANSAC_CircleParamEsitimator::FitCircleLeastSquare(data, &x, &y, &r);

		parameters.push_back(x);
		parameters.push_back(y);
		parameters.push_back(r);
	}

	/**
	* Return true if the distance between the line defined by the parameters and the
	* given point is smaller than 'delta' (see constructor).
	* @param parameters The line parameters [n_x,n_y,a_x,a_y].
	* @param data Check that the distance between this point and the line is smaller than 'delta'.
	*/
	virtual int agree(const std::vector<S> &parameters, const T &data)
	{
		const double *pPara = &(parameters[0]);
		double dx = data.x - pPara[0];
		double dy = data.y - pPara[1];
		double dis = sqrt(dx*dx + dy*dy) - pPara[2];
		if (dis*dis < deltaSquared)
		{
			return 1;
		}

		return 0;
	}

	/**
	* Test the class methods, output to specified stream.
	*/
	//static void debugTest(std::ostream &out);


	static int FitCircleLeastSquare(const std::vector<T > &data,double *centerX, double *centerY,double *radius )
	{
		int rc = OK;
		int dataSize = data.size();
		int i = 0;

		double *a = NULL;
		double *b = NULL;
		double *x = NULL;
		double D, E, F;
		double *aa = NULL;//     n*m，存放矩阵A的广义逆
		double *u = NULL;//     m*m左奇异向量U
		double *v = NULL;//      n*n右奇异向量V转置 

		if (rc == OK)
		{
			if (data.size() < 3)
			{
				throw Exception("Not enough points(points < 3),fit circle least square error!", __FILE__, __LINE__);
			}

			if (data.size() > 3500)
			{
				throw Exception("fit circle least square Points too much, maybe slow", __FILE__, __LINE__);
			}
		}


		if (rc == OK)
		{
			a = (double *)malloc(sizeof(double)*dataSize * 3);
			b = (double *)malloc(sizeof(double)*dataSize);  //
			x = (double *)malloc(sizeof(double) * 3);
			aa = (double *)malloc(sizeof(double)*dataSize * 3);
			u = (double *)malloc(sizeof(double)*dataSize*dataSize);//
			v = (double *)malloc(sizeof(double) * 3 * 3);

			if (!a || !b || !x || !aa || !u || !v)
			{
				throw Exception("estimate circle least square parameter error", __FILE__, __LINE__);
			}
		}

		if (rc == OK)
		{
			for (i = 0; i < dataSize; i++)
			{
				b[i] = -(data[i].x*data[i].x + data[i].y*data[i].y);
			}
			for (i = 0; i < dataSize; i++)
			{
				a[i * 3 + 0] = data[i].x;
				a[i * 3 + 1] = data[i].y;
				a[i * 3 + 2] = 1;
			}
		}

		if (rc == OK)
		{
			rc = CVMT_agmiv(a, dataSize, 3, b, x, aa, 0.0000001, u, v, (dataSize + 1));
			if (rc != OK)
			{
				throw Exception("CVMT_agmiv error!", __FILE__, __LINE__);
			}
		}

		if (rc == OK)
		{
			D = x[0];
			E = x[1];
			F = x[2];

			*centerX = -D / 2;
			*centerY = -E / 2;
			*radius = sqrt(D*D + E*E - 4 * F) / 2;
		}

		if (a)
		{
			free(a);
			a = NULL;
		}
		if (b)
		{
			free(b);
			b = NULL;
		}
		if (x)
		{
			free(x);
			x = NULL;
		}
		if (aa)
		{
			free(aa);
			aa = NULL;
		}
		if (u)
		{
			free(u);
			u = NULL;
		}
		if (v)
		{
			free(v);
			v = NULL;
		}
		return rc;
	}

	static int FitCircleLeastSquare(const std::vector<const T *> &data, double *x, double* y, double* r)
	{
		int rc = OK;
		int dataSize = data.size();
		int i = 0;

		double *a = NULL;
		double *b = NULL;
		double *x = NULL;
		double D, E, F;
		double *aa = NULL;//     n*m，存放矩阵A的广义逆
		double *u = NULL;//     m*m左奇异向量U
		double *v = NULL;//      n*n右奇异向量V转置 

		if (rc == OK)
		{
			if (data.size() < 3)
			{
				throw Exception("Not enough points(points < 3),fit circle least square error!", __FILE__, __LINE__);
			}

			if (data.size() > 3500)
			{
				throw Exception("fit circle least square Points too much, maybe slow", __FILE__, __LINE__);
			}
		}


		if (rc == OK)
		{
			a = (double *)malloc(sizeof(double)*dataSize * 3);
			b = (double *)malloc(sizeof(double)*dataSize);  //
			x = (double *)malloc(sizeof(double) * 3);
			aa = (double *)malloc(sizeof(double)*dataSize * 3);
			u = (double *)malloc(sizeof(double)*dataSize*dataSize);//
			v = (double *)malloc(sizeof(double) * 3 * 3);

			if (!a || !b || !x || !aa || !u || !v)
			{
				throw Exception("estimate circle least square parameter error", __FILE__, __LINE__);
			}
		}

		if (rc == OK)
		{
			for (i = 0; i < dataSize; i++)
			{
				b[i] = -(data[i]->x*data[i]->x + data[i]->y*data[i]->y);
			}
			for (i = 0; i < dataSize; i++)
			{
				a[i * 3 + 0] = data[i]->x;
				a[i * 3 + 1] = data[i]->y;
				a[i * 3 + 2] = 1;
			}
		}

		if (rc == OK)
		{
			rc = CVMT_agmiv(a, dataSize, 3, b, x, aa, 0.0000001, u, v, (dataSize + 1));
			if (rc != OK)
			{
				throw Exception("CVMT_agmiv error!", __FILE__, __LINE__);
			}
		}

		if (rc == OK)
		{
			D = x[0];
			E = x[1];
			F = x[2];

			*centerX = -D / 2;
			*centerY = -E / 2;
			*radius = sqrt(D*D + E*E - 4 * F) / 2;
		}

		if (a)
		{
			free(a);
			a = NULL;
		}
		if (b)
		{
			free(b);
			b = NULL;
		}
		if (x)
		{
			free(x);
			x = NULL;
		}
		if (aa)
		{
			free(aa);
			aa = NULL;
		}
		if (u)
		{
			free(u);
			u = NULL;
		}
		if (v)
		{
			free(v);
			v = NULL;
		}
		return rc;
	}

private:
	double deltaSquared;
};


#endif
