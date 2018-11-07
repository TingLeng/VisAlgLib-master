/*******************************************************************************
* Copyright (C) 2011, 上海微电子装备有限公司
* All rights reserved.
* 产品号   : SSB225, SSB500, SSB300, SLD500
* 所属组件 : CV
* 模块名称 : CV
* 文件名称 : CVMT.cpp
* 概要描述 :
* 历史记录 :
* 版本    日  期    作  者  内容
*
* V1.0  2015-08-18  tuq  添加文件头部注释
*
*
******************************************************************************/

#include <math.h>
#include <float.h>

#include "CVMT.h"
#include "limit.h"
#include <assert.h>
#include <corecrt_malloc.h>
#include "VisionLibrary.h"
#include <vcruntime_string.h>
#include "Exception.h"


static void ppp(double a[], double e[], double s[], double v[], int m, int n);

static void sss(double fg[2], double cs[2]);

/******************************************************************************
函数名  ：  bmuav()
函数说明：  实矩阵奇异值分解
返回值  ：  若迭代600次仍未求解，则返回－1，若正常求解，则返回1
输入参数：  double a[]    存放M*N实矩阵A
int m                   A的行数
int n                   A的列数
double eps      给定的精度要求
int ka              max(m,n)+1
输出参数：  double a[]      函数执行完后其对角线依次给出奇异值
double u[]      m*m左奇异向量U
double v[]      n*n右奇异向量V转置
******************************************************************************/
int bmuav(double a[], int m, int n, double u[], double v[], double eps, int ka)
{

	/*int i,j,k,l,it,ll,kk,ix,iy,mm,nn,iz,m1,ks;
	double d,dd,t,sm,sm1,em1,sk,ek,b,c,shh,fg[2],cs[2];*/



	int i = 0;
	int j = 0;
	int k = 0;
	int l = 0;
	int it = 0;
	int ll = 0;
	int kk = 0;
	int ix = 0;
	int iy = 0;
	int mm = 0;
	int nn = 0;
	int iz = 0;
	int m1 = 0;
	int ks = 0;
	double d = 0;
	double dd = 0;
	double t = 0;
	double sm = 0;
	double sm1 = 0;
	double em1 = 0;
	double sk = 0;
	double ek = 0;
	double b = 0;
	double c = 0;
	double shh = 0;
	double fg[2] = { 0 };
	double cs[2] = { 0 };
	double *s = NULL;
	double *e = NULL;
	double *w = NULL;


	assert(ka == CV4A_max(m, n) + 1);

	s = (double*)malloc(ka * sizeof(double));
	e = (double*)malloc(ka * sizeof(double));
	w = (double*)malloc(ka * sizeof(double));
	it = 600; k = n;
	if (m - 1 < n) k = m - 1;
	l = m;
	if (n - 2 < m) l = n - 2;
	if (l < 0) l = 0;
	ll = k;
	if (l > k) ll = l;
	if (ll >= 1)
	{
		for (kk = 1; kk <= ll; kk++)
		{
			if (kk <= k)
			{
				d = 0.0;
				for (i = kk; i <= m; i++)
				{
					ix = (i - 1)*n + kk - 1; d = d + a[ix] * a[ix];
				}
				s[kk - 1] = sqrt(d);
				if (s[kk - 1] != 0.0)
				{
					ix = (kk - 1)*n + kk - 1;
					if (a[ix] != 0.0)
					{
						s[kk - 1] = fabs(s[kk - 1]);
						if (a[ix] < 0.0) s[kk - 1] = -s[kk - 1];
					}
					for (i = kk; i <= m; i++)
					{
						iy = (i - 1)*n + kk - 1;
						a[iy] = a[iy] / s[kk - 1];
					}
					a[ix] = 1.0 + a[ix];
				}
				s[kk - 1] = -s[kk - 1];
			}
			if (n >= kk + 1)
			{
				for (j = kk + 1; j <= n; j++)
				{
					if ((kk <= k) && (s[kk - 1] != 0.0))
					{
						d = 0.0;
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1)*n + kk - 1;
							iy = (i - 1)*n + j - 1;
							d = d + a[ix] * a[iy];
						}
						d = -d / a[(kk - 1)*n + kk - 1];
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1)*n + j - 1;
							iy = (i - 1)*n + kk - 1;
							a[ix] = a[ix] + d*a[iy];
						}
					}
					e[j - 1] = a[(kk - 1)*n + j - 1];
				}
			}/*end of if(n)*/
			if (kk <= k)
			{
				for (i = kk; i <= m; i++)
				{
					ix = (i - 1)*m + kk - 1; iy = (i - 1)*n + kk - 1;
					u[ix] = a[iy];
				}
			}
			if (kk <= l)
			{
				d = 0.0;
				for (i = kk + 1; i <= n; i++)
					d = d + e[i - 1] * e[i - 1];
				e[kk - 1] = sqrt(d);
				if (e[kk - 1] != 0.0)
				{
					if (e[kk] != 0.0)
					{
						e[kk - 1] = fabs(e[kk - 1]);
						if (e[kk] < 0.0) e[kk - 1] = -e[kk - 1];
					}
					for (i = kk + 1; i <= n; i++)
						e[i - 1] = e[i - 1] / e[kk - 1];
					e[kk] = 1.0 + e[kk];
				}
				e[kk - 1] = -e[kk - 1];
				if ((kk + 1 <= m) && (e[kk - 1] != 0.0))
				{
					for (i = kk + 1; i <= m; i++) w[i - 1] = 0.0;
					for (j = kk + 1; j <= n; j++)
						for (i = kk + 1; i <= m; i++)
							w[i - 1] = w[i - 1] + e[j - 1] * a[(i - 1)*n + j - 1];
					for (j = kk + 1; j <= n; j++)
						for (i = kk + 1; i <= m; i++)
						{
							ix = (i - 1)*n + j - 1;
							a[ix] = a[ix] - w[i - 1] * e[j - 1] / e[kk];
						}
				}
				for (i = kk + 1; i <= n; i++)
					v[(i - 1)*n + kk - 1] = e[i - 1];
			}
		}
	} /*end 0f if(ll)*/

	mm = n;
	if (m + 1 < n) mm = m + 1;
	if (k < n) s[k] = a[k*n + k];
	if (m < mm) s[mm - 1] = 0.0;
	if (l + 1 < mm) e[l] = a[l*n + mm - 1];
	e[mm - 1] = 0.0;
	nn = m;
	if (m > n) nn = n;
	if (nn >= k + 1)
	{
		for (j = k + 1; j <= nn; j++)
		{
			for (i = 1; i <= m; i++)
				u[(i - 1)*m + j - 1] = 0.0;
			u[(j - 1)*m + j - 1] = 1.0;
		}
	}
	if (k >= 1)
	{
		for (ll = 1; ll <= k; ll++)
		{
			kk = k - ll + 1; iz = (kk - 1)*m + kk - 1;
			if (s[kk - 1] != 0.0)
			{
				if (nn >= kk + 1)
					for (j = kk + 1; j <= nn; j++)
					{
						d = 0.0;
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1)*m + kk - 1;
							iy = (i - 1)*m + j - 1;
							d = d + u[ix] * u[iy] / u[iz];
						}
						d = -d;
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1)*m + j - 1;
							iy = (i - 1)*m + kk - 1;
							u[ix] = u[ix] + d*u[iy];
						}
					}
				for (i = kk; i <= m; i++)
				{
					ix = (i - 1)*m + kk - 1; u[ix] = -u[ix];
				}
				u[iz] = 1.0 + u[iz];
				if (kk - 1 >= 1)
					for (i = 1; i <= kk - 1; i++)
						u[(i - 1)*m + kk - 1] = 0.0;
			}
			else
			{
				for (i = 1; i <= m; i++)
					u[(i - 1)*m + kk - 1] = 0.0;
				u[(kk - 1)*m + kk - 1] = 1.0;
			}
		}
	}/* end of for(ll)*/
	for (ll = 1; ll <= n; ll++)
	{
		kk = n - ll + 1; iz = kk*n + kk - 1;
		if ((kk <= l) && (e[kk - 1] != 0.0))
		{
			for (j = kk + 1; j <= n; j++)
			{
				d = 0.0;
				for (i = kk + 1; i <= n; i++)
				{
					ix = (i - 1)*n + kk - 1; iy = (i - 1)*n + j - 1;
					d = d + v[ix] * v[iy] / v[iz];
				}
				d = -d;
				for (i = kk + 1; i <= n; i++)
				{
					ix = (i - 1)*n + j - 1; iy = (i - 1)*n + kk - 1;
					v[ix] = v[ix] + d*v[iy];
				}
			}
		}
		for (i = 1; i <= n; i++)
			v[(i - 1)*n + kk - 1] = 0.0;
		v[iz - n] = 1.0;
	}/* end of for(ll)*/
	for (i = 1; i <= m; i++)
		for (j = 1; j <= n; j++)
			a[(i - 1)*n + j - 1] = 0.0;
	/**/
	m1 = mm; it = 600;
	while (1 == 1)
	{
		if (mm == 0)
		{
			ppp(a, e, s, v, m, n);
			free(s); free(e); free(w); return(1);
		}
		if (it == 0)
		{
			ppp(a, e, s, v, m, n);
			free(s); free(e); free(w); return(-1);
		}
		kk = mm - 1;
		while ((kk != 0) && (fabs(e[kk - 1]) != 0.0))
		{
			d = fabs(s[kk - 1]) + fabs(s[kk]);
			dd = fabs(e[kk - 1]);
			if (dd > eps*d) kk = kk - 1;
			else e[kk - 1] = 0.0;
		}
		if (kk == mm - 1)
		{
			kk = kk + 1;
			if (s[kk - 1] < 0.0)
			{
				s[kk - 1] = -s[kk - 1];
				for (i = 1; i <= n; i++)
				{
					ix = (i - 1)*n + kk - 1; v[ix] = -v[ix];
				}
			}
			while ((kk != m1) && (s[kk - 1] < s[kk]))
			{
				d = s[kk - 1]; s[kk - 1] = s[kk]; s[kk] = d;
				if (kk < n)
					for (i = 1; i <= n; i++)
					{
						ix = (i - 1)*n + kk - 1; iy = (i - 1)*n + kk;
						d = v[ix]; v[ix] = v[iy]; v[iy] = d;
					}
				if (kk < m)
					for (i = 1; i <= m; i++)
					{
						ix = (i - 1)*m + kk - 1; iy = (i - 1)*m + kk;
						d = u[ix]; u[ix] = u[iy]; u[iy] = d;
					}
				kk = kk + 1;
			}
			it = 600;
			mm = mm - 1;
		}/*if (mm==0)*/
		else
		{
			ks = mm;
			while ((ks > kk) && (fabs(s[ks - 1]) != 0.0))
			{
				d = 0.0;
				if (ks != mm) d = d + fabs(e[ks - 1]);
				if (ks != kk + 1) d = d + fabs(e[ks - 2]);
				dd = fabs(s[ks - 1]);
				if (dd > eps*d) ks = ks - 1;
				else s[ks - 1] = 0.0;
			}
			if (ks == kk)
			{
				kk = kk + 1;
				d = fabs(s[mm - 1]);
				t = fabs(s[mm - 2]);
				if (t > d) d = t;
				t = fabs(e[mm - 2]);
				if (t > d) d = t;
				t = fabs(s[kk - 1]);
				if (t > d) d = t;
				t = fabs(e[kk - 1]);
				if (t > d) d = t;
				sm = s[mm - 1] / d;
				sm1 = s[mm - 2] / d;
				em1 = e[mm - 2] / d;
				sk = s[kk - 1] / d; ek = e[kk - 1] / d;
				b = ((sm1 + sm)*(sm1 - sm) + em1*em1) / 2.0;
				c = sm*em1; c = c*c; shh = 0.0;
				if ((b != 0.0) || (c != 0.0))
				{
					shh = sqrt(b*b + c);
					if (b < 0.0) shh = -shh;
					shh = c / (b + shh);
				}
				fg[0] = (sk + sm)*(sk - sm) - shh;
				fg[1] = sk*ek;
				for (i = kk; i <= mm - 1; i++)
				{
					sss(fg, cs);
					if (i != kk) e[i - 2] = fg[0];
					fg[0] = cs[0] * s[i - 1] + cs[1] * e[i - 1];
					e[i - 1] = cs[0] * e[i - 1] - cs[1] * s[i - 1];
					fg[1] = cs[1] * s[i];
					s[i] = cs[0] * s[i];
					if ((cs[0] != 1.0) || (cs[1] != 0.0))
						for (j = 1; j <= n; j++)
						{
							ix = (j - 1)*n + i - 1;
							iy = (j - 1)*n + i;
							d = cs[0] * v[ix] + cs[1] * v[iy];
							v[iy] = -cs[1] * v[ix] + cs[0] * v[iy];
							v[ix] = d;
						}
					sss(fg, cs);
					s[i - 1] = fg[0];
					fg[0] = cs[0] * e[i - 1] + cs[1] * s[i];
					s[i] = -cs[1] * e[i - 1] + cs[0] * s[i];
					fg[1] = cs[1] * e[i];
					e[i] = cs[0] * e[i];
					if (i < m)
						if ((cs[0] != 1.0) || (cs[1] != 0.0))
							for (j = 1; j <= m; j++)
							{
								ix = (j - 1)*m + i - 1;
								iy = (j - 1)*m + i;
								d = cs[0] * u[ix] + cs[1] * u[iy];
								u[iy] = -cs[1] * u[ix] + cs[0] * u[iy];
								u[ix] = d;
							}
				}/*end of for (i=kk; i<=mm-1; i++)*/
				e[mm - 2] = fg[0];
				it = it - 1;
			}/*end of if (ks==kk)*/
			else
			{
				if (ks == mm)
				{
					kk = kk + 1;
					fg[1] = e[mm - 2]; e[mm - 2] = 0.0;
					for (ll = kk; ll <= mm - 1; ll++)
					{
						i = mm + kk - ll - 1;
						fg[0] = s[i - 1];
						sss(fg, cs);
						s[i - 1] = fg[0];
						if (i != kk)
						{
							fg[1] = -cs[1] * e[i - 2];
							e[i - 2] = cs[0] * e[i - 2];
						}
						if ((cs[0] != 1.0) || (cs[1] != 0.0))
							for (j = 1; j <= n; j++)
							{
								ix = (j - 1)*n + i - 1;
								iy = (j - 1)*n + mm - 1;
								d = cs[0] * v[ix] + cs[1] * v[iy];
								v[iy] = -cs[1] * v[ix] + cs[0] * v[iy];
								v[ix] = d;
							}
					}
				}
				else
				{
					kk = ks + 1;
					fg[1] = e[kk - 2];
					e[kk - 2] = 0.0;
					for (i = kk; i <= mm; i++)
					{
						fg[0] = s[i - 1];
						sss(fg, cs);
						s[i - 1] = fg[0];
						fg[1] = -cs[1] * e[i - 1];
						e[i - 1] = cs[0] * e[i - 1];
						if ((cs[0] != 1.0) || (cs[1] != 0.0))
							for (j = 1; j <= m; j++)
							{
								ix = (j - 1)*m + i - 1;
								iy = (j - 1)*m + kk - 2;
								d = cs[0] * u[ix] + cs[1] * u[iy];
								u[iy] = -cs[1] * u[ix] + cs[0] * u[iy];
								u[ix] = d;
							}
					}
				}/*end  of while ((ks>kk)&&(fabs(s[ks-1])!=0.0))*/
			}
		}/*end of while (1==1)*/
	}/*end of if (k>=1)*/
	return(1);
}

/*bmuav()调用的子函数*/
static void ppp(double a[], double e[], double s[], double v[], int m, int n)
{
	int i, j, p, q;
	double d;
	if (m >= n)
		i = n;
	else i = m;
	for (j = 1; j <= i - 1; j++)
	{
		assert(j - 1 >= 0 && j - 1 < CV4A_max(m, n) + 1);



		a[(j - 1)*n + j - 1] = s[j - 1];
		a[(j - 1)*n + j] = e[j - 1];
	}

	a[(i - 1)*n + i - 1] = s[i - 1];

	if (m < n)
		a[(i - 1)*n + i] = e[i - 1];

	for (i = 1; i <= n - 1; i++)
		for (j = i + 1; j <= n; j++)
		{
			p = (i - 1)*n + j - 1;
			q = (j - 1)*n + i - 1;
			d = v[p];
			v[p] = v[q];
			v[q] = d;
		}
	return;
}

/*bmuav()调用的子函数*/
static void sss(double fg[2], double cs[2])
{
	double r, d;
	if ((fabs(fg[0]) + fabs(fg[1])) == 0.0)
	{
		cs[0] = 1.0; cs[1] = 0.0; d = 0.0;
	}
	else
	{
		d = sqrt(fg[0] * fg[0] + fg[1] * fg[1]);
		if (fabs(fg[0]) > fabs(fg[1]))
		{
			d = fabs(d);
			if (fg[0] < 0.0) d = -d;
		}
		if (fabs(fg[1]) >= fabs(fg[0]))
		{
			d = fabs(d);
			if (fg[1] < 0.0) d = -d;
		}
		cs[0] = fg[0] / d; cs[1] = fg[1] / d;
	}
	r = 1.0;
	if (fabs(fg[0]) > fabs(fg[1])) r = cs[1];
	else
		if (cs[0] != 0.0) r = 1.0 / cs[0];
	fg[0] = d; fg[1] = r;
	return;
}
/******************************************************************************
函数名  ：  bginv()
函数说明：  求解广义逆的奇异值分解法
返回值  ：  若迭代600次仍未求解，则返回－1，若正常求解，则返回1
输入参数：  double a[]    存放M*N实矩阵A
int m                   A的行数
int n                   A的列数
double eps      给定的精度要求
int ka              max(m,n)+1
输出参数：  double a[]      函数执行完后其对角线依次给出奇异值
double aa[]   n*m，存放矩阵A的广义逆
double u[]      m*m左奇异向量U
double v[]      n*n右奇异向量V转置
******************************************************************************/
int bginv(double a[], int m, int n, double aa[], double eps, double u[], double v[], int ka)
{
	int i, j, k, l, t, p, q, f;
	i = bmuav(a, m, n, u, v, eps, ka);
	if (i < 0) return(-1);
	j = n;
	if (m < n) j = m;
	j = j - 1;
	k = 0;
	while ((k <= j) && (a[k*n + k] != 0.0)) k = k + 1;
	k = k - 1;

	for (i = 0; i <= n - 1; i++)
		for (j = 0; j <= m - 1; j++)
		{
			t = i*m + j; aa[t] = 0.0;
			for (l = 0; l <= k; l++)
			{
				f = l*n + i; p = j*m + l; q = l*n + l;
				aa[t] = aa[t] + v[f] * u[p] / a[q];
			}
		}
	return(1);
}
/******************************************************************************
函数名  ：  agmiv()
函数说明：  求解线性方程组最小二乘问题的广义逆法
返回值  ：  若迭代600次仍未求解，则返回－1，若正常求解，则返回1
输入参数：  double a[]    存放M*N实矩阵A
int m                   A的行数
int n                   A的列数
double b[]    长度为m的一维矩阵，存放超定方程组右端常数向量
double eps      给定的精度要求
int ka              max(m,n)+1
输出参数：  double a[]      函数执行完后其对角线依次给出奇异值
double x[]      长度为n的一维矩阵，存放超定方程的最小二乘解
double aa[]   n*m，存放矩阵A的广义逆
double u[]      m*m左奇异向量U
double v[]      n*n右奇异向量V转置
******************************************************************************/
int CVMT_agmiv(double a[], int m, int n, double b[], double x[], double aa[], double eps, double u[], double v[], int ka)
{
	int rc = OK;
	int i, j;
	i = bginv(a, m, n, aa, eps, u, v, ka);

	if (i < 0)
	{
		throw VisionLibrary::Exception("agmiv error", __FILE__, __LINE__);
	}
	for (i = 0; i <= n - 1; i++)
	{
		x[i] = 0.0;
		for (j = 0; j <= m - 1; j++)
			x[i] = x[i] + aa[i*m + j] * b[j];
	}
	return OK;
}
/******************************************************************************
函数名  ：  dngin()
函数说明：  求解非线性方程组最小二乘问题的广义逆法
返回值  ：  若迭代600次仍未达到精度要求，则返回负数或0，若正常求解，则返回1
输入参数：  int m          非线性方程组中方程个数
int n          非线性方程组中未知数个数
double eps1    控制最小二乘解的精度要求
double eps2    用于奇异值分解得中控制精度要求
double x[]     存放非线性方程组的初始近似值，各分量不全为0
int ka               max(m,n)+1
double A1[]      雅可比矩阵对应函数
double A2[]      雅可比矩阵对应函数
double A3[]      雅可比矩阵对应函数
double A4[]      雅可比矩阵对应函数
double A5[]      雅可比矩阵对应函数
输出参数：  double x[]       存放最小二乘解
******************************************************************************/
int dngin(int m, int n, double eps1, double eps2, double x[], int ka, double A1[], double A2[], double A3[], double A4[], double A5[])
{
	int rc = OK;
	int i, j, k, l, kk, jt;
	double y[10], b[10], alpha, z, h2, y1, y2, y3, y0, h1;
	double *p, *d, *pp, *dx, *u, *v, *w;
	p = (double*)malloc(m*n * sizeof(double));
	d = (double*)malloc(m * sizeof(double));
	pp = (double*)malloc(n*m * sizeof(double));
	dx = (double*)malloc(n * sizeof(double));
	u = (double*)malloc(m*m * sizeof(double));
	v = (double*)malloc(n*n * sizeof(double));
	w = (double*)malloc(ka * sizeof(double));
	l = 600; alpha = 1.0;
	while (l > 0)
	{
		dnginf(m, n, x, d, A1, A2, A3, A4, A5);
		dngins(m, n, x, p, A1, A2, A3, A4, A5);
		rc = CVMT_agmiv(p, m, n, d, dx, pp, eps2, u, v, ka);

		if (rc != OK)
		{
			free(p); free(d); free(pp); free(w);
			free(dx); free(u); free(v);
			//rc = CVCC_cv_log_error(CVMT_SYSTEM_ERROR, rc, __FILE__, __LINE__, "dngin error");
			throw VisionLibrary::Exception("dngin error", __FILE__, __LINE__);
			//return(rc);
		}
		j = 0; jt = 1; h2 = 0.0;
		while (jt == 1)
		{
			jt = 0;
			if (j <= 2)
				z = alpha + 0.01*j;
			else
				z = h2;
			for (i = 0; i <= n - 1; i++)
				w[i] = x[i] - z*dx[i];

			dnginf(m, n, w, d, A1, A2, A3, A4, A5);
			y1 = 0.0;
			for (i = 0; i <= m - 1; i++)
				y1 = y1 + d[i] * d[i];
			for (i = 0; i <= n - 1; i++)
				w[i] = x[i] - (z + 0.00001)*dx[i];
			dnginf(m, n, w, d, A1, A2, A3, A4, A5);
			y2 = 0.0;
			for (i = 0; i <= m - 1; i++)
				y2 = y2 + d[i] * d[i];
			y0 = (y2 - y1) / 0.00001;


			if (fabs(y0) > 1.0e-10)
			{
				h1 = y0; h2 = z;

				if (j == 0)
				{
					y[0] = h1; b[0] = h2;
				}

				else
				{
					y[j] = h1; kk = 0; k = 0;
					while ((kk == 0) && (k <= j - 1))
					{
						y3 = h2 - b[k];
						if (fabs(y3) + 1.0 == 1.0)
							kk = 1;
						else
							h2 = (h1 - y[k]) / y3;
						k = k + 1;
					}
					b[j] = h2;
					if (kk != 0)
						b[j] = 1.0e+35;
					h2 = 0.0;
					for (k = j - 1; k >= 0; k--)
						h2 = -y[k] / (b[k + 1] + h2);
					h2 = h2 + b[0];
				}
				j = j + 1;
				if (j <= 7)
					jt = 1;
				else
					z = h2;
			}
		}/*end of while (jt==1)*/
		alpha = z; y1 = 0.0; y2 = 0.0;

		for (i = 0; i <= n - 1; i++)
		{
			dx[i] = -alpha*dx[i];
			x[i] = x[i] + dx[i];
			y1 = y1 + fabs(dx[i]);
			y2 = y2 + fabs(x[i]);
		}
		if (y1 < eps1*y2)
		{
			free(p); free(pp); free(d); free(w);
			free(dx); free(u); free(v);

			//rc = CVCC_cv_log_error(CVMT_SYSTEM_ERROR, rc, __FILE__, __LINE__, "dngin error");
			throw VisionLibrary::Exception("dngin error", __FILE__, __LINE__);
			//return rc;
		}
		l = l - 1;
	}/* end of  while (l>0)*/
	free(p); free(pp); free(d); free(dx);
	free(u); free(v); free(w);
	return OK;
}
/******************************************************************************
函数名  ：  dnginf()
函数说明：  dngin()调用的函数，计算CalibrateTsaiStep2e()涉及非线性方程组各方程左端函数值
返回值  ：  无
输入参数：  int m          非线性方程组中方程个数
int n          非线性方程组中未知数个数
double x[]     存放非线性方程组的初始近似值，各分量不全为0
double A1[]      雅可比矩阵对应函数
double A2[]      雅可比矩阵对应函数
double A3[]      雅可比矩阵对应函数
double A4[]      雅可比矩阵对应函数
double A5[]      雅可比矩阵对应函数
输出参数：  double d[]       计算CalibrateTsaiStep2e()涉及非线性方程组各方程左端函数值
******************************************************************************/
void dnginf(int m, int n, double x[], double d[], double A1[], double A2[], double A3[], double A4[], double A5[])
{
	int i;
	for (i = 0; i < m; i++)
	{
		d[i] = A1[i] * x[0] + A2[i] * x[1] + A3[i] * x[2] + A4[i] * x[1] * x[2] + A5[i];
	}
}
/******************************************************************************
函数名  ：  dngins()
函数说明：  dngin()调用的函数，计算CalibrateTsaiStep2e()涉及非线性方程组的雅可比矩阵
返回值  ：  无
输入参数：  int m          非线性方程组中方程个数
int n          非线性方程组中未知数个数
double x[]     存放非线性方程组的初始近似值，各分量不全为0
double A1[]      雅可比矩阵对应函数
double A2[]      雅可比矩阵对应函数
double A3[]      雅可比矩阵对应函数
double A4[]      雅可比矩阵对应函数
double A5[]      雅可比矩阵对应函数
输出参数：  double p[]       计算CalibrateTsaiStep2e()涉及非线性方程组的雅可比矩阵
******************************************************************************/
void dngins(int m, int n, double x[], double p[],
	double A1[], double A2[], double A3[], double A4[], double A5[])
{
	int i;
	for (i = 0; i < m; i++)
	{
		p[i * 3 + 0] = A1[i];
		p[i * 3 + 1] = A2[i] + A4[i] * x[2];
		p[i * 3 + 2] = A3[i] + A4[i] * x[1];
	}

}
/******************************************************************************
函数名  ：  brinv()
函数说明：  实矩阵求逆的全选主元高斯－约旦法
返回值  ：  返回0表示矩阵A奇异，不能分解，1表示正常返回
输入参数：  double a[]    n*n矩阵A
int n         矩阵的阶数
输出参数：  double a[]    n*n矩阵A的逆矩阵
******************************************************************************/
int brinv(double a[], int n)
{
	int *is, *js, i, j, k, l, u, v;
	double d, p;
	is = (int*)malloc(n * sizeof(int));
	js = (int*)malloc(n * sizeof(int));

	memset(js, 0, n * sizeof(int));
	memset(is, 0, n * sizeof(int));

	for (k = 0; k <= n - 1; k++)
	{
		d = 0.0;
		for (i = k; i <= n - 1; i++)
			for (j = k; j <= n - 1; j++)
			{
				l = i*n + j;
				p = fabs(a[l]);
				if (p > d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}

		if (fabs(d) < DBL_EPSILON)
		{
			free(is);
			free(js);
			printf("err**not inv\n");
			return(0);
		}

		if (is[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = is[k] * n + j;
				p = a[u]; a[u] = a[v]; a[v] = p;
			}

		if (js[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k; v = i*n + js[k];
				p = a[u]; a[u] = a[v]; a[v] = p;
			}

		l = k*n + k;
		a[l] = 1.0 / a[l];
		for (j = 0; j <= n - 1; j++)
			if (j != k)
			{
				u = k*n + j; a[u] = a[u] * a[l];
			}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
				for (j = 0; j <= n - 1; j++)
					if (j != k)
					{
						u = i*n + j;
						a[u] = a[u] - a[i*n + k] * a[k*n + j];
					}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
			{
				u = i*n + k;
				a[u] = -a[u] * a[l];
			}
	}

	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = js[k] * n + j;
				p = a[u]; a[u] = a[v]; a[v] = p;
			}

		if (is[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k; v = i*n + is[k];
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
	}

	free(is);
	free(js);

	return(1);
}

/******************************************************************************
函数名  ：  brmul()
函数说明：  实矩阵相乘
返回值  ：  无
输入参数：  double a[]     m*n矩阵A
double b[]       n*k矩阵B
int m                    A的行数
int n                    A的列数，也就是B的行数
int k                    B的列数
输出参数：  double c[]       m*k矩阵C=AB
******************************************************************************/
void brmul(double a[], double b[], int m, int n, int k, double c[])
{
	int i, j, l, u;
	for (i = 0; i <= m - 1; i++)
		for (j = 0; j <= k - 1; j++)
		{
			u = i*k + j;
			c[u] = 0.0;

			for (l = 0; l <= n - 1; l++)
				c[u] = c[u] + a[i*n + l] * b[l*k + j];
		}
	return;
}
/******************************************************************************
函数名  ：  chhqr()
函数说明：  求赫申伯格全部特征值的QR方法
返回值  ：  -1表示迭代jt次仍未满足精度条件，1表示正常
输入参数：  double a[]   n*n存放上H矩阵A
int  n       上H矩阵的阶数
double eps   控制精度要求
int jt           控制最大迭代次数
输出参数：  double u[]   n个特征值的实部
double v[]   n个特征值的虚部
******************************************************************************/
int chhqr(double a[], int n, double u[], double v[], double eps, int jt)
{
	int m, it, i, j, k, l, ii, jj, kk, ll;
	double b, c, w, g, xy, p, q, r, x, s, e, f, z, y;
	it = 0; m = n;
	while (m != 0)
	{
		l = m - 1;
		while ((l > 0) && (fabs(a[l*n + l - 1]) > eps*
			(fabs(a[(l - 1)*n + l - 1]) + fabs(a[l*n + l])))) l = l - 1;
		ii = (m - 1)*n + m - 1; jj = (m - 1)*n + m - 2;
		kk = (m - 2)*n + m - 1; ll = (m - 2)*n + m - 2;
		if (l == m - 1)
		{
			u[m - 1] = a[(m - 1)*n + m - 1]; v[m - 1] = 0.0;
			m = m - 1; it = 0;
		}
		else if (l == m - 2)
		{
			b = -(a[ii] + a[ll]);
			c = a[ii] * a[ll] - a[jj] * a[kk];
			w = b*b - 4.0*c;
			y = sqrt(fabs(w));
			if (w > 0.0)
			{
				xy = 1.0;
				if (b < 0.0) xy = -1.0;
				u[m - 1] = (-b - xy*y) / 2.0;
				u[m - 2] = c / u[m - 1];
				v[m - 1] = 0.0; v[m - 2] = 0.0;
			}
			else
			{
				u[m - 1] = -b / 2.0; u[m - 2] = u[m - 1];
				v[m - 1] = y / 2.0; v[m - 2] = -v[m - 1];
			}
			m = m - 2; it = 0;
		}
		else
		{
			if (it >= jt)
			{
				printf("fail\n");
				return(-1);
			}
			it = it + 1;
			for (j = l + 2; j <= m - 1; j++)
				a[j*n + j - 2] = 0.0;
			for (j = l + 3; j <= m - 1; j++)
				a[j*n + j - 3] = 0.0;
			for (k = l; k <= m - 2; k++)
			{
				if (k != l)
				{
					p = a[k*n + k - 1]; q = a[(k + 1)*n + k - 1];
					r = 0.0;
					if (k != m - 2) r = a[(k + 2)*n + k - 1];
				}
				else
				{
					x = a[ii] + a[ll];
					y = a[ll] * a[ii] - a[kk] * a[jj];
					ii = l*n + l; jj = l*n + l + 1;
					kk = (l + 1)*n + l; ll = (l + 1)*n + l + 1;
					p = a[ii] * (a[ii] - x) + a[jj] * a[kk] + y;
					q = a[kk] * (a[ii] + a[ll] - x);
					r = a[kk] * a[(l + 2)*n + l + 1];
				}
				if ((fabs(p) + fabs(q) + fabs(r)) != 0.0)
				{
					xy = 1.0;
					if (p < 0.0) xy = -1.0;
					s = xy*sqrt(p*p + q*q + r*r);
					if (k != l) a[k*n + k - 1] = -s;
					e = -q / s; f = -r / s; x = -p / s;
					y = -x - f*r / (p + s);
					g = e*r / (p + s);
					z = -x - e*q / (p + s);
					for (j = k; j <= m - 1; j++)
					{
						ii = k*n + j; jj = (k + 1)*n + j;
						p = x*a[ii] + e*a[jj];
						q = e*a[ii] + y*a[jj];
						r = f*a[ii] + g*a[jj];
						if (k != m - 2)
						{
							kk = (k + 2)*n + j;
							p = p + f*a[kk];
							q = q + g*a[kk];
							r = r + z*a[kk]; a[kk] = r;
						}
						a[jj] = q; a[ii] = p;
					}
					j = k + 3;
					if (j >= m - 1) j = m - 1;
					for (i = l; i <= j; i++)
					{
						ii = i*n + k; jj = i*n + k + 1;
						p = x*a[ii] + e*a[jj];
						q = e*a[ii] + y*a[jj];
						r = f*a[ii] + g*a[jj];
						if (k != m - 2)
						{
							kk = i*n + k + 2;
							p = p + f*a[kk];
							q = q + g*a[kk];
							r = r + z*a[kk]; a[kk] = r;
						}
						a[jj] = q; a[ii] = p;
					}
				}
			}
		}
	}
	return(1);
}
/******************************************************************************
函数名  ：  dqrrt()
函数说明：  求实系数代数方程全部根的QR方法
返回值  ：  -1表示迭代jt次仍未满足精度条件，1表示正常
输入参数：  double a[]       存放n次多项式的n+1个系数
int n            多项式方程的次数
double eps       控制精度要求
int jt               控制最大迭代次数
输出参数：  double xr[]          返回n个根的实部
double xi[]          返回n个根的虚部
******************************************************************************/
int dqrrt(double a[], int n, double xr[], double xi[], double eps, int jt)
{
	int i, j;
	double *q;
	q = (double*)malloc(n*n * sizeof(double));
	for (j = 0; j <= n - 1; j++)
		q[j] = -a[n - j - 1] / a[n];
	for (j = n; j <= n*n - 1; j++)
		q[j] = 0.0;
	for (i = 0; i <= n - 2; i++)
		q[(i + 1)*n + i] = 1.0;
	i = chhqr(q, n, xr, xi, eps, jt);
	free(q); return(i);
}


// add by frank 2007-5-11 10:40

bool doubleEqualZero(double value, double precision)
{
	bool result = 0;

	if ((value > -precision) && (value < precision))
	{
		result = true;
	}
	else
	{
		result = false;
	}

	return result;
}


// 确定float数值是否等于0
bool floatEqualZero(float value, float precision)
{
	bool result = 0;

	if ((value > -precision) && (value < precision))
	{
		result = true;
	}
	else
	{
		result = false;
	}

	return result;
}

//求抛物线极值的x 坐标
/*
利用克莱默法则求解线性方程组
ax^2+bx+c=y, x=x1,x2,x3, y=y1,y2,y3
求出a ,b ,c 后，返回极值的坐标-b/(2a)
*/
double ParabolaExtream(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double detA = pow(x1, 2)*x2 + pow(x3, 2)*x1 + pow(x2, 2)*x3 - pow(x3, 2)*x2 - pow(x1, 2)*x3 - pow(x2, 2)*x1;
	double A_x[9] = { x2 - x3,                     x3 - x1 ,                     x1 - x2,
		pow(x3,2) - pow(x2,2),        pow(x1,2) - pow(x3,2),        pow(x2,2) - pow(x1,2),
		x3*pow(x2,2) - x2*pow(x3,2),  x1*pow(x3,2) - x3*pow(x1,2),  x2*pow(x1,2) - x1*pow(x2,2) };
	double A_1[9];
	int i = 0;
	double a, b, c;

	for (i = 0; i < 9; i++)
	{
		A_1[i] = A_x[i] / detA;
	}
	a = A_1[0] * y1 + A_1[1] * y2 + A_1[2] * y3;
	b = A_1[3] * y1 + A_1[4] * y2 + A_1[5] * y3;
	c = A_1[6] * y1 + A_1[7] * y2 + A_1[8] * y3;

	return (0 - b) / (2 * a);
}
//
//bool CVMT_is_line_seg_para( const MvLineSeg2Df* lineseg1, const MvLineSeg2Df* lineseg2 )
//{
//	double dx1, dy1;
//	double dx2, dy2;
//	
//	bool line1v, line2v;
//
//	dx1 = lineseg1->x1 - lineseg1->x2;
//	dy1 = lineseg1->y1 - lineseg1->y2;
//	dx2 = lineseg2->x1 - lineseg2->x2;
//	dy2 = lineseg2->y1 - lineseg2->y2;
//
//	line1v = doubleEqualZero(dx1, 0.001);
//	line2v = doubleEqualZero(dx2, 0.001);
//
//	if (line1v && line2v)
//	{
//		return true;
//	}
//	else if ((line1v && !line2v) || (!line1v && line2v))
//	{
//		return false;
//	}
//	else
//	{
//		if (fabs(dy1/dx1-dy2/dx2) < 0.01)
//		{
//			return true;
//		}
//	}
//
//	return false;
//}
//
//double CVMT_get_para_line_seg_dis( const MvLineSeg2Df* lineseg1, const MvLineSeg2Df* lineseg2 )
//{
//	double a, b, c1, c2;
//	double dx1, dy1;
//
//	dx1 = lineseg1->x1 - lineseg1->x2;
//	dy1 = lineseg1->y1 - lineseg1->y2;
//
//	b = -dx1;
//	a = dy1;
//	
//	c1 = -a*lineseg1->x1 - b*lineseg1->y1;
//	c2 = -a*lineseg2->x1 - b*lineseg2->y1;
//
//	return fabs(c1-c2)/sqrt(a*a+b*b);
//}
//
//
//
///*
//   Determine the intersection point of two line segments
//   Return FALSE if the lines don't intersect
//*/
//bool CVMT_get_line_seg_intersector(double x1, double y1,double x2, double y2,
//										double x3, double y3,double x4, double y4,
//										double *x, double *y)
//{
//   double mua,mub;
//   double denom,numera,numerb;
//
//   denom  = (y4-y3) * (x2-x1) - (x4-x3) * (y2-y1);
//   numera = (x4-x3) * (y1-y3) - (y4-y3) * (x1-x3);
//   numerb = (x2-x1) * (y1-y3) - (y2-y1) * (x1-x3);
//
//   /* Are the line coincident? */
//   if (fabs(numera) < 0.0001 && fabs(numerb) < 0.0001 && fabs(denom) < 0.0001) 
//   {
//      *x = (x1 + x2) / 2;
//      *y = (y1 + y2) / 2;
//      return true;
//   }
//
//   /* Are the line parallel */
//   if (fabs(denom) < 0.0001) 
//   {
//      *x = 0;
//      *y = 0;
//      return false;
//   }
//
//   /* Is the intersection along the the segments */
//   mua = numera / denom;
//   mub = numerb / denom;
//   if (mua < 0 || mua > 1 || mub < 0 || mub > 1)
//   {
//      *x = 0;
//      *y = 0;
//      return false;
//   }
//   *x = x1 + mua * (x2 - x1);
//   *y = y1 + mua * (y2 - y1);
//   return true;
//}
//
//
//
//bool CVMT_get_line_ins(const MvLineSeg2Df* lineseg1, const MvLineSeg2Df* lineseg2, CV4A_POINT_2DF_STRUCT* ins)
//{
//	double a1, b1, c1;
//	double a2, b2, c2;
//
//	double dx1, dy1;
//	double dx2, dy2;
//	
//
//	dx1 = lineseg1->x1 - lineseg1->x2;
//	dy1 = lineseg1->y1 - lineseg1->y2;
//	dx2 = lineseg2->x1 - lineseg2->x2;
//	dy2 = lineseg2->y1 - lineseg2->y2;
//
//	
//	b1 = -dx1;
//	a1 = dy1;
//
//	b2 = -dx2;
//	a2 = dy2;
//
//	c1 = -a1*lineseg1->x1 - b1*lineseg1->y1;
//	c2 = -a2*lineseg2->x1 - b2*lineseg2->y1;
//
//
//	double d=a1*b2-a2*b1;  
//	if(fabs(d)<0.001) // 不相交   
//		return false;  
//	ins->x = (c2*b1-c1*b2)/d;  
//	ins->y = (a2*c1-a1*c2)/d;  
//	return true; 
//}

//
//double CVMT_multiply(CV4A_POINT_2DF_STRUCT* sp,CV4A_POINT_2DF_STRUCT *ep,CV4A_POINT_2DF_STRUCT *op)  
//{  
//	return((sp->x-op->x)*(ep->y-op->y) - (ep->x-op->x)*(sp->y-op->y));  
//}  
//
//bool CVMT_is_line_in_rect( const MvLineSeg2Df* line, const CV4A_RECT_STRUCT* region )
//{
//	if (line->x1 >= region->x && line->x1 < region->x+region->width &&
//		line->y1 >= region->y && line->y1 < region->y+region->height &&
//		line->x2 >= region->x && line->x2 < region->x+region->width &&
//		line->y2 >= region->y && line->y2 < region->y+region->height)
//	{
//		return true;
//	}
//
//	return false;
//}
//
//bool CVMT_is_line_intersect_rect( const MvLineSeg2Df* line, const CV4A_RECT_STRUCT* region )
//{
//	double x, y;
//	if (CVMT_get_line_seg_intersector(line->x1, line->y1, 
//		line->x2, line->y2, 
//		region->x, region->y, 
//		region->x+region->width-1, region->y, &x, &y))
//	{
//		return true;
//	}
//
//	if (CVMT_get_line_seg_intersector(line->x1, line->y1, 
//		line->x2, line->y2, 
//		region->x, region->y, 
//		region->x, region->y+region->height-1, &x, &y))
//	{
//		return true;
//	}
//
//	if (CVMT_get_line_seg_intersector(line->x1, line->y1, 
//		line->x2, line->y2, 
//		region->x+region->width-1, region->y, 
//		region->x+region->width-1, region->y+region->height-1, &x, &y))
//	{
//		return true;
//	}
//
//	if (CVMT_get_line_seg_intersector(line->x1, line->y1, 
//		line->x2, line->y2, 
//		region->x, region->y+region->height-1, 
//		region->x+region->width-1, region->y+region->height-1, &x, &y))
//	{
//		return true;
//	}
//
//	return false;
//}
//
//double CVMT_get_pt_line_dis( float k, float b, float x, float y )
//{
//	float A = k;
//	float B = -1;
//	float C = b;
//
//	return fabs(A*x+B*y+C)/sqrt(A*A+B*B);
//}
//
//void CVMT_get_pt_line_ver( float k, float b, float x, float y, float* vx, float *vy )
//{
//	float A1 = k;
//	float B1 = -1;
//	float C1 = b;
//
//	float A2 = -1/k;
//	float B2 = -1;
//	float C2 = y-A2*x;
//
//	float tmp = A1*B2-B1*A2;
//
//	*vx = -(C1*B2-B1*C2) / tmp;
//	*vy = -(A1*C2-A2*C1) / tmp;
//}






//bool CVMT_is_point_on_line_seg( const MvLineSeg2Df* lineseg, MvPoint2Df pt )
//{
//	MvPoint2Df le, ls;
//	le.x = lineseg->x1;
//	le.y = lineseg->y1;
//	ls.x = lineseg->x2;
//	ls.y = lineseg->y2;
//	return ((CVMT_multiply(&le，&pt，&ls)==0)  
//		&& ( ( (p.x-l.s.x) * (p.x-l.e.x) <=0 ) && ( (p.y-l.s.y)*(p.y-l.e.y) <=0 ) )   );  
//}

