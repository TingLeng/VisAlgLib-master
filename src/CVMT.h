/*
*Copyright(C) 2007, Huazhong University of Science and Technology(HUST)

*文件名称：AL_IP_Calibrate_Match.h
*文件标识：For VxWorks
*文件摘要：本程序文件是供AL_IP_Calibrate.cpp调用的数值计算函数
*
*当前版本： Ver3.1
*作    者：    谢 威
*完成日期： 2007.02.07
*
*修改记录：  谢威 2007.04.05 Ver3.1优化代码；
*                       谢威 王大为 2007.02.07 Ver3.0 根据SMEE测试用例做相应修改完善；
*                       谢威 王大为 2007.01.30 Ver2.5 根据SMEE代码走读报告做相应修改完善；
*
*/
#ifndef _CVMT_H
#define _CVMT_H






//#define CVMT_SYSTEM_ERROR            0x43560700   



/******************************************************************************
函数名  ：  bmuav()
函数说明：  实矩阵奇异值分解
返回值  ：  若迭代60次仍未求解，则返回－1，若正常求解，则返回1
输入参数：  double a[]    存放M*N实矩阵A
                        int m                   A的行数
                        int n                   A的列数
                        double eps      给定的精度要求
                        int ka              CV4A_max(m,n)+1
输出参数：  double a[]      函数执行完后其对角线依次给出奇异值
                        double u[]      m*m左奇异向量U
                        double v[]      n*n右奇异向量V转置
******************************************************************************/
int bmuav(double a[],int m,int n,double u[],double v[],double eps,int ka);

/******************************************************************************
函数名  ：  bginv()
函数说明：  求解广义逆的奇异值分解法
返回值  ：  若迭代60次仍未求解，则返回－1，若正常求解，则返回1
输入参数：  double a[]    存放M*N实矩阵A
                        int m                   A的行数
                        int n                   A的列数
                        double eps      给定的精度要求
                        int ka              CV4A_max(m,n)+1
输出参数：  double a[]      函数执行完后其对角线依次给出奇异值
                        double aa[]   n*m，存放矩阵A的广义逆
                        double u[]      m*m左奇异向量U
                        double v[]      n*n右奇异向量V转置
******************************************************************************/
int bginv(double a[],int m,int n,double aa[],double eps, double u[],double v[],int ka);

/******************************************************************************
函数名  ：  agmiv()
函数说明：  求解线性方程组最小二乘问题的广义逆法
返回值  ：  若迭代60次仍未求解，则返回－1，若正常求解，则返回1
输入参数：  double a[]    存放M*N实矩阵A
                        int m                   A的行数
                        int n                   A的列数
                        double b[]    长度为m的一维矩阵，存放超定方程组右端常数向量
                        double eps      给定的精度要求
                        int ka              CV4A_max(m,n)+1
输出参数：  double a[]      函数执行完后其对角线依次给出奇异值
                        double x[]      长度为n的一维矩阵，存放超定方程的最小二乘解
                        double aa[]   n*m，存放矩阵A的广义逆
                        double u[]      m*m左奇异向量U
                        double v[]      n*n右奇异向量V转置
******************************************************************************/
int CVMT_agmiv(double a[],int m,int n,double b[],double x[],double aa[],double eps,double u[],double v[],int ka);

/******************************************************************************
函数名  ：  dngin()
函数说明：  求解非线性方程组最小二乘问题的广义逆法
返回值  ：  若迭代60次仍未达到精度要求，则返回负数或0，若正常求解，则返回1
输入参数：  int m          非线性方程组中方程个数
                        int n          非线性方程组中未知数个数
                        double eps1    控制最小二乘解的精度要求
                        double eps2    用于奇异值分解得中控制精度要求
                        double x[]     存放非线性方程组的初始近似值，各分量不全为0
                        int ka               CV4A_max(m,n)+1
                        double A1[]      雅可比矩阵对应函数
                        double A2[]      雅可比矩阵对应函数
                        double A3[]      雅可比矩阵对应函数
                        double A4[]      雅可比矩阵对应函数
                        double A5[]      雅可比矩阵对应函数
输出参数：  double x[]       存放最小二乘解
******************************************************************************/
int dngin(int m,int n,double eps1,double eps2,double x[],int ka, double A1[], double A2[], double A3[], double A4[], double A5[]);

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
void dnginf(int m, int n, double x[], double d[],double A1[], double A2[], double A3[], double A4[], double A5[]);

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
void dngins(int m, int n, double x[], double p[],double A1[], double A2[], double A3[], double A4[], double A5[]);

/******************************************************************************
函数名  ：  brinv()
函数说明：  实矩阵求逆的全选主元高斯－约旦法
返回值  ：  返回0表示矩阵A奇异，不能分解，1表示正常返回
输入参数：  double a[]    n*n矩阵A
            int n         矩阵的阶数
输出参数：  double a[]    n*n矩阵A的逆矩阵
******************************************************************************/
int brinv( double a[],  int n);

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
void brmul(double a[],double b[],int m,int n,int k,double c[]);

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
int chhqr(double a[],int n,double u[],double v[],double eps,int jt);

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
int dqrrt(double a[],int n,double xr[], double xi[],double eps,int jt);



// add by frank 2007-5-11 10:40

// 确定double数值是否等于0
bool doubleEqualZero( double value, double precision );

// 确定float数值是否等于0
bool floatEqualZero( float value, float precision );

double ParabolaExtream(double x1, double y1, double x2, double y2, double x3, double y3);

//
//
///*
// Function:
//	CVMT_is_line_seg_para
// Parameters:
//	MvLineSeg2Df & lineseg1， 线段1
//	MvLineSeg2Df & lineseg2， 线段2
// Pre-condition:
//	None
// Post-condition:
//	None
// Returns:
//	true， 平行
//	false， 不平行
// Description:
//	判断两个线段是否平行
//*/
//bool CVMT_is_line_seg_para(const MvLineSeg2Df* lineseg1, const MvLineSeg2Df* lineseg2);
//
//
///*
// Function:
//	CVMT_get_para_line_seg_dis
// Parameters:
//	MvLineSeg2Df & lineseg1， 线段1
//	MvLineSeg2Df & lineseg2， 线段2
// Pre-condition:
//	两个线段是平行的
// Post-condition:
//	None
// Returns:
//	距离
// Description:
//	求平面内两个平行线段的距离
//*/
//double CVMT_get_para_line_seg_dis(const MvLineSeg2Df* lineseg1, const MvLineSeg2Df* lineseg2);
//
//
//
//double CVMT_get_pt_line_dis(float k, float b, float x, float y);
//
//
///*
// Function:
//	CVMT_get_pt_line_ver
// Parameters:
//	float k
//	float b
//	float x
//	float y
//	float * vx
//	float * vy
// Pre-condition:
//	None
// Post-condition:
//	None
// Returns:
//	OK
// Description:
//	计算点到直线的垂足
//*/
//void CVMT_get_pt_line_ver(float k, float b, float x, float y, float* vx, float *vy);
//
///*
// Function:
//	CVMT_get_line_seg_intersector
// Parameters:
//	double x1
//	double y1
//	double x2
//	double y2
//	double x3
//	double y3
//	double x4
//	double y4
//	double * x
//	double * y
// Pre-condition:
//	None
// Post-condition:
//	None
// Returns:
//	OK
// Description:
//	None
//*/
//bool CVMT_get_line_seg_intersector(double x1, double y1,double x2, double y2,
//										double x3, double y3,double x4, double y4,
//										double *x, double *y);
//
//
///*
// Function:
//	CVMT_get_line_ins
// Parameters:
//	const MvLineSeg2Df * lineseg1
//	const MvLineSeg2Df * lineseg2
// Pre-condition:
//	线段所在直线相交
// Post-condition:
//	None
// Returns:
//	OK
// Description:
//	None
//*/
//bool CVMT_get_line_ins(const MvLineSeg2Df* lineseg1, const MvLineSeg2Df* lineseg2, CV4A_POINT_2DF_STRUCT* ins);


//
///*
// Function:
//	CVMT_is_point_on_line_seg
// Parameters:
//	const MvLineSeg2Df * lineseg
//	MvPoint2Df pt
// Pre-condition:
//	None
// Post-condition:
//	None
// Returns:
//	OK
// Description:
//	None
//*/
//bool CVMT_is_point_on_line_seg(const MvLineSeg2Df* lineseg, MvPoint2Df pt);

/*
 Function:
	CVMT_multiply
 Parameters:
	MvPoint2Df sp
	MvPoint2Df ep
	MvPoint2Df op
 Pre-condition:
	None
 Post-condition:
	None
 Returns:
	OK
 Description:
	(sp-op)*(ep-op)的叉积 
	r=multiply(sp,ep,op),得到(sp-op)*(ep-op)的叉积  
	r>0:sp在矢量op ep的顺时针方向；  
	r=0：op sp ep三点共线；  
	r<0: sp在矢量 op ep的逆时针方向
*/
//double CVMT_multiply(CV4A_POINT_2DF_STRUCT* sp,CV4A_POINT_2DF_STRUCT* ep,CV4A_POINT_2DF_STRUCT* op);
//
//
//bool CVMT_is_line_in_rect(const MvLineSeg2Df* line, const CV4A_RECT_STRUCT* region);
//
//
//bool CVMT_is_line_intersect_rect(const MvLineSeg2Df* line, const CV4A_RECT_STRUCT* region);





#endif


