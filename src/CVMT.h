/*
*Copyright(C) 2007, Huazhong University of Science and Technology(HUST)

*�ļ����ƣ�AL_IP_Calibrate_Match.h
*�ļ���ʶ��For VxWorks
*�ļ�ժҪ���������ļ��ǹ�AL_IP_Calibrate.cpp���õ���ֵ���㺯��
*
*��ǰ�汾�� Ver3.1
*��    �ߣ�    л ��
*������ڣ� 2007.02.07
*
*�޸ļ�¼��  л�� 2007.04.05 Ver3.1�Ż����룻
*                       л�� ����Ϊ 2007.02.07 Ver3.0 ����SMEE������������Ӧ�޸����ƣ�
*                       л�� ����Ϊ 2007.01.30 Ver2.5 ����SMEE�����߶���������Ӧ�޸����ƣ�
*
*/
#ifndef _CVMT_H
#define _CVMT_H






//#define CVMT_SYSTEM_ERROR            0x43560700   



/******************************************************************************
������  ��  bmuav()
����˵����  ʵ��������ֵ�ֽ�
����ֵ  ��  ������60����δ��⣬�򷵻أ�1����������⣬�򷵻�1
���������  double a[]    ���M*Nʵ����A
                        int m                   A������
                        int n                   A������
                        double eps      �����ľ���Ҫ��
                        int ka              CV4A_max(m,n)+1
���������  double a[]      ����ִ�������Խ������θ�������ֵ
                        double u[]      m*m����������U
                        double v[]      n*n����������Vת��
******************************************************************************/
int bmuav(double a[],int m,int n,double u[],double v[],double eps,int ka);

/******************************************************************************
������  ��  bginv()
����˵����  �������������ֵ�ֽⷨ
����ֵ  ��  ������60����δ��⣬�򷵻أ�1����������⣬�򷵻�1
���������  double a[]    ���M*Nʵ����A
                        int m                   A������
                        int n                   A������
                        double eps      �����ľ���Ҫ��
                        int ka              CV4A_max(m,n)+1
���������  double a[]      ����ִ�������Խ������θ�������ֵ
                        double aa[]   n*m����ž���A�Ĺ�����
                        double u[]      m*m����������U
                        double v[]      n*n����������Vת��
******************************************************************************/
int bginv(double a[],int m,int n,double aa[],double eps, double u[],double v[],int ka);

/******************************************************************************
������  ��  agmiv()
����˵����  ������Է�������С��������Ĺ����淨
����ֵ  ��  ������60����δ��⣬�򷵻أ�1����������⣬�򷵻�1
���������  double a[]    ���M*Nʵ����A
                        int m                   A������
                        int n                   A������
                        double b[]    ����Ϊm��һά���󣬴�ų����������Ҷ˳�������
                        double eps      �����ľ���Ҫ��
                        int ka              CV4A_max(m,n)+1
���������  double a[]      ����ִ�������Խ������θ�������ֵ
                        double x[]      ����Ϊn��һά���󣬴�ų������̵���С���˽�
                        double aa[]   n*m����ž���A�Ĺ�����
                        double u[]      m*m����������U
                        double v[]      n*n����������Vת��
******************************************************************************/
int CVMT_agmiv(double a[],int m,int n,double b[],double x[],double aa[],double eps,double u[],double v[],int ka);

/******************************************************************************
������  ��  dngin()
����˵����  �������Է�������С��������Ĺ����淨
����ֵ  ��  ������60����δ�ﵽ����Ҫ���򷵻ظ�����0����������⣬�򷵻�1
���������  int m          �����Է������з��̸���
                        int n          �����Է�������δ֪������
                        double eps1    ������С���˽�ľ���Ҫ��
                        double eps2    ��������ֵ�ֽ���п��ƾ���Ҫ��
                        double x[]     ��ŷ����Է�����ĳ�ʼ����ֵ����������ȫΪ0
                        int ka               CV4A_max(m,n)+1
                        double A1[]      �ſɱȾ����Ӧ����
                        double A2[]      �ſɱȾ����Ӧ����
                        double A3[]      �ſɱȾ����Ӧ����
                        double A4[]      �ſɱȾ����Ӧ����
                        double A5[]      �ſɱȾ����Ӧ����
���������  double x[]       �����С���˽�
******************************************************************************/
int dngin(int m,int n,double eps1,double eps2,double x[],int ka, double A1[], double A2[], double A3[], double A4[], double A5[]);

/******************************************************************************
������  ��  dnginf()
����˵����  dngin()���õĺ���������CalibrateTsaiStep2e()�漰�����Է������������˺���ֵ
����ֵ  ��  ��
���������  int m          �����Է������з��̸���
                        int n          �����Է�������δ֪������
                        double x[]     ��ŷ����Է�����ĳ�ʼ����ֵ����������ȫΪ0
                        double A1[]      �ſɱȾ����Ӧ����
                        double A2[]      �ſɱȾ����Ӧ����
                        double A3[]      �ſɱȾ����Ӧ����
                        double A4[]      �ſɱȾ����Ӧ����
                        double A5[]      �ſɱȾ����Ӧ����
���������  double d[]       ����CalibrateTsaiStep2e()�漰�����Է������������˺���ֵ
******************************************************************************/
void dnginf(int m, int n, double x[], double d[],double A1[], double A2[], double A3[], double A4[], double A5[]);

/******************************************************************************
������  ��  dngins()
����˵����  dngin()���õĺ���������CalibrateTsaiStep2e()�漰�����Է�������ſɱȾ���
����ֵ  ��  ��
���������  int m          �����Է������з��̸���
                        int n          �����Է�������δ֪������
                        double x[]     ��ŷ����Է�����ĳ�ʼ����ֵ����������ȫΪ0
                        double A1[]      �ſɱȾ����Ӧ����
                        double A2[]      �ſɱȾ����Ӧ����
                        double A3[]      �ſɱȾ����Ӧ����
                        double A4[]      �ſɱȾ����Ӧ����
                        double A5[]      �ſɱȾ����Ӧ����
���������  double p[]       ����CalibrateTsaiStep2e()�漰�����Է�������ſɱȾ���
******************************************************************************/
void dngins(int m, int n, double x[], double p[],double A1[], double A2[], double A3[], double A4[], double A5[]);

/******************************************************************************
������  ��  brinv()
����˵����  ʵ���������ȫѡ��Ԫ��˹��Լ����
����ֵ  ��  ����0��ʾ����A���죬���ֽܷ⣬1��ʾ��������
���������  double a[]    n*n����A
            int n         ����Ľ���
���������  double a[]    n*n����A�������
******************************************************************************/
int brinv( double a[],  int n);

/******************************************************************************
������  ��  brmul()
����˵����  ʵ�������
����ֵ  ��  ��
���������  double a[]     m*n����A
                        double b[]       n*k����B
                        int m                    A������
                        int n                    A��������Ҳ����B������
                        int k                    B������
���������  double c[]       m*k����C=AB
******************************************************************************/
void brmul(double a[],double b[],int m,int n,int k,double c[]);

/******************************************************************************
������  ��  chhqr()
����˵����  ����겮��ȫ������ֵ��QR����
����ֵ  ��  -1��ʾ����jt����δ���㾫��������1��ʾ����
���������  double a[]   n*n�����H����A
                        int  n       ��H����Ľ���
                        double eps   ���ƾ���Ҫ��
                        int jt           ��������������
���������  double u[]   n������ֵ��ʵ��
                        double v[]   n������ֵ���鲿
******************************************************************************/
int chhqr(double a[],int n,double u[],double v[],double eps,int jt);

/******************************************************************************
������  ��  dqrrt()
����˵����  ��ʵϵ����������ȫ������QR����
����ֵ  ��  -1��ʾ����jt����δ���㾫��������1��ʾ����
���������  double a[]       ���n�ζ���ʽ��n+1��ϵ��
                        int n            ����ʽ���̵Ĵ���
                        double eps       ���ƾ���Ҫ��
                        int jt               ��������������
���������  double xr[]          ����n������ʵ��
                        double xi[]          ����n�������鲿
******************************************************************************/
int dqrrt(double a[],int n,double xr[], double xi[],double eps,int jt);



// add by frank 2007-5-11 10:40

// ȷ��double��ֵ�Ƿ����0
bool doubleEqualZero( double value, double precision );

// ȷ��float��ֵ�Ƿ����0
bool floatEqualZero( float value, float precision );

double ParabolaExtream(double x1, double y1, double x2, double y2, double x3, double y3);

//
//
///*
// Function:
//	CVMT_is_line_seg_para
// Parameters:
//	MvLineSeg2Df & lineseg1�� �߶�1
//	MvLineSeg2Df & lineseg2�� �߶�2
// Pre-condition:
//	None
// Post-condition:
//	None
// Returns:
//	true�� ƽ��
//	false�� ��ƽ��
// Description:
//	�ж������߶��Ƿ�ƽ��
//*/
//bool CVMT_is_line_seg_para(const MvLineSeg2Df* lineseg1, const MvLineSeg2Df* lineseg2);
//
//
///*
// Function:
//	CVMT_get_para_line_seg_dis
// Parameters:
//	MvLineSeg2Df & lineseg1�� �߶�1
//	MvLineSeg2Df & lineseg2�� �߶�2
// Pre-condition:
//	�����߶���ƽ�е�
// Post-condition:
//	None
// Returns:
//	����
// Description:
//	��ƽ��������ƽ���߶εľ���
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
//	����㵽ֱ�ߵĴ���
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
//	�߶�����ֱ���ཻ
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
	(sp-op)*(ep-op)�Ĳ�� 
	r=multiply(sp,ep,op),�õ�(sp-op)*(ep-op)�Ĳ��  
	r>0:sp��ʸ��op ep��˳ʱ�뷽��  
	r=0��op sp ep���㹲�ߣ�  
	r<0: sp��ʸ�� op ep����ʱ�뷽��
*/
//double CVMT_multiply(CV4A_POINT_2DF_STRUCT* sp,CV4A_POINT_2DF_STRUCT* ep,CV4A_POINT_2DF_STRUCT* op);
//
//
//bool CVMT_is_line_in_rect(const MvLineSeg2Df* line, const CV4A_RECT_STRUCT* region);
//
//
//bool CVMT_is_line_intersect_rect(const MvLineSeg2Df* line, const CV4A_RECT_STRUCT* region);





#endif


