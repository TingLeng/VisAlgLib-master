#pragma once

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef CV4A_max
#define CV4A_max(a, b) ((a) > (b) ? a : b)
#endif


#ifndef CV4A_min
#define CV4A_min(a, b) ((a) > (b) ? b : a)
#endif


#ifndef CV_MAX_UINT
#define CV_MAX_UINT ((unsigned int)(~0)) 
#endif


#ifndef CV_MAX_INT
#define CV_MAX_INT ((int)(CV_MAX_UINT >> 1))
#endif


#ifndef CV_MIN_INT
#define CV_MIN_INT ((int)(((unsigned int)CV_MAX_INT)+1))
#endif



#ifndef CV_MAX_USHORT
#define CV_MAX_USHORT ((unsigned short)(~0)) 
#endif


#ifndef CV_MAX_SHORT
#define CV_MAX_SHORT ((short)(CV_MAX_USHORT >> 1))
#endif


#ifndef CV_MIN_SHORT
#define CV_MIN_SHORT ((short)(((unsigned short)CV_MAX_SHORT)+1))
#endif


#ifndef CV_MAX_DOUBLE
#define CV_MAX_DOUBLE (1.79769e+308)
#endif


#ifndef CV_DOUBLE_EPS
#define CV_DOUBLE_EPS (0.00001)
#endif