#pragma once
#include "limit.h"

namespace VisionLibrary
{
	template<typename _Tp> inline static _Tp saturate_cast(signed int v) { return _Tp(v); }
	template<typename _Tp> inline static _Tp saturate_cast(unsigned int v) { return _Tp(v); }
	template<typename _Tp> inline static _Tp saturate_cast(signed short v) { return _Tp(v); }
	template<typename _Tp> inline static _Tp saturate_cast(unsigned short v) { return _Tp(v); }
	template<typename _Tp> inline static _Tp saturate_cast(float v) { return _Tp(v); }
	template<typename _Tp> inline static _Tp saturate_cast(double v) { return _Tp(v); }

	// int -> short
	template<> inline short saturate_cast<short>(signed int v)
	{
		return (short)CV4A_max(CV_MIN_SHORT, CV4A_min(CV_MAX_SHORT, v));
	}

	// int -> ushort
	template<> inline unsigned short saturate_cast<unsigned short>(signed int v)
	{
		return (unsigned short)CV4A_max(0, CV4A_min(CV_MAX_USHORT, v));
	}

	// int -> char
	template<> inline  char saturate_cast<char>(signed int v)
	{
		return (char)CV4A_max(-128, CV4A_min(127, v));
	}

	// int -> uchar
	template<> inline  unsigned char saturate_cast<unsigned char>(signed int v)
	{
		return (unsigned char)CV4A_max(0, CV4A_min(255, v));
	}


	// uint -> short
	template<> inline  short saturate_cast<short>(unsigned int v)
	{
		return (short)CV4A_max(0, CV4A_min(CV_MAX_SHORT, v));
	}

	// uint -> ushort
	template<> inline  unsigned short saturate_cast<unsigned short>(unsigned int v)
	{
		return (unsigned short)CV4A_max(0, CV4A_min(CV_MAX_USHORT, v));
	}

	// uint -> char
	template<> inline  char saturate_cast<char>(unsigned int v)
	{
		return (char)CV4A_max(0, CV4A_min(127, v));
	}

	// uint -> uchar
	template<> inline  unsigned char saturate_cast<unsigned char>(unsigned int v)
	{
		return (unsigned char)CV4A_max(0, CV4A_min(255, v));
	}

	// short -> char
	template<> inline  char saturate_cast<char>(short v)
	{
		return (char)CV4A_max(-128, CV4A_min(127, v));
	}

	// short -> uchar
	template<> inline  unsigned char saturate_cast<unsigned char>(short v)
	{
		return (unsigned char)CV4A_max(0, CV4A_min(255, v));
	}

	// ushort -> char
	template<> inline  char saturate_cast<char>(unsigned short v)
	{
		return (char)CV4A_max(0, CV4A_min(127, v));
	}

	// ushort -> uchar
	template<> inline  unsigned char saturate_cast<unsigned char>(unsigned short v)
	{
		return (unsigned char)CV4A_max(0, CV4A_min(255, v));
	}

	// float -> int
	template<> inline  int saturate_cast<int>(float v)
	{
		return (int)CV4A_max(CV_MIN_INT, CV4A_min(CV_MAX_INT, v));
	}

	// float -> uint
	template<> inline  unsigned int saturate_cast<unsigned int>(float v)
	{
		return (unsigned int)CV4A_max(0, CV4A_min(CV_MAX_UINT, v));
	}

	// float -> short
	template<> inline  short saturate_cast<short>(float v)
	{
		return (short)CV4A_max(CV_MIN_SHORT, CV4A_min(CV_MAX_SHORT, v));
	}

	// float -> ushort
	template<> inline  unsigned short saturate_cast<unsigned short>(float v)
	{
		return (unsigned short)CV4A_max(0, CV4A_min(CV_MAX_USHORT, v));
	}

	// float -> char
	template<>  inline  char saturate_cast<char>(float v)
	{
		return (char)CV4A_max(-128, CV4A_min(127, v));
	}

	// float -> uchar
	template<>  inline  unsigned char saturate_cast<unsigned char>(float v)
	{
		return (unsigned char)CV4A_max(0, CV4A_min(255, v));
	}

	// double -> float
	template<> inline  float saturate_cast<float>(double v)
	{
		return (float)v;
	}

	// double -> int
	template<> inline  int saturate_cast<int>(double v)
	{
		return (int)CV4A_max(CV_MIN_INT, CV4A_min(CV_MAX_INT, v));
	}

	// double -> uint
	template<> inline  unsigned int saturate_cast<unsigned int>(double v)
	{
		return (unsigned int)CV4A_max(0, CV4A_min(CV_MAX_UINT, v));
	}

	// double -> short
	template<> inline  short saturate_cast<short>(double v)
	{
		return (short)CV4A_max(CV_MIN_SHORT, CV4A_min(CV_MAX_SHORT, v));
	}

	// double -> ushort
	template<> inline  unsigned short saturate_cast<unsigned short>(double v)
	{
		return (unsigned short)CV4A_max(0, CV4A_min(CV_MAX_USHORT, v));
	}

	// double -> char
	template<>  inline  char saturate_cast<char>(double v)
	{
		return (char)CV4A_max(-128, CV4A_min(127, v));
	}

	// double -> uchar
	template<> inline  unsigned char saturate_cast<unsigned char>(double v)
	{
		return (unsigned char)CV4A_max(0, CV4A_min(255, v));
	}
}

