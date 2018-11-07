#pragma once
#include "saturate_cast.h"
#include "Exception.h"
#include "VisionLibrary.h"


namespace VisionLibrary
{

	typedef enum
	{
		BORDER_REPLICATE,
		BORDER_REFLECT,
		BORDER_REFLECT_101,
		BORDER_WRAP,
		BORDER_CONSTANT
	}BorderType;


	/*
	Function:
	borderInterpolate
	Parameters:
	int p
	int len
	int borderType
	Pre-condition:
	None
	Post-condition:
	None
	Returns:
	OK
	Description:
	Various border types, image boundaries are denoted with '|'

	BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
	BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
	BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
	BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
	BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
	*/
	int BorderInterpolate(int p, int len, int borderType)
	{
		if ((unsigned)p < (unsigned)len)
		{

		}
		else if (borderType == BORDER_REPLICATE)
		{
			p = p < 0 ? 0 : len - 1;
		}
		else if (borderType == BORDER_REFLECT || borderType == BORDER_REFLECT_101)
		{
			int delta = borderType == BORDER_REFLECT_101;
			if (len == 1)
				return 0;
			do
			{
				if (p < 0)
					p = -p - 1 + delta;
				else
					p = len - 1 - (p - len) - delta;
			} while ((unsigned)p >= (unsigned)len);
		}
		else if (borderType == BORDER_WRAP)
		{
			if (p < 0)
				p -= ((p - len + 1) / len)*len;
			if (p >= len)
				p %= len;
		}
		else if (borderType == BORDER_CONSTANT)
		{
			p = -1;
		}
		else
		{
			assert(false);
		}
		return p;
	}








	template<class ST, class DT, class KT>
	void RowFilter(ST* src, int width, DT *dst, KT* kernel, int kernelSize)
	{
		int i;
		int j;
		double sum1, sum2;


		if (kernelSize % 2 != 1 || kernelSize > width)
		{
			throw Exception("kernelSize incorrect", __FILE__, __LINE__);
		}

		if (kernelSize == 3)
		{

			if (kernel[0] == (KT)-1.0 && kernel[1] == (KT)0.0)
			{
				for (i = 0; i<width - 1; i += 2, src += 2)
				{
					dst[i] = saturate_cast<DT>(src[2] - src[0]);
					dst[i + 1] = saturate_cast<DT>(src[3] - src[1]);
				}

				for (; i<width; i++, src++)
				{
					dst[i] = saturate_cast<DT>(src[2] - src[0]);
				}
			}
			else
			{
				for (i = 0; i<width - 1; i += 2, src += 2)
				{
					dst[i] = saturate_cast<DT>(src[0] * kernel[0] + src[1] * kernel[1] + src[2] * kernel[2]);
					dst[i + 1] = saturate_cast<DT>(src[1] * kernel[0] + src[2] * kernel[1] + src[3] * kernel[2]);
				}

				for (; i<width; i++, src++)
				{
					dst[i] = saturate_cast<DT>(src[0] * kernel[0] + src[1] * kernel[1] + src[2] * kernel[2]);
				}
			}
		}
		else
		{
			for (i = 0; i<width; i++, src++)
			{
				sum1 = 0;
				sum2 = 0;

				for (j = 0; j<kernelSize - 1; j += 2)
				{
					sum1 += src[j] * kernel[j];
					sum2 += src[j + 1] * kernel[j + 1];
				}


				for (; j<kernelSize; j++)
				{
					sum1 += src[j] * kernel[j];
				}

				dst[i] = saturate_cast<DT>(sum1 + sum2);
			}
		}


		//return OK;
	}


	template<class ST, class DT, class KT>
	void RowFilterSymm(ST* src, int width, DT *dst, KT* kernel, int kernelSize)
	{
		int i;
		int j;
		//double sum = 0;
		double sum1, sum2;


		if (kernelSize % 2 != 1 || kernelSize > width)
		{
			throw Exception("kernelSize incorrect", __FILE__, __LINE__);
		}

		if (kernelSize == 3)
		{
			if (kernel[0] == (KT)1.0 && kernel[1] == (KT)2.0)
			{

				for (i = 0; i<width - 1; i += 2, src += 2)
				{
					dst[i] = saturate_cast<DT>(src[0] + src[2] + src[1] * 2);
					dst[i + 1] = saturate_cast<DT>(src[1] + src[3] + src[2] * 2);
				}

				for (; i<width; i++, src++)
				{
					dst[i] = saturate_cast<DT>(src[0] + src[2] + src[1] * 2);
				}
			}
			else if (kernel[0] == (KT)1.0 && kernel[1] == (KT)-2.0)
			{
				for (i = 0; i<width - 1; i += 2, src += 2)
				{
					dst[i] = saturate_cast<DT>(src[0] + src[2] - src[1] * 2);
					dst[i + 1] = saturate_cast<DT>(src[1] + src[3] - src[2] * 2);
				}
				for (; i<width; i++, src++)
				{
					dst[i] = saturate_cast<DT>(src[0] + src[2] - src[1] * 2);
				}
			}
			else
			{
				for (i = 0; i<width - 1; i += 2, src += 2)
				{
					dst[i] = saturate_cast<DT>(src[0] * kernel[0] + src[2] * kernel[2] + src[1] * kernel[1]);
					dst[i + 1] = saturate_cast<DT>(src[1] * kernel[0] + src[3] * kernel[2] + src[2] * kernel[1]);
				}

				for (; i<width; i++, src++)
				{
					dst[i] = saturate_cast<DT>(src[0] * kernel[0] + src[2] * kernel[2] + src[1] * kernel[1]);
				}
			}
		}
		else
		{
			for (i = 0; i<width; i++, src++)
			{
				sum1 = 0;
				sum2 = 0;

				for (j = 0; j<kernelSize - 1; j += 2)
				{
					sum1 += src[j] * kernel[j];
					sum2 += src[j + 1] * kernel[j + 1];
				}
				for (; j<kernelSize; j++)
				{
					sum1 += src[j] * kernel[j];
				}

				dst[i] = saturate_cast<DT>(sum1 + sum2);
			}
		}


		//return OK;
	}

	template<class ST, class DT, class KT>
	void ColFilter(ST** src, int width, int count, KT* kernel,
		int nKernelSize, DT* dst, int dststep)
	{
		int i, j, k;
		//double sum;
		double sum1, sum2;
		ST** src2 = src;
		//DT* oridst = dst;

		for (i = 0; i<count; i++)
		{
			for (j = 0; j<width; j++)
			{
				sum1 = 0;
				sum2 = 0;
				for (k = 0; k<nKernelSize - 1; k += 2)
				{
					sum1 += kernel[k] * (*(src2 + k))[j];
					sum2 += kernel[k + 1] * (*(src2 + k + 1))[j];
				}

				for (; k<nKernelSize; k++)
				{
					sum1 += kernel[k] * (*(src2 + k))[j];
				}

				dst[j] = saturate_cast<DT>(sum1 + sum2);
			}

			dst = (DT*)((char*)dst + dststep);
			src2++;
		}

		//return OK;
	}


	template<class ST, class DT, class KT>
	void SepFilter2D(const ST* src, int srcWidth, int srcHeight, int srcWidthStep,
		DT* dst, int dstWidth, int dstHeight, int dstWidthStep,
		KT* kx, KT* ky, int kernelSize, bool kxSymm, bool kySymm, int borderType)
	{
		assert(srcWidth == dstWidth);
		assert(srcHeight == dstHeight);

		int rc = OK;
		int i, j, k, m;
		ST* buffer1 = NULL;
		DT* buffer2 = NULL;
		int kxSize = kernelSize;
		int kySize = kernelSize;
		int nBorderSize = kxSize / 2;
		ST* pBuf1 = NULL;
		DT* pBuf2 = NULL;

		buffer1 = new ST[srcWidth + 2 * nBorderSize];
		buffer2 = new DT[srcWidth*(srcHeight + 2 * nBorderSize)];

		pBuf1 = buffer1 + nBorderSize;
		for (i = 0; i<srcHeight; i++)
		{
			memcpy(pBuf1, (void*)(((char*)src) + i * srcWidthStep), srcWidth * sizeof(ST));

			// make row border
			for (j = 1; j <= nBorderSize; j++)
			{
				k = -j;
				m = BorderInterpolate(k, srcWidth, borderType);
				pBuf1[k] = pBuf1[m];

				k = srcWidth - 1 + j;
				m = BorderInterpolate(k, srcWidth, borderType);
				pBuf1[k] = pBuf1[m];

			}

			pBuf2 = buffer2 + (i + nBorderSize)*srcWidth;
			if (kxSymm)
			{
				RowFilterSymm<ST, DT, KT>(buffer1, srcWidth, pBuf2, kx, kxSize);
			}
			else
			{
				RowFilter<ST, DT, KT>(buffer1, srcWidth, pBuf2, kx, kxSize);
			}
			//if (rc != OK)
			//{
			//	break;
			//}
		}

		if (rc == OK)
		{
			//make col border
			pBuf2 = buffer2 + nBorderSize * srcWidth;
			for (i = 1; i <= nBorderSize; i++)
			{
				k = -i;
				m = BorderInterpolate(k, srcHeight, borderType);
				memcpy(pBuf2 + k * srcWidth, pBuf2 + m * srcWidth, srcWidth * sizeof(DT));

				k = srcHeight - 1 + i;
				m = BorderInterpolates(k, srcHeight, borderType);
				memcpy(pBuf2 + k * srcWidth, pBuf2 + m * srcWidth, srcWidth * sizeof(DT));
			}


			DT **pBuf22 = new DT *[srcHeight + 2 * nBorderSize];
			for (i = 0; i<srcHeight + 2 * nBorderSize; i++)
			{
				pBuf22[i] = buffer2 + i * srcWidth;
			}

			if (kySymm)
			{
				ColFilter<DT, DT, KT>(pBuf22, srcWidth, srcHeight, ky,
					kySize, dst, dstWidthStep);
			}
			else
			{
				ColFilter<DT, DT, KT>(pBuf22, srcWidth, srcHeight, ky,
					kySize, dst, dstWidthStep);
			}


			delete[] pBuf22;
			pBuf22 = NULL;

		}
		if (buffer1)
		{
			delete[] buffer1;
			buffer1 = NULL;
		}

		if (buffer2)
		{
			delete[] buffer2;
			buffer2 = NULL;
		}

		//return rc;
	}


	template<class ST, class DT>
	void gaussianFilter1D(ST* src, int swidth, DT* dst, int dstep, double sigma)
	{
		int rc = OK;
		int nMaxWidth = 30;
		double fDieOff = 0.00005;//0.0001;
		double ssq = sigma*sigma;
		int i, j = 0;
		int k, m;
		double *pFilterKernel = NULL;
		int nKernelSize = 0;
		ST* buffer;
		int nBorderSize;// = kxSize / 2;
		ST* pBuf = NULL;

		for (i = 1; i <= nMaxWidth; i++)
		{
			if (exp(-i*i / (2 * ssq)) < fDieOff)
			{
				break;
			}
			j = i;
		}

		if (j == 0)
		{
			j = 1;
		}

		nKernelSize = j * 2 + 1;
		pFilterKernel = new double[nKernelSize];

		double sum = 0;
		for (i = -j, k = 0; i <= j; i++, k++)
		{
			pFilterKernel[k] = exp(-(i*i) / (2 * ssq)) / (2 * PI*ssq);
			sum += pFilterKernel[k];
		}

		for (i = 0; i<nKernelSize; i++)
		{
			pFilterKernel[i] *= 1 / sum;
		}

		nBorderSize = nKernelSize / 2;
		buffer = new ST[swidth + 2 * nBorderSize];
		pBuf = buffer + nBorderSize;
		memcpy(pBuf, (void*)(((char*)src)), swidth * sizeof(ST));

		for (j = 1; j <= nBorderSize; j++)
		{
			k = -j;
			m = BorderInterpolate(k, swidth, BORDER_REFLECT);
			pBuf[k] = pBuf[m];

			k = swidth - 1 + j;
			m = BorderInterpolate(k, swidth, BORDER_REFLECT);
			pBuf[k] = pBuf[m];
		}

		RowFilter(buffer, swidth, dst, pFilterKernel, nKernelSize);


		if (pFilterKernel)
		{
			delete[] pFilterKernel;
			pFilterKernel = NULL;
		}
		if (buffer)
		{
			delete[] buffer;
			buffer = NULL;
		}

		//return rc;
	}


}
