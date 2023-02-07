//--------------------------------------------------------------------------------
// Copyright 2019-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
//
// 
// * Derivative works may append the above copyright notice but should not remove or modify earlier notices.
//
// MIT License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
// associated documentation files (the "Software"), to deal in the Software without restriction, including without 
// limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS 
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF 
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
#ifndef DEF_DATAPTR_H
	#define DEF_DATAPTR_H

	#include "common_defs.h"
	#include <assert.h>
	#include <vector>
	#include <string>

	#ifdef USE_CUDA
		#include "cuda.h"	
		#define PUSH_CTX		cuCtxPushCurrent(cuCtx);
		#define POP_CTX			CUcontext pctx; cuCtxPopCurrent(&pctx);
	#else
		#define PUSH_CTX
		#define POP_CTX		
	#endif

	#define DT_MISC			0
	#define DT_UCHAR		1		// 8-bit
	#define DT_USHORT		2		// 16-bit
	#define DT_UCHAR3		3		// 24-bit
	
	#define DT_UCHAR4		4		// 32-bit, 4 bytes, 4*sizeof(char)
	#define DT_INT			5		// 32-bit, 4 bytes, 1*sizeof(int32_t)	
	#define DT_UINT			6		// 32-bit, 4 bytes, 1*sizeof(uint32_t)	
	#define DT_FLOAT		7		// 32-bit, 4 bytes, 1*sizeof(float)

	#define DT_UINT64		8		//  64-bit,  8 bytes, 1*sizeof(uint64_t)
	#define DT_FLOAT3		12		//  96-bit, 12 bytes, 3*sizeof(float)
	#define DT_FLOAT4		16	    // 128-bit, 16 bytes, 4*sizeof(float)
	
	#define DT_CPU			1		// use flags
	#define DT_CUMEM		2
	#define DT_CUARRAY		4
	#define DT_GLTEX		8	
	#define DT_GLVBO		16

	class HELPAPI DataPtr {
	public:
		DataPtr() { mNum=0; mMax=0; mStride=0; mUseRX=0; mUseRY=0; mUseRZ=0; mUseType=DT_MISC; mUseFlags=DT_MISC; 
					mSize=0; mCpu=0; 
					#ifdef USE_CUDA
						mGpu=0; mGrsc=0; mGarray=0; mGLID=-1; mGtex = -1; mGsurf = -1; 
					#endif			
					}
		~DataPtr();
		
		// Buffer operations
		void			Resize ( int stride, uint64_t cnt, char* dat=0x0, uchar dest_flags=DT_CPU );
		int				Append ( int stride, uint64_t cnt, char* dat=0x0, uchar dest_flags=DT_CPU );
		void			UseMax ()	{ mNum = mMax; }
		void			SetUsage ( uchar dt, uchar flags=DT_MISC, int rx=-1, int ry=-1, int rz=-1 );		// special usage (2D,3D,GLtex,GLvbo,etc.)
		void			UpdateUsage ( uchar flags );		
		void			ReallocateCPU ( uint64_t oldsz, uint64_t newsz );
		void			FillBuffer ( uchar v );
		void			CopyTo ( DataPtr* dest, uchar dest_flags );
		void			Commit ();		
		void			Retrieve ();		
		void			Clear ();

		// Data access
		int				getStride ( uchar dtype );
		uint64_t		getDataSz ( int cnt, int stride )	{ return (uint64_t) cnt * stride; }
		int				getNum()	{ return mNum; }
		int				getMax()	{ return mMax; }
		char*			getData()	{ return mCpu; }
		#ifdef USE_CUDA
			CUdeviceptr		getGPU()	{ return mGpu; }		
		#endif
		void			SetElem(uint64_t n,  void* dat)	{ memcpy ( mCpu+n*mStride, dat, mStride); }
		char*			getPtr(uint64_t n)		{ return mCpu + n*mStride; }		

		// Helper functions		
		void			SetElemInt(uint64_t n, int val)	{ * (int*) (mCpu+n*mStride) = val; }
		int				getElemInt(uint64_t n)			{ return * (int*) (mCpu+n*mStride); }

		

	public:
		uint64_t		mNum=0, mMax=0, mSize=0;
		int				mStride=0;
		uchar			mRefID=0, mUseType=0, mUseFlags=0;	// usage
		int				mUseRX=0, mUseRY=0, mUseRZ=0;
		bool			bCpu=false, bGpu=false;
		char*			mCpu=NULL;			
		
		int				mGLID=0;				// OpenGL

		#ifdef USE_CUDA
			CUdeviceptr		mGpu;			// CUDA
			CUgraphicsResource	mGrsc;		// CUDA-GL interop
			CUarray			mGarray;	
			CUtexObject		mGtex;			// CUDA texture/surface interop
			CUsurfObject	mGsurf;
		#endif

		static int		mFBO;
	};

#endif




