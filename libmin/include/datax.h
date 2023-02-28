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
#ifndef DEF_DATAX
	#define DEF_DATAX

	#include "common_defs.h"

	#define	HEAP_MAX			2147483640	// largest heap size (range of hpos)
	#define	ELEM_MAX			2147483640	// largest number of elements in a buffer (range of hval)
	#define HEAP_INIT			8
	#define HEAP_POS			2
	#define REF_MAX				64
	#define BUNDEF				65535

	typedef signed int			hpos;		// pointers into heap (sint = 32-bit)
	typedef signed int			hval;		// values in heap (sint = 32-bit)	
	typedef hval				href;		// values are typically references 
	typedef unsigned short		ushort;
	typedef unsigned char		uchar;
	typedef signed short		bufPos;
	
	// Heap types
	typedef signed int			hpos;		// pointers into heap (sint = 32-bit)
	typedef signed int			hval;		// values in heap (sint = 32-bit)	
	typedef hval				href;		// values are typically references 	
	typedef signed short		bufPos;
	
	#ifdef CUDA_KERNEL
		typedef char*			devdata_t;
	#else
		#ifdef USE_CUDA
			#include <cuda.h>
			typedef CUdeviceptr		devdata_t;
		#else	
			typedef void*			devdata_t;
		#endif
	#endif

	// DataX buffers (GPU)
	#define	DMAXBUF		16				// fixed for now
	struct HELPAPI cuDataX {
		devdata_t	mbuf[DMAXBUF];				// See DataX::UpdateGPUAccess

		#ifdef CUDA_KERNEL
			inline __device__ float*  bufF(int n)	{ return (float*) mbuf[n]; }	
			inline __device__ float3* bufF3(int n)	{ return (float3*) mbuf[n]; }
			inline __device__ int*	  bufI(int n)	{ return (int*) mbuf[n]; }
			inline __device__ uint*	  bufUI(int n)	{ return (uint*) mbuf[n]; }
			inline __device__ char*	  bufC(int n)	{ return (char*) mbuf[n]; }
		#endif
	};		

	// DataX buffers (CPU)
	//
	#ifndef CUDA_KERNEL				// not on gpu..
		
		#include <assert.h>
		#include <vector>
		#include <string>
		#include "vec.h"	
		#include "dataptr.h"

		class HELPAPI hList {				// heap data
		public:
			ushort		cnt;
			ushort		max;
			hpos		pos;
		};

		class HELPAPI DataX {
		public:				
			DataX ();
			~DataX();

			void		Print ();
		
			// Buffer Operations
			bool		isActive ( int buf )		{ return (mRef[buf]==BUNDEF) ? false : true; }
			int			AddBuffer		( int ref, std::string name, ushort stride, uint64_t maxcnt, uchar dest_flags=DT_CPU );		// add buffer
			void		SetBufferUsage	( int i, uchar dt, uchar flags=DT_MISC, int rx=-1, int ry=-1, int rz=-1 );
			char*		ExpandBuffer	( int i, int max_cnt);						
			void		ResizeBuffer	( int i, int max_cnt, bool safe=false );			// safely resize a buffer. does not change number of elements.
			
			void		DeleteBuffer	( int i );			// delete the buffer 
			void		ClearBuffer		( int i );			// clear memory (free) but keep buffer
			void		EmptyBuffer		( int i )			{ int b = mRef[i]; if (b != BUNDEF) mBuf[b].mNum = 0; }		// clear active elements
			void		ReserveBuffer	( int i, int cnt )	{ int b = mRef[i]; if (b != BUNDEF) mBuf[b].mNum = cnt;}	// set number of active elements 
			
			void		SetBuffer		( int i, int len, char* dat );			
			int			FindBuffer		( std::string name );								// Find buffer by name. Good for finding shapes, may be multiple prim/attr with same name.		
			int			FindBuffer		( int b, std::string name );						// Find sub-buffer (within a given shape)	
			void		CopyBuffer		( uchar bsrc, uchar bdest, DataX* dest, uchar dest_flags=DT_CPU );	// deep copy one buffer to another					
			void		FillBuffer		( int userid, uchar v );
			
			// All Buffer Operations
			void		SetNum			( int num );										// set same number of used elements on all buffers
			void		EmptyAllBuffers	();
			void		DeleteAllBuffers ();												// erase all buffers (delete from CPU & GPU)
			void		CopyAllBuffers ( DataX* dest, uchar dest_flags=DT_CPU );			// copy all buffers to another DataX. buffer listings must match
			void		MatchAllBuffers ( DataX* src, uchar use_flags=DT_MISC );			// match all buffers from another DataX

			// GPU Operations
			#ifdef USE_CUDA
			void		AssignToGPU ( std::string var_name, CUmodule& module );				// assign DataX to a GPU symbolic variable
			#endif
			void		UpdateGPUAccess ();													// update GPU symbol to hold updated pointers
			void		Retrieve ( int i );													// retrieve buffer from GPU to CPU
			void		RetrieveAll ();
			void		Commit ( int i );													// commit buffer from CPU to GPU			
			void		CommitAll ();														// commit all buffers to GPU

			// Buffer accessors
			char*		printElem ( int i, int n, char* buf );
			char*		RandomElem ( uchar b, href& ndx );				
			bool		DelElem ( uchar b, int n );	
			int			AddElem ();						// add element (to all buffers)
			int			AddElem ( int b );				// add element (to specific buffer)
			char*		AddElem ( int b, int cnt );		// add specified number of elements (return start)		
			int			GetNumBuf ()					{ return (int) mBuf.size(); }	
			bool		isValidBuf(int i)				{ int b = mRef[i]; return (b == BUNDEF) ? false : true; }   // call before Get.. funcs
			char*		GetStart ( int i )				{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mCpu; }
			char*		GetEnd ( int i )				{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mCpu + (mBuf[b].mNum-1)*mBuf[b].mStride; }		
			int			GetNumElem ( int i )			{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mNum; }
			char*		GetBufData ( int i  )			{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mCpu; }		
			DataPtr*	GetBuffer ( int i )				{ int b=mRef[i]; return (b==BUNDEF) ? 0 : &mBuf[b]; }		
			int			GetMaxElem ( int i )			{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mMax; }
			int			GetBufStride ( int i )			{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mStride; }
			int			GetBufSize ( int i )			{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mNum*mBuf[b].mStride; }		 			
			char*		GetElem(int i, int ndx)			{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mCpu + ndx * mBuf[b].mStride; }
			char*		GetElemFast(char* bufdata, int stride, int ndx)		{ return bufdata + ndx * stride; }

			// Element access
			void		SetElem	     ( int i, int n, void* val )		{ int b=mRef[i]; if (b==BUNDEF) return; memcpy ( mBuf[b].mCpu + n*mBuf[b].mStride, val, mBuf[b].mStride); }
			void		SetElemFloat ( int i, int n, float val )		{ int b=mRef[i]; if (b==BUNDEF) return; * ((float*) mBuf[b].mCpu+n) = val; }
			void		SetElemInt   ( int i, int n, int val )			{ int b=mRef[i]; if (b==BUNDEF) return; * ((int*) mBuf[b].mCpu+n) = val; }
			void		SetElemUInt  ( int i, int n, int val )			{ int b=mRef[i]; if (b==BUNDEF) return; * ((uint*) mBuf[b].mCpu+n) = val; }
			void		SetElemChar  ( int i, int n, uchar val )		{ int b=mRef[i]; if (b==BUNDEF) return; * ((uchar*) mBuf[b].mCpu+n) = val; }
			void		SetElemXLong ( int i, int n, xlong val )		{ int b=mRef[i]; if (b==BUNDEF) return; * ((xlong*) mBuf[b].mCpu+n) = val; }
			void		SetElemClr   ( int i, int n, CLRVAL val )		{ int b=mRef[i]; if (b==BUNDEF) return; * ((CLRVAL*) mBuf[b].mCpu+n) = val; }
			void		SetElemVec2  ( int i, int n, Vector2DF& val )	{ int b=mRef[i]; if (b==BUNDEF) return; * ((Vector2DF*) mBuf[b].mCpu+n) = val; }
			void		SetElemVec3  ( int i, int n, Vector3DF& val )	{ int b=mRef[i]; if (b==BUNDEF) return; * ((Vector3DF*) mBuf[b].mCpu+n) = val; }
			void		SetElemVec4  ( int i, int n, Vector4DF& val )	{ int b=mRef[i]; if (b==BUNDEF) return; * ((Vector4DF*) mBuf[b].mCpu+n) = val; }
			void		SetElemM4    ( int i, int n, Matrix4F& val )	{ int b=mRef[i]; if (b==BUNDEF) return; * ((Matrix4F*) mBuf[b].mCpu+n) = val; }				
			void		SetElemStr   ( int i, int n, std::string val );
		
			// old API interface
			float		GetElemFloat ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0; return * ((float*) mBuf[b].mCpu+n); }
			int			GetElemInt   ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0; return * ((int*) mBuf[b].mCpu+n); }
			xlong		GetElemXLong ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0; return * ((xlong*) mBuf[b].mCpu+n); }
			uint		GetElemUInt   ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0; return * ((uint*) mBuf[b].mCpu+n); }
			ushort		GetElemUShort ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0;  return * ((ushort*) mBuf[b].mCpu+n); }
			uchar		GetElemChar   ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0; return * ((uchar*) mBuf[b].mCpu+n); }
			CLRVAL*		GetElemClr   ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0;  return ((CLRVAL*) mBuf[b].mCpu+n); }
			Vector2DF*	GetElemVec2  ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0; return ((Vector2DF*) mBuf[b].mCpu+n); }
			Vector3DF*	GetElemVec3  ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0;  return ((Vector3DF*) mBuf[b].mCpu+n); }
			Vector4DF*	GetElemVec4  ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0;  return ((Vector4DF*) mBuf[b].mCpu+n); }
			Matrix4F*	GetElemM4    ( int i, int n )					{ int b=mRef[i]; if (b==BUNDEF) return 0;  return ((Matrix4F*) mBuf[b].mCpu+n); }
			std::string GetElemStr   ( int i, int n );

			// new API interface
			char*			bufC(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((char*) mBuf[b].mCpu+n); }
			int*			bufI(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((int*) mBuf[b].mCpu+n); }
			short*			bufS(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((short*) mBuf[b].mCpu+n); }
			uint*			bufUI(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((uint*) mBuf[b].mCpu+n); }
			ushort*			bufUS(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((ushort*) mBuf[b].mCpu+n); }
			float*			bufF(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((float*) mBuf[b].mCpu+n); }
			Vector3DF*		bufF3(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((Vector3DF*) mBuf[b].mCpu+n); }
			Vector4DF*		bufF4(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((Vector4DF*) mBuf[b].mCpu+n); }
			Matrix4F*		bufM4(int i, int n=0)	{ int b=mRef[i];	return (b==BUNDEF) ? 0 : ((Matrix4F*) mBuf[b].mCpu+n); }

			bool			hasBuf( int i )			{ int b=mRef[i]; return (b!=BUNDEF); }
			int				getBufSz ( int i )		{ int b=mRef[i]; return (b==BUNDEF) ? 0 : mBuf[b].mSize; }		// max size (not num)			

			char*			cpu(int i)		{int b=mRef[i];	return (b==BUNDEF) ? 0 : mBuf[b].mCpu; }
			int				glid(int i)		{int b=mRef[i];	return (b==BUNDEF) ? 0 : mBuf[b].mGLID; }
			int*			glidptr(int i)	{int b=mRef[i];	return (b==BUNDEF) ? 0 : &mBuf[b].mGLID; }
			
			#ifdef USE_CUDA
				CUdeviceptr		gpu(int i)		{int b=mRef[i];	return (b==BUNDEF) ? 0 : mBuf[b].mGpu; }
				CUdeviceptr*	gpuptr(int i)	{int b=mRef[i];	return (b==BUNDEF) ? 0 : &mBuf[b].mGpu; }			
				CUgraphicsResource* grsc(int i) {int b=mRef[i];	return (b==BUNDEF) ? 0 : &mBuf[b].mGrsc; }
				cuDataX			getGPUData()	{ return cuDataCPU; }
			#endif	

	#ifdef USE_BASEOBJ
			void		SetElemObj   ( int b, int n, BaseObject* val )	{ if (b==BUF_UNDEF) return; * ((BaseObject**) mBuf[b].data+n) = val; }
			BaseObject*	GetElemObj   ( int b, int n )					{ if (b==BUF_UNDEF) return 0x0; return * ((BaseObject**) mBuf[b].data+n); }
	#endif
		
			//----- Heap functions
			void			ClearHeap ();
			void			ClearRefs ( hList* list );
			void			AddHeap ( int max );
			void			CopyHeap ( DataX& src );
			void			ResetHeap ();
			hval			AddRef ( hval r, hList* list, hval delta  );
			hpos			HeapAlloc ( ushort size, ushort& ret );
			hpos			HeapExpand ( ushort size, ushort& ret  );
			void			HeapAddFree ( hpos pos, int size );

			// Heap queries		
			hval*			GetHeap ( hpos& num, hpos& max, hpos& free );		
			hval*			GetHeap ()	{ return mHeap; }
			int				GetHeapSize ();
			int				GetHeapNum ()	{ return mHeapNum; }
			int				GetHeapMax ()	{ return mHeapMax; }
			int				GetHeapFree ()	{ return mHeapFree; }

		private:										// internal functions
			int		AddElemDirect(int b);				// low-level add. no user-level indirection

		public:
		
			std::vector< DataPtr >	mBuf;				// list of buffers
			int						mRef[REF_MAX];

			#ifdef USE_CUDA
			std::string				cuDataName;			// name of datax device variable
			CUdeviceptr				cuDataGPU;			// data on gpu (all buffers, see above)
			cuDataX					cuDataCPU;			// data on cpu
			#endif

			hpos					mHeapNum;			// data buffer heap
			hpos					mHeapMax;
			hpos					mHeapFree;
			hval*					mHeap;		
		};

	#endif

#endif




