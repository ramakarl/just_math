//--------------------------------------------------------------------------------
// Copyright 2007-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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

// **** Image includes
#ifndef DEF_IMAGE_INFO
	#define	DEF_IMAGE_INFO

	#include "common_defs.h"				
	#include <stdio.h>
	#ifdef DEBUG_HEAP
		#ifndef _CRTDBG_MAP_ALLOC
			#define _CRTDBG_MAP_ALLOC  
		#endif
		#include <stdlib.h>  
		#include <crtdbg.h> 
	#else
		#include <stdlib.h>  
	#endif

	#define IMGCUBE		0	

	typedef unsigned char	XBYTE;	
	typedef unsigned char	uchar;
	typedef unsigned short	XBYTE2;	
	//typedef unsigned short	XBYTE4;	

	class HELPAPI ImageOp {
	public:
		enum Format {			
			FmtNone = 0,
			BW1 = 1,
			BW8,
			BW16,
			BW32,
			RGB8,
			RGB12,
			RGB16,
			RGB24,
			BGR24,			
			RGBA16,
			RGBA24,
			RGBA32,
			BGR8,
			I420,
			IYUV,
			F8,
			F16,
			Custom
		};
		enum Filter {
			NoFilter = 0,
			Linear = 1,
			MipNoFilter = 2,
			MipLinear = 3
		};
		enum Usage {
			Light = 0,
			Medium = 1,
			Heavy = 2
		};
		enum Flags {
			// Format flags
			Color = 1,			// BW or color?
			Masked = 2,			// 0-value is mask?
			Alpha = 4,			// Alpha channel?
			AlphaMerged = 8,	// Alpha channel is merged?
			Depth = 16,			// Depth channel?
			DepthMerged = 32,	// Depth channel is merged?
			Indexed = 64,		// Indexed pixels?
			Lowclr = 128,			// Low-color pixels (12-bit)?
			Highclr = 256,			// High-color pixels (24-bit)?
			Trueclr = 512,			// True-color pixels (32-bit)?
			// Extended usage flags
			Channels = 1024,	// Other channels present?
			FilterLo = 2048,	// Linear filter?
			FilterHi = 4096,	// Mipmap filter?
			UseLo = 8192,		// Usage method
			UseHi = 16384,
			Cube = 32768		// Cube map?
		};
		enum ImageStatus {
			Ready = 0,
			NeedsUpdate = 1,
			NeedsRebuild = 2,
			NotReady = 3
		};
		enum FormatStatus {
			Idle = 0,
			Loading = 1,
			Saving = 2,
			Successs = 3,			// Yesss - the extra 's' is necesssary because gcc uses the 'Success' keyword.
			DepthNotSupported = 4,
			FeatureNotSupported = 5,
			InvalidFile = 6,
			LibVersion = 7,
			FileNotFound = 8,
			NotImplemented = 9,			
			LoadOk = 10,
			LoadDone = 11, 
			LoadNotReady = 12
		};		
	};

	// Image Info Structure
	struct HELPAPI ImageInfo {
			
			ImageInfo ( int xr, int yr, ImageOp::Format ef );
			ImageInfo ();
			void Reset ();

			ImageInfo& operator= ( const ImageInfo& op ) {				
				eFormat = op.eFormat;
				eFlags = op.eFlags;
				mXres = op.mXres;
				mYres = op.mYres;
				mBitsPerPix = op.mBitsPerPix;
				mBytesPerRow = op.mBytesPerRow;
				return *this;
			}
			
			void SetFormat ( int x, int y, ImageOp::Format ef) {
				mXres = x; mYres = y;
				eFlags = GetFlags ( ef ); 
				eFormat = ef; 
				mBitsPerPix = GetBitsPerPix ( ef );
				mBytesPerRow = GetBytesPerRow ( x, ef );
				
			}
			void SetEqual ( ImageOp::Flags ef, unsigned int src_flags )		{ SetFlag ( eFlags, ef, ( (src_flags & ((unsigned int) ef)) == (unsigned int) ef ) );}
			void SetFilter ( ImageOp::Filter ef );
			void SetUsage ( ImageOp::Usage ef );			
			ImageOp::Filter GetFilter ();
			ImageOp::Usage GetUsage ();			
			bool IsTransparent ()											{ return HasFlag ( eFlags, ImageOp::Alpha); }
			bool IsCube ()													{ return HasFlag ( eFlags, ImageOp::Cube ); }
			bool HasPowerSize ();

			unsigned int GetFlags () { return eFlags; }			// member var
			unsigned int GetFlags (ImageOp::Format ef);			// generic
			void SetFlag ( unsigned int& flags, ImageOp::Flags f, bool val)		{ if (val) flags |= (unsigned int) f; else flags &= ~(unsigned int) f; }
			bool HasFlag ( unsigned int& flags, ImageOp::Flags f )				{ return (flags & (unsigned int) f)==0 ? false : true; }
			bool NoFilter()	{ return !HasFlag ( eFlags, ImageOp::FilterLo ); }
			bool MipFilter() { return HasFlag ( eFlags, ImageOp::FilterHi ); }
			
			//void SetBitsPerPixel (int x);			
			inline unsigned char GetBitsPerPix () { return mBitsPerPix; }
			unsigned char GetBitsPerPix(ImageOp::Format ef);

			inline int GetBytesPerPix ()	{ return mBitsPerPix >> 3; }
			inline int GetBytesPerPix (ImageOp::Format ef )		{ return GetBitsPerPix(ef) >> 3; }

			unsigned char GetDataType (ImageOp::Format ef);

			inline unsigned long GetBytesPerRow ()	{ return mBytesPerRow; }
			inline unsigned long GetBytesPerRow (int x, ImageOp::Format ef )	{ return GetBitsPerPix(ef)*x >> 3; }

			inline unsigned long GetSize ()	{ return (unsigned long) mBitsPerPix * (unsigned long) mXres * mYres >> 3; }
			inline unsigned long GetSize (int x, int y, ImageOp::Format ef )	{ return GetBitsPerPix(ef) * (unsigned long) x * (unsigned long) y >> 3; }

			// Query functions
			int QuerySmallestPower (int x);
			void QueryPowerSize (int &x, int &y);
			bool QueryClip ( int& sx1, int& sy1, int& sx2, int& sy2, int cx1, int cy1, int cx2, int cy2 );
			void QueryRect ( ImageOp::Format f, int src_x, XBYTE* src, int sx1, int sy1, int sx2, int sy2, XBYTE*& src_start, XBYTE*& src_end, int& src_wid, int& src_pitch );
			bool QueryPaste ( ImageOp::Format src_f, int src_x, int src_y, XBYTE* src_dat, int sx1, int sy1, int sx2, int sy2,
							  ImageOp::Format dest_f, int dest_x, int dest_y, XBYTE* dest_dat, int dx1, int dy1, 
							  XBYTE*& src_start, XBYTE*& src_end, int& src_wid, int& src_pitch, 
							  XBYTE*& dest_start, XBYTE*& dest_end, int& dest_wid, int& dest_pitch);					

			// Data
			ImageOp::Format		eFormat;				// Image Format
			unsigned int		eFlags;					// Image Flags
			unsigned char		mBitsPerPix;			// BPP = Bits-per-Pixel
			unsigned long		mBytesPerRow;			// Bytes per Row = BPP*Xres >> 3
														// Bytes per Pixel = BPP >> 3
														// Size = mBPP*Xres*Yres >> 3
			int					mXres, mYres;			// Image Resolution
	};	

	// Image Format Params
	struct ImageFormatParams {
							// JPEG
		int		iQuality;	// 0 to 100
		float	fScaling;	// 1/2,1/4,1/8,1

	};

#endif

