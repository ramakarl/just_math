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

#ifndef DEF_IMAGEFORMAT
	#define	DEF_IMAGEFORMAT

	#include "image.h"
	#include "image_info.h"
	#include <stdio.h>

#ifdef WIN32
	#include <windows.h>
	#include <conio.h>
#endif

	#define FILE_NAMELEN	512

	class HELPAPI CImageFormat {
	public:
		CImageFormat ();
		~CImageFormat ();

		virtual bool Save (char *filename, Image* pImg) {return false;}		
		virtual bool Load (char *filename, Image* pImg) {return false;}		
		virtual bool LoadIncremental ( const char* filename, Image* pImg) { return false; }
		virtual ImageOp::FormatStatus LoadNextRow () { return ImageOp::LoadNotReady; }		

#if defined(BUILD_VC)
		bool TransferBitmap ( HBITMAP hBmp );
#endif
		// Helper functions		
		int GetWidth ( Image* img )				{ return img->GetWidth();  }
		int GetHeight ( Image* img )				{ return img->GetHeight(); }
		ImageOp::Format GetFormat ( Image* img )	{ return img->GetFormat(); }
		int GetBitsPerPixel ( Image* img )		{ return img->GetBitsPerPixel ();}
		int GetBytesPerPixel ( Image* img )		{ return img->GetBytesPerPixel ();}
		int GetBytesPerRow ( Image* img )			{ return img->GetBytesPerRow ();}
		uchar* GetData ( Image* img )				{ return img->GetData  (); }
		void CreateImage ( Image*& img, int xr, int yr, ImageOp::Format fm );

		ImageOp::FormatStatus GetStatus ()	{ return m_eStatus; }
		void GetStatusMsg (char* buf);

		void StartLoad ( char* filename, Image* pImg );		
		void FinishLoad ();

	protected:
		Image*				m_pOrigImage;			// Original image. (ImageFormat does not own it)
		Image*				m_pNewImage;
		char				m_Filename[FILE_NAMELEN];
		ImageOp::FormatStatus 	m_eStatus;		

		// General format data
		int					m_Xres, m_Yres;		
		int					m_BytesPerRow;
		int					m_BitsPerPixel;
	};


#endif


