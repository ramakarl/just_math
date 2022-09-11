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
#ifndef DEF_IMAGE
	#define DEF_IMAGE

	#include <string>	
	#include "common_defs.h"				
	#include "image_info.h"
	#include "vec.h"
	#include "dataptr.h"

	class CImageFormat;

	class HELPAPI Image {
	public:
		Image ();
		Image ( int xr, int yr, ImageOp::Format fmt, uchar use_flags=DT_CPU );
		Image ( std::string name, int xr, int yr, ImageOp::Format fmt );
		~Image ();
		void Clear ();
		ImageInfo* getInfo() { return &m_Info; }

		virtual bool Load(std::string fname) {
			std::string errmsg;
			return Load(fname, errmsg);			
		}

		// Image Loading & Saving
		bool Load(char* filename, char* alphaname);
		bool Load(std::string filename, std::string& errmsg);
		bool LoadAlpha(char* filename);
		bool LoadIncremental(char* filename);
		ImageOp::FormatStatus LoadNextRow();
		bool Save(char* filename);								// Save Image

		// Image Creation, Resizing & Reformatting				
		void Create ();
		void Create ( int xr, int yr, ImageOp::Format eFormat );		// Create image		
		void ResizeImage ( int xr, int yr, ImageOp::Format eFormat );		
		void ResizeChannel ( int chan, int xr, int yr, ImageOp::Format eFormat );		
		void AddChannel ( std::string name, int xr, int yr, ImageOp::Format eFormat );		
		void ChangeFormat (ImageOp::Format eFormat) 	{ (this->*m_ReformatFunc) ( eFormat); } // Change pixel format of data
		void DeleteBuffers ();
		void CopyIntoBuffer ( DataPtr& dest, DataPtr& src, int bpp, int w, int h );

		// GPU
		void SetUsage ( uchar use_flags );
		void Commit (uchar use_flags=0);
		void CommitAll ();
		int getGLID()			{ return m_Pix.mGLID; }
		#ifdef USE_CUDA
			CUdeviceptr getGPU()	{ return m_Pix.mGpu;  }		
			CUarray getArray( CUtexObject& in, CUsurfObject& out )	{ in = m_Pix.mGtex; out = m_Pix.mGsurf; return m_Pix.mGarray; }
		#endif

		// Channels		
		//int FindChannel ( std::string name )			{ return FindBuffer ( name ) - 1; }		// Channels are returned as index to param info 
		//bool LoadChannel ( std::string name, std::string fname );
		//int  GetNumChannels ()							{ return GetNumBuf()-1; }
		
		// Image Copying
        Image* CopyNew ();						// Create new ImageX by direct copy of this one.
		Image* CopyNew ( ImageOp::Format new_format );  // Create new ImageX by direct copy of this one, changing format
		Image* CopyNewPower ( bool resize);	// Create new ImageX by copy of this one as a power image.
		void Copy ( Image* new_img );			// Copy into given ImageX, overwriting format with this one
		void Copy ( Image* new_img, ImageOp::Format new_format );	// Copy into given ImageX, overwriting format with specified one
		void CopyAlpha ( Image* new_alpha );		

		// Image Operations
		inline Vector4DF GetPixelUV (float x, float y)	{ XBYTE r,g,b,a; (this->*m_GetPixelFunc) ( (int) (x*(getInfo()->mXres-1)), (int) (y*(getInfo()->mYres-1)),r,g,b,a); return Vector4DF(r/255.0f,g/255.0f,b/255.0f,a/255.0f); }
		inline float GetPixelUV16 ( float u, float v )							{ XBYTE r,g,b,a;(this->*m_GetPixelFunc) ( int(u)*(getInfo()->mXres-1), int(v)*(getInfo()->mYres-1), r,g,b,a); return (float) (r+(g/255.0f))/255.0f; }
		inline float GetPixelValUV ( float u, float v )							{ XBYTE r,g,b,a;(this->*m_GetPixelFunc) ( int(u)*(getInfo()->mXres-1), int(v)*(getInfo()->mYres-1), r,g,b,a); return (float) r/255.0f; }
		int  GetPixel (int x, int y )											{ XBYTE r,g,b,a;(this->*m_GetPixelFunc) (x,y,r,g,b,a); return (int) r; }
		void GetPixel (int x, int y, XBYTE& i)									{ XBYTE g,b,a;	(this->*m_GetPixelFunc) (x, y, i, g, b, a); }
		void GetPixel (int x, int y, XBYTE& r, XBYTE& g, XBYTE& b)				{ XBYTE a;		(this->*m_GetPixelFunc) (x, y, r, g, b, a); }
		void GetPixel (int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a)	{				(this->*m_GetPixelFunc) (x, y, r, g, b, a); }
		void SetPixel (int x, int y, XBYTE i)									{ (this->*m_SetPixelFunc) (x, y, i, i, i, 255); }
		void SetPixel (int x, int y, XBYTE r, XBYTE g, XBYTE b)					{ (this->*m_SetPixelFunc) (x, y, r, g, b, 255); }
		void SetPixel (int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a)		{ (this->*m_SetPixelFunc) (x, y, r, g, b, a); }
		void Fill (XBYTE i)														{ (this->*m_FillFunc) (i, i, i, 255); }
		void Fill (XBYTE r, XBYTE g, XBYTE b)									{ (this->*m_FillFunc) (r, g, b, 255); }
		void Fill (XBYTE r, XBYTE g, XBYTE b, XBYTE a)							{ (this->*m_FillFunc) (r, g, b, a); }		
		void Scale ( int nx, int ny );

		// General Information (Not specific to this image)
		unsigned long GetBytesPerRow (int dx, int dy, ImageOp::Format f ) { return getInfo()->GetBytesPerRow ( dx, f ); }
		int GetSize (int dx, int dy, ImageOp::Format f )		{ return getInfo()->GetSize ( dx, dy, f ); }

		// Image Information (for current layer ! )
		void SetFilter ( ImageOp::Filter ef )		{ SetFilter ( 0, ef ); }
		void SetFilter ( int chan, ImageOp::Filter ef );
		void SetFilter ( std::string chan, ImageOp::Filter ef );
		void SetInfo ( ImageInfo* info, int xr, int yr, ImageOp::Format fmt );
		int GetWidth ()							{ return getInfo()->mXres; }
		int GetHeight ()						{ return getInfo()->mYres; }
		XBYTE* GetData ()						{ return (XBYTE*) m_Pix.mCpu; }		// fast access (for pixel functions)
		unsigned long GetSize ()				{ return getInfo()->GetSize (); }
		unsigned char GetBitsPerPixel ()		{ return getInfo()->mBitsPerPix;}		
		unsigned long  GetBytesPerRow ()		{ return getInfo()->GetBytesPerRow();}		
		unsigned char GetDataType (ImageOp::Format ef)	{ return getInfo()->GetDataType(ef); }
		int GetBytesPerPixel ()					{ return getInfo()->GetBytesPerPix();}
		bool HasFlag (ImageOp::Flags eFlag)		{ return getInfo()->HasFlag (getInfo()->eFlags, eFlag); }
		bool IsTransparent ()					{ return getInfo()->IsTransparent(); }		
		ImageOp::Format GetFormat ()			{ return getInfo()->eFormat; }
		int GetFlags ()							{ return getInfo()->eFlags; }
		ImageOp::Usage GetUsage ();
		ImageOp::Filter GetFilter ();
		bool NoFilter()	{ return !getInfo()->HasFlag ( getInfo()->eFlags, ImageOp::FilterLo ); }
		bool MipFilter() { return getInfo()->HasFlag ( getInfo()->eFlags, ImageOp::FilterHi ); }


		// Pivot
		/*void SetPivot (float x, float y)				{ getInfo().SetPivot ( x, y) ; }
		void SetPivotCenter ()	{ getInfo().SetPivot ( float(getInfo().mXres/2.0), float(getInfo().mYres/2.0) ); }
		float GetPivotX ()								{ return getInfo().Xpiv; }
		float GetPivotY ()								{ return getInfo().Ypiv; }
		//void GetRange (unsigned int& a, unsigned int& b )	{ a = mMin; b = mMax; }*/

		// Paste - Copy sub-region of src into dest. No rescaling.
		void Paste ( int src_x1, int src_y1, int src_x2, int src_y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty )
			{ (this->*m_PasteFunc) ( src_x1, src_y1, src_x2, src_y2, offx, offy, dest, dest_format, destx, desty ); }

		// Essential Helper Functions		
		void TransferFrom ( Image* new_img);				// Transfer data ownership from another image
		void TransferData ( char* buf );					// Copy data from external source. Size & format must already be set.
		void SetFormatFunc ( int chan );
		void SetFormatFunc ( ImageOp::Format eFormat );

	private:
		// Internal functions
		void SetFlag (ImageOp::Flags ef, bool val)					{ getInfo()->SetFlag (getInfo()->eFlags, ef, val ); }
		void SetFormat (int xr, int yr, ImageOp::Format eFormat)	{ getInfo()->SetFormat (xr,yr, eFormat); }	
		void SetEqual (ImageOp::Flags ef, unsigned int f)			{ getInfo()->SetEqual (ef, f); }

		// Performance Function pointers for this image
		void (Image::*m_GetPixelFunc) (int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a);
		void (Image::*m_SetPixelFunc) (int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a);	
		void (Image::*m_FillFunc) (XBYTE r, XBYTE g, XBYTE b, XBYTE a);			
		void (Image::*m_PasteFunc) ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void (Image::*m_ScaleFunc) ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void (Image::*m_RemapFunc) ( unsigned int vmin, unsigned int vmax );
		void (Image::*m_ReformatFunc) (ImageOp::Format eFormat);
		void (Image::*m_AlphaFunc) ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		// BW8
		void GetPixelBW8 ( int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a );
		void SetPixelBW8 ( int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void FillBW8 ( XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void ScaleBW8 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void PasteBW8 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void RemapBW8 ( unsigned int vmin, unsigned int vmax );
		void ReformatBW8 ( ImageOp::Format eFormat );		
		void AlphaBW8 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		// BW16
		void GetPixelBW16 ( int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a );
		void SetPixelBW16 ( int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void FillBW16 ( XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void ScaleBW16 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void PasteBW16 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void RemapBW16 ( unsigned int vmin, unsigned int vmax );
		void ReformatBW16 ( ImageOp::Format eFormat );		
		void AlphaBW16 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		// BW32
		void GetPixelBW32 ( int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a );
		void SetPixelBW32 ( int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void FillBW32 ( XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void ScaleBW32 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void PasteBW32 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void RemapBW32 ( unsigned int vmin, unsigned int vmax );
		void ReformatBW32 ( ImageOp::Format eFormat );		
		void AlphaBW32 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		// RGB24
		void GetPixelRGB24 ( int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a );
		void SetPixelRGB24 ( int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void FillRGB24 ( XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void ScaleRGB24 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void PasteRGB24 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void RemapRGB24 ( unsigned int vmin, unsigned int  vmax );
		void ReformatRGB24 ( ImageOp::Format eFormat );		
		void AlphaRGB24 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		// BGR24
		void GetPixelBGR24 ( int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a );
		void SetPixelBGR24 ( int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void FillBGR24 ( XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void ScaleBGR24 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void PasteBGR24 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void RemapBGR24 ( unsigned int vmin, unsigned int vmax );
		void ReformatBGR24 ( ImageOp::Format eFormat );		
		void AlphaBGR24 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		// RGBA32
		void GetPixelRGBA32 ( int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a );
		void SetPixelRGBA32 ( int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void FillRGBA32 ( XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void ScaleRGBA32 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void PasteRGBA32 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void RemapRGBA32 ( unsigned int vmin, unsigned int vmax );
		void ReformatRGBA32 ( ImageOp::Format eFormat );		
		void AlphaRGBA32 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		// F8
		void GetPixelF8 ( int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a );
		void SetPixelF8 ( int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void FillF8 ( XBYTE r, XBYTE g, XBYTE b, XBYTE a );
		void ScaleF8 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void PasteF8 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty );
		void RemapF8 ( unsigned int vmin, unsigned int vmax );
		void ReformatF8 ( ImageOp::Format eFormat );		
		void AlphaF8 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y );

		ImageInfo			m_Info;
		DataPtr				m_Pix;
		DataPtr				m_Depth;
		DataPtr				m_Alpha;		

		uchar				m_UseFlags;

		static CImageFormat*		mpImageLoader;		// For incremental image loading
		static XBYTE fillbuf[];
	};

#endif


