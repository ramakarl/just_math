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

#include <math.h>
#include <assert.h>

#include "image.h"
#include "imageformat.h"
#include "imageformat_bmp.h"
#include "imageformat_jpg.h"
#include "imageformat_png.h"
#include "imageformat_tiff.h"
#include "imageformat_tga.h"

CImageFormat* Image::mpImageLoader = 0x0;
XBYTE Image::fillbuf[16384];

#define BUF_INFO		0
#define BUF_PIX			1
#define BUF_ALPHA		2
#define BUF_DEPTH		3

//-----------------------------------------------------
//  Image Geometry Control
//-----------------------------------------------------

Image::Image ()
{
	m_UseFlags = DT_CPU;
	m_Pix.Clear();
	m_Alpha.Clear();
	m_Depth.Clear();	
	ResizeImage ( 0, 0, ImageOp::RGB8 );
}
Image::Image ( int xr, int yr, ImageOp::Format fmt, uchar use_flags )
{
	m_UseFlags = use_flags;
	m_Pix.Clear();
	m_Alpha.Clear();
	m_Depth.Clear();
	ResizeImage ( xr, yr, fmt );
}

void Image::Create ( )
{
	return Create ( 0, 0, ImageOp::RGB8 );
}

void Image::Create ( int xr, int yr, ImageOp::Format eFormat)
{	
	ResizeImage ( xr, yr, eFormat );
}
Image::~Image (void)
{
	DeleteBuffers ();
}
void Image::Clear ()
{
	DeleteBuffers ();
}

void Image::DeleteBuffers ()
{
	m_Pix.Clear();
	m_Depth.Clear();
	m_Alpha.Clear();
}

void Image::SetUsage ( uchar use_flags )
{
	m_UseFlags = use_flags;
	m_Pix.UpdateUsage ( m_UseFlags );
}

void Image::Retrieve()
{		
	m_Pix.Retrieve();
}
void Image::Commit ( uchar use_flags )
{	
	if ( use_flags != 0 ) {
		m_UseFlags = use_flags;		
		m_Pix.UpdateUsage(m_UseFlags);
	}
	m_Pix.Commit();				// assumes allocated on gpu
}

void Image::CommitAll ()
{
	m_Pix.Commit();
	m_Depth.Commit();
	m_Alpha.Commit();
}

void Image::SetInfo ( ImageInfo* info, int xr, int yr, ImageOp::Format fmt )
{	
	info->SetFormat ( xr, yr, fmt );
}
void Image::ResizeImage ( int xr, int yr )
{
	ResizeImage ( xr, yr, getInfo()->eFormat );
}

void Image::ResizeImage ( int xr, int yr, ImageOp::Format eFormat)
{
	ResizeChannel ( 0, xr, yr, eFormat );
}

void Image::ResizeChannel ( int chan, int xr, int yr, ImageOp::Format eFormat)
{
	ImageInfo* info = getInfo();
	
	if ( xr != info->mXres || yr != info->mYres || eFormat != info->eFormat ) {

		// Set new pixel format parameters		
		uchar dt = info->GetDataType ( eFormat );
		info->SetFormat ( xr, yr, eFormat );		
		m_Pix.SetUsage ( dt, m_UseFlags, xr, yr,1 );
		m_Pix.Resize ( GetBytesPerPixel(), xr*yr, 0x0, m_UseFlags );		
		m_Pix.mNum = xr*yr;
				
		// Create extended buffers
		if (HasFlag(ImageOp::Alpha) && !HasFlag(ImageOp::AlphaMerged)) {	// Check if we need seperate alpha data..					
			m_Alpha.Resize ( sizeof(XBYTE), xr*yr );
			m_Alpha.SetUsage ( DT_UCHAR, DT_CPU, xr,yr,1 );		// 8-bit alpha
		}
		
		if (HasFlag(ImageOp::Depth) && !HasFlag(ImageOp::DepthMerged)) { // Check if we need seperate depth data..
			m_Depth.Resize ( sizeof(XBYTE), xr*yr );
			m_Depth.SetUsage ( DT_UCHAR, DT_CPU, xr,yr,1 );		// 8-bit depth
		}

		// Update formatting functions
		SetFormatFunc ( 0 );
	}
}
/*
void Image::AddChannel ( std::string name, int xr, int yr, ImageOp::Format eFormat )
{
	int chan = GetNumBuf()-1;

	char infoname[256];
	sprintf ( infoname, "i%02d", chan );	
	ImageInfo info ( xr, yr, eFormat );
	AddParam ( p, infoname, DT_DATA, 0x0 );
	SetParamData ( p, (uchar*) &info, sizeof(ImageInfo) );

	// Add new channel data
	bufPos b = AddBuffer ( 'E', p, name, info.GetBytesPerPix(eFormat), xr*yr );
	ReserveBuffer ( b, xr*yr );
}*/

/*bool Image::LoadChannel ( std::string name, std::string filename )
{
	ImageX		ctrl;
	ImageX		tmp_img ( 0, 0, ImageOp::RGBA8 );
	int			chan;

	ctrl.clear();
	ctrl.add_data ( BUF_UNDEF );

	// Load image file
	if ( ctrl.Load ( filename) ) {

		// Determine channel destination buffer
		chan = FindChannel ( name );
		if ( chan == BUF_UNDEF-1)	{
			AddChannel ( 0, name, tmp_img.GetWidth(), tmp_img.GetHeight(), tmp_img.GetFormat() );
			chan = FindChannel ( name );
		}
		ImageInfo* info = getInfo (chan);
		XBYTE* pixbuf = getData (chan);

		// Resize channel buffer
		ResizeChannel ( chan, tmp_img.GetWidth(), tmp_img.GetHeight(), tmp_img.GetFormat() );

		// Copy pixel data from temp image to channel
		ctrl.Paste ( 0, 0, tmp_img.GetWidth(), tmp_img.GetHeight(), 0, 0, pixbuf, tmp_img.GetFormat(), tmp_img.GetWidth(), tmp_img.GetHeight() );

		printf ( 'imgf', INFO, "Loaded channel: %s (%d), %dx%d, bpp:%d\n", name.c_str(), chan, tmp_img.GetWidth(), tmp_img.GetHeight(), tmp_img.GetBitsPerPixel() );

	}		
	return true;
}
*/

Vector4DF Image::GetPixelFilteredUV (float x, float y)
{
	float u = x * (getInfo()->mXres - 1);
	float v = y * (getInfo()->mYres - 1);
	int xu = u;
	int yu = v;
	u -= xu;
	v -= yu;
	
	XBYTE r,g,b,a;
	Vector4DF c[7];

	(this->*m_GetPixelFunc) ( xu,    yu, r,g,b,a ); c[0] = Vector4DF(r,g,b,a);
	(this->*m_GetPixelFunc) ( xu+1,  yu, r,g,b,a ); c[1] = Vector4DF(r,g,b,a);
	(this->*m_GetPixelFunc) ( xu,  yu+1, r,g,b,a ); c[2] = Vector4DF(r,g,b,a);
	(this->*m_GetPixelFunc) ( xu+1,yu+1, r,g,b,a ); c[3] = Vector4DF(r,g,b,a);
	
	// bi-linear filtering
	c[4] = c[0] + (c[1]-c[0]) * u;
	c[5] = c[2] + (c[3]-c[2]) * u;
	c[6] = c[4] + (c[5]-c[4]) * v;	
	
	return Vector4DF(c[6].x/255.0f,c[6].y/255.0f,c[6].z/255.0f,c[6].w/255.0f); 
}

float Image::GetPixelFilteredUV16 (float x, float y)
{
	float u = x * (m_Info.mXres - 1);
	float v = y * (m_Info.mYres - 1);
	int xu = u;
	int yu = v;
	u -= xu;
	v -= yu;
	
	float c[7];
	unsigned short i ;

	if (xu < 0 || yu < 0 || xu >= m_Info.mXres-1 || yu >= m_Info.mYres-1 ) return 0;

	i = GetPixel16 ( xu,    yu );	c[0] = float(i) / 65535.0f;
	i = GetPixel16 ( xu+1,  yu );	c[1] = float(i) / 65535.0f;
	i = GetPixel16 ( xu,  yu+1 );	c[2] = float(i) / 65535.0f;
	i = GetPixel16 ( xu+1,yu+1 );   c[3] = float(i) / 65535.0f;
	
	// bi-linear filtering
	c[4] = c[0] + (c[1]-c[0]) * u;
	c[5] = c[2] + (c[3]-c[2]) * u;
	c[6] = c[4] + (c[5]-c[4]) * v;	
	
	return c[6];
}


void Image::SetFormatFunc ( int chan )
{
	ImageInfo* info = getInfo ();
	SetFormatFunc ( info->eFormat );
}

void Image::SetFormatFunc ( ImageOp::Format eFormat )
{	
	switch ( eFormat ) {
	case ImageOp::BW8:
		m_GetPixelFunc = &Image::GetPixelBW8;
		m_SetPixelFunc = &Image::SetPixelBW8;
		m_FillFunc = &Image::FillBW8;
		m_ScaleFunc = &Image::ScaleBW8;
		m_PasteFunc = &Image::PasteBW8;
		m_RemapFunc = &Image::RemapBW8;
		m_ReformatFunc = &Image::ReformatBW8;
		m_AlphaFunc = &Image::AlphaBW8;
		break;
	case ImageOp::BW16:
		m_GetPixelFunc = &Image::GetPixelBW16;
		m_SetPixelFunc = &Image::SetPixelBW16;
		m_FillFunc = &Image::FillBW16;
		m_ScaleFunc = &Image::ScaleBW16;
		m_PasteFunc = &Image::PasteBW16;
		m_RemapFunc = &Image::RemapBW16;
		m_ReformatFunc = &Image::ReformatBW16;
		m_AlphaFunc = &Image::AlphaBW16;
		break;
	case ImageOp::BW32:
		m_GetPixelFunc = &Image::GetPixelBW32;
		m_SetPixelFunc = &Image::SetPixelBW32;
		m_FillFunc = &Image::FillBW32;
		m_ScaleFunc = &Image::ScaleBW32;
		m_PasteFunc = &Image::PasteBW32;
		m_RemapFunc = &Image::RemapBW32;
		m_ReformatFunc = &Image::ReformatBW32;
		m_AlphaFunc = &Image::AlphaBW32;
		break;
	case ImageOp::RGB8:		
		m_GetPixelFunc = &Image::GetPixelRGB8;
		m_SetPixelFunc = &Image::SetPixelRGB8;
		m_FillFunc = &Image::FillRGB8;
		m_ScaleFunc = &Image::ScaleRGB8;
		m_PasteFunc = &Image::PasteRGB8;
		m_RemapFunc = &Image::RemapRGB8;
		m_ReformatFunc = &Image::ReformatRGB8;
		m_AlphaFunc = &Image::AlphaRGB8;
		break;
	case ImageOp::RGBA8:		
		m_GetPixelFunc = &Image::GetPixelRGBA8;
		m_SetPixelFunc = &Image::SetPixelRGBA8;
		m_FillFunc = &Image::FillRGBA8;
		m_ScaleFunc = &Image::ScaleRGBA8;
		m_PasteFunc = &Image::PasteRGBA8;
		m_RemapFunc = &Image::RemapRGBA8;
		m_ReformatFunc = &Image::ReformatRGBA8;
		m_AlphaFunc = &Image::AlphaRGBA8;
		break;
	case ImageOp::RGB16:
		m_GetPixelFunc = &Image::GetPixelRGB16;
		m_SetPixelFunc = &Image::SetPixelRGB16;
		m_FillFunc = &Image::FillRGB16;
		m_ScaleFunc = &Image::ScaleRGB16;
		m_PasteFunc = &Image::PasteRGB16;
		m_RemapFunc = &Image::RemapRGB16;
		m_ReformatFunc = &Image::ReformatRGB16;
		m_AlphaFunc = &Image::AlphaRGB16;
		break;
	case ImageOp::RGBA32F:
		m_GetPixelFunc = &Image::GetPixelRGBA32F;
		m_SetPixelFunc = &Image::SetPixelRGBA32F;
		m_FillFunc = &Image::FillRGBA32F;
		m_ScaleFunc = &Image::ScaleRGBA32F;
		m_PasteFunc = &Image::PasteRGBA32F;
		m_RemapFunc = &Image::RemapRGBA32F;
		m_ReformatFunc = &Image::ReformatRGBA32F;
		m_AlphaFunc = &Image::AlphaRGBA32F;
		break;
	case ImageOp::BGR8:
		m_GetPixelFunc = &Image::GetPixelBGR8;
		m_SetPixelFunc = &Image::SetPixelBGR8;
		m_FillFunc = &Image::FillBGR8;
		m_ScaleFunc = &Image::ScaleBGR8;
		m_PasteFunc = &Image::PasteBGR8;
		m_RemapFunc = &Image::RemapBGR8;
		m_ReformatFunc = &Image::ReformatBGR8;
		m_AlphaFunc = &Image::AlphaBGR8;
		break;
	case ImageOp::F32:
		m_GetPixelFunc = &Image::GetPixelF32;
		m_SetPixelFunc = &Image::SetPixelF32;
		m_FillFunc = &Image::FillF32;
		m_ScaleFunc = &Image::ScaleF32;
		m_PasteFunc = &Image::PasteF32;
		m_RemapFunc = &Image::RemapF32;
		m_ReformatFunc = &Image::ReformatF32;
		m_AlphaFunc = &Image::AlphaF32;
		break;
	};

}

// Copy data from external source. Size & format must already be set.
void Image::TransferData ( char* buf ) 
{
	memcpy ( m_Pix.mCpu, buf, m_Pix.mSize );		// size valid after Resize

	m_Pix.Commit ();
}

void Image::TransferFrom ( Image* src_img )
{
	int xr = src_img->GetWidth();
	int yr = src_img->GetHeight();
	
	if ( xr > 0 && yr > 0 ) {

		// Determine new format
		// Note: Only resolution, format and data are transfered. 
		// All other flags (filtering, usage) are left alone.
		unsigned int orig_flags = getInfo()->eFlags;
		
		uchar dt = src_img->GetDataType( src_img->GetFormat() );
		SetFormat ( xr, yr, src_img->GetFormat() );				// Set new pixel format
		m_Pix.SetUsage( dt, m_UseFlags, xr,yr,1);
		m_Pix.mNum = xr * yr;

		SetEqual ( ImageOp::Channels, orig_flags );
		SetEqual ( ImageOp::FilterLo, orig_flags );
		SetEqual ( ImageOp::FilterHi, orig_flags );
		SetEqual ( ImageOp::UseLo, orig_flags );
		SetEqual ( ImageOp::UseHi, orig_flags );

		// Deep copy pixel data		
		CopyIntoBuffer ( m_Pix, src_img->m_Pix, m_Info.GetBytesPerPix(), xr, yr );
			
		if (src_img->HasFlag(ImageOp::Depth) && !src_img->HasFlag(ImageOp::DepthMerged)) {	// Check if we need seperate depth data.. 
			CopyIntoBuffer ( m_Depth, src_img->m_Depth, sizeof(XBYTE), xr, yr );
			m_Depth.SetUsage(DT_UCHAR, DT_CPU, xr, yr,1 );		// 8-bit depth
		}
		if (src_img->HasFlag(ImageOp::Alpha) && !src_img->HasFlag(ImageOp::AlphaMerged)) {
			CopyIntoBuffer(m_Alpha, src_img->m_Alpha, sizeof(XBYTE), m_Info.mXres, m_Info.mYres);
			m_Alpha.SetUsage(DT_UCHAR, DT_CPU, xr, yr,1 );		// 8-bit alpha
		}
		
		SetFormatFunc (0);
	}
}

void Image::CopyIntoBuffer ( DataPtr& dest, DataPtr& src, int bpp, int w, int h )
{
	dest.Clear();
	if ( src.mCpu == 0 ) return;
	dest.Resize ( bpp, w*h );
	src.CopyTo ( &dest, DT_CPU );
}

bool Image::LoadAlpha ( char *filename )
{
	Image ctrl;
	Image alpha_img ( 0, 0, ImageOp::RGBA8 );
	std::string errmsg;

	// Load the alpha image
	if ( ctrl.Load ( filename, errmsg ) ) {

		// Change format to RGBA8
		ChangeFormat ( ImageOp::RGBA8 );

		// Copy loaded image into alpha channel
        CopyAlpha ( &alpha_img );
	}		
	return true;
}

bool Image::LoadIncremental ( char *filename )
{

#ifdef BUILD_JPG
	CImageFormatJpg*	pJpgLoader;

	pJpgLoader = new CImageFormatJpg;
	if ( pJpgLoader->LoadIncremental ( filename, this ) ) {
		mpImageLoader = pJpgLoader;
		return true;
	}
	// Load failed. Delete loader.
	delete ( pJpgLoader );
#endif

	mpImageLoader = 0x0;
	return false;
}

ImageOp::FormatStatus Image::LoadNextRow ()
{
	if ( mpImageLoader == 0x0) return ImageOp::LoadNotReady;

	ImageOp::FormatStatus eStatus = mpImageLoader->LoadNextRow ();	
	if ( eStatus == ImageOp::LoadDone ) {		
		// Remove image loader when done
		delete mpImageLoader;
		mpImageLoader = 0x0;				
	}
	return eStatus;
}

bool Image::Load (char* filename, char* alphaname )
{
	std::string errmsg;
	Load ( filename, errmsg );
	bool result = LoadAlpha ( alphaname );		
	return result;
}
/*bool Image::LoadCubemap ( int n, char* filename )
{
	int xr, yr;
	ImageX temp;
	temp.Load ( filename );

	if ( n==0 ) {
		DeleteBuffers ();
		
		// Set new pixel format
		xr = temp.GetWidth ();
		yr = temp.GetHeight ();		
		m_Info.Size = GetBytesPerPixel() * xr * yr;	
		m_Info.mXres = xr; m_Info.mYres = yr;	
		m_Info.bDataOwner = true;		
		SetFormat ( ImageOp::RGB8 );
		SetBPP ( xr );

		// Add buffers for each face
		AddAttr ( IMGCUBE+0, -1, "cube0", GetBytesPerPixel(), xr * yr );
		AddAttr ( IMGCUBE+1, -1, "cube1", GetBytesPerPixel(), xr * yr );
		AddAttr ( IMGCUBE+2, -1, "cube2", GetBytesPerPixel(), xr * yr );
		AddAttr ( IMGCUBE+3, -1, "cube3", GetBytesPerPixel(), xr * yr );
		AddAttr ( IMGCUBE+4, -1, "cube4", GetBytesPerPixel(), xr * yr );
		AddAttr ( IMGCUBE+5, -1, "cube5", GetBytesPerPixel(), xr * yr );
		SetPivot (0, 0);								// Default pivot, top left.
		SetFormatFunc (0);
		
		m_Info.bCube = true;
	}
	CopyBuffer ( IMGCUBE + n, m_Pix.buf, &temp );
	
	updateTexture ();

	return true;
}*/

bool Image::Load ( std::string filename, std::string& errmsg)
{	
	char msg[1000];

	// NOTES ABOUT CImageFormats:
	// 1) Load functions return 'true' if the file was successfully loaded by
	// that format. On success, pNewImg==0 indicates the loader was able to use
	// the original m_Img structure for efficiency, otherwise pNewImg will hold 
	// the new image object.
	// 2) Failure of one image format loader does not mean another will fail.
	// 3) Each image format loader will maintain its own error status if it fails.
	
	// Get image type
	FILE* fp = fopen( filename.c_str(), "rb" );  
	if ( !fp ) {		
		errmsg = std::string("ERROR: File not found: ") + filename;
		return false;
	}
	unsigned char type[4];
	fread ((void*) type, sizeof(char), 4, fp );
	fclose ( fp );
	bool bTry = false;
	
	strcpy ( msg, filename.c_str() );

	// Try loading as a JPG file
	if ((type[0] == 0xD8 && type[1] == 0xFF) || (type[1] == 0xD8 && type[0] == 0xFF)) {		
		#ifdef BUILD_JPG
			CImageFormatJpg		jpg_format;
			if ( jpg_format.Load ( msg, this ) ) {
				errmsg = "";
				return true;		
			}
			jpg_format.GetStatusMsg ( msg );	
			bTry = true;
		#else
			errmsg = "JPEG library not included";
			return false;
		#endif
	}


	// Try loading as a TIFF file 
	if (type[0] == 0x49 && type[1] == 0x49 && type[2] == 0x2A ) {
		CImageFormatTiff	tiff_format;
		if ( tiff_format.Load ( msg, this  ) ) {
			return true;
		}
		tiff_format.GetStatusMsg ( msg );	
		bTry = true;
	}


#ifdef BUILD_BMP
	// Try loading as a BMP file
	if ((type[0] == 0x4D && type[1] == 0x42) || (type[1] == 0x4D && type[0] == 0x42)) {
		CImageFormatBmp		bmp_format;
		if ( bmp_format.Load ( msg, (Image*) this ) ) {
			//bind_data ( this );		// must rebind to update m_Info struct
			errmsg = "";
			return true;		
		}
		bmp_format.GetStatusMsg ( msg );
		bTry = true;
	}
#endif

	// Try loading as a PNG file
	if (type[0] == 0x89 && type[1] == 0x50 && type[2] == 0x4E && type[3] == 0x47) {
		CImageFormatPng		png_format;
		if ( png_format.Load ( msg, this ) ) {
			errmsg = "";
			return true;		
		}
		png_format.GetStatusMsg ( msg );
		bTry = true;
	}


	// Try loading as TGA file
	if( type[1] == 0 && (type[2] == 2 || type[2] == 3) ) {
		CImageFormatTga		tga_format;
		if ( tga_format.Load ( msg, this ) ) {
			errmsg = "";
			return true;
		}
		tga_format.GetStatusMsg ( msg );
		bTry = true;
	}

	// Generate Load Error Message - Unable to load		
	if ( !bTry ) { sprintf ( msg, "Unknown format.\n" ); }
	errmsg = std::string(msg) + std::string(" file: ") + filename;
	
	// Default filtering
	SetFilter( 0, ImageOp::Filter::Linear );

	return false;	
}

bool Image::Save (char *filename)
{
	std::string fname = filename;
	std::string fext = fname.substr ( fname.length()-3, 3 );

	#ifdef BUILD_PNG
		if ( fext.compare("png")==0 ) {
			CImageFormatPng		png_format;
			png_format.Save ( filename, this );
		}
	#endif
	#ifdef BUILD_JPG
		if ( fext.compare("jpg")==0 ) {
			CImageFormatJpg		jpg_format;
			jpg_format.Save ( filename, this );
		}
	#endif
	#ifdef BUILD_TIF
		if ( fext.compare("tif")==0 ) {
			CImageFormatTiff	tiff_format;
			tiff_format.Save ( filename, this );
		}
	#endif

   	return false;
}

void Image::SetFilter ( int chan, ImageOp::Filter ef )
{
	ImageInfo* info = getInfo ();
	info->SetFilter ( ef );
}
void Image::SetFilter ( std::string name, ImageOp::Filter ef )
{
	SetFilter ( 0, ef );
}

ImageOp::Filter Image::GetFilter()
{
	return getInfo()->GetFilter ();
}


// Scales 'this' image with filtering.
void Image::Scale (int nx, int ny)
{
	// Create new image data for rescaling
	Image* new_img = new Image ( nx, ny, GetFormat () );

	// Rescale current data into new data buffer
	{ (this->*m_ScaleFunc) ( new_img->GetData(), GetFormat(), nx, ny ); }

	// Make new data the current data (transfer ownership)
	TransferFrom ( new_img );
	delete ( new_img );
}

// Copies 'this' image into a new image. 'this' is not modified.
// The new image will have same image format.
Image* Image::CopyNew ()
{
	// Create new image
	Image* new_img = new Image (  GetWidth(), GetHeight(), GetFormat() );
	
	dbgprintf ( "ERROR: CopyNew not implemented for ImageX.\n");

	// Copy current data into new one (same format)
	//new_img->CopyBuffers ( this );

	// Return new image 
	// (calling function is now responsible for it!)
	//new_img->Update ( 1 ); 
	return new_img;
}

// Copies 'this' image into a new image. 'this' is not modified.
// New image will take on the format specified
Image* Image::CopyNew ( ImageOp::Format new_format )
{
	// Create new image and copy with same format
	Image* new_img = CopyNew ();
	
	// Change format
	//new_img->ChangeFormat ( new_format );
	//new_img->Update ( 1 ); 
	return new_img;
}

// Copies 'this' image into another one. 
// The caller allocated the dest image
void Image::Copy ( Image* new_img )
{
	//new_img->Resize ( m_Info.mXres, m_Info.mYres, m_Info.eFormat );
	//new_img->CopyBuffers ( (GeomX*) this );	
	//new_img->Update ( 1 ); 	
}

// Copies 'this' image into another one, and changes format.
// Caller allocates the dest image
void Image::Copy ( Image* new_img, ImageOp::Format new_format )
{
	//new_img->Resize ( m_Info.mXres, m_Info.mYres, m_Info.eFormat );
	//new_img->CopyBuffers ( (GeomX*) this );	
	//new_img->ChangeFormat ( new_format );
	//new_img->Update ( 1 ); 	
}

// Copies the color data of another image into the alpha channel of 'this' one. 
// The dimensions of 'this' remain the same. If the alpha image size
// does not match, it will be clipped. This will do nothing if
// the pixel format of 'this' image does not have an alpha channel. 
void Image::CopyAlpha ( Image* new_alpha )
{	
	/*if ( m_Info.HasOption ( m_Info.eFlags, ImageOp::Alpha ) ) {
		 (this->*m_AlphaFunc) ( 0, 0, GetWidth(), GetHeight(), new_alpha->GetData(), new_alpha->GetFormat(), new_alpha->GetWidth(), new_alpha->GetHeight() );
		 Update ( 1 );
	}*/
}



/*
bool Image::UpdateGL ()
{
	Image* img = (Image*) b.dat;
	int flgs = img->getUpdate();
	GLuint* VBO = (GLuint*) img->getVBO();
	
	//debug.Printf ( "Updated Shape: %s, %p\n", img->getName().c_str(), mShape );

	if ( flgs & (objFlags) ObjectX::fBuild ) {			

		// Allocate VBO array
		#ifdef DEBUG_GLERR
			glGetError ();
		#endif
		if ( VBO != 0x0 ) {
			glDeleteTextures ( 1, VBO );
			//debug.Printf ( "Deleted GLID: %d\n", mShape->VBO[0] );
			setGLID ( -1 );
			DeleteVBO ();
		}
		VBO = (GLuint*) CreateVBO ( 1 );

		// Generate textures
		glGenTextures ( 1, VBO );
		#ifdef DEBUG_GLERR
			if ( glGetError() != GL_NO_ERROR ) return false;
		#endif
		printf ( 'data', INFO, "Data, GPU: %d, %s (GLID: %d) \n", img->getID(), img->getName().c_str(), VBO[0] );
		img->setGLID ( VBO[0] );					// fast access to GL buffer data
		img->clearUpdate ( ObjectX::fBuild );		// done getting new vbo
		flgs |= ObjectX::fUpdatePos;				// force texture update
	}	
	if ( flgs & (objFlags) ObjectX::fUpdatePos ) {

		int chan = img->GetParamInt ( b.pos, "channel" );
		ImageInfo* info = img->getInfo ( chan );
		XBYTE* pixbuf = img->getData ( chan );

		// How many images?
		int imgcnt = info->IsCube() ? 6 : 1;
		int imgbind = info->IsCube() ? GL_TEXTURE_CUBE_MAP : GL_TEXTURE_2D;
		int imgfilter = info->MipFilter() ? (info->NoFilter() ? GL_LINEAR_MIPMAP_NEAREST : GL_LINEAR_MIPMAP_LINEAR) :
											(info->NoFilter() ? GL_NEAREST : GL_LINEAR );		
		// Bind texture
		glBindTexture ( imgbind, VBO[ 0 ] );
		glPixelStorei( GL_UNPACK_ALIGNMENT, 1);								// pixel alignment
		glTexParameterf ( imgbind, GL_TEXTURE_WRAP_S, GL_REPEAT );			// wrapping
		glTexParameterf ( imgbind, GL_TEXTURE_WRAP_T, GL_REPEAT );
		glTexParameteri ( imgbind, GL_TEXTURE_MIN_FILTER, imgfilter );		// filtering       
		glTexParameteri ( imgbind, GL_TEXTURE_MAG_FILTER, (imgfilter==GL_LINEAR_MIPMAP_LINEAR) ? GL_LINEAR : imgfilter );		
		
		
		int ms = GL_MAX_TEXTURE_SIZE;

		// *NOTE* 
		// glewInit causes INVALID ENUM on some calls to glTexImage2D. But the call still succeeds.
		// The workaround is to temporarily disable ARB debug here.
		//rendGL->setARBDebug ( 0 );

		// Load textures
		for (int n=0; n < imgcnt; n++ ) {
			int target = info->IsCube() ? GL_TEXTURE_CUBE_MAP_POSITIVE_X+n : GL_TEXTURE_2D ;
			switch ( info->eFormat ) {				
			case ImageOp::BW8:		glTexImage2D ( target, 0, GL_LUMINANCE8, info->mXres, info->mYres,	0, GL_LUMINANCE,	GL_UNSIGNED_BYTE, pixbuf );	break;
			case ImageOp::BW16:		glTexImage2D ( target, 0, GL_LUMINANCE16, info->mXres, info->mYres, 0, GL_LUMINANCE,	GL_UNSIGNED_SHORT, pixbuf );	break;
			case ImageOp::BW32:		glTexImage2D ( target, 0, GL_LUMINANCE32, info->mXres, info->mYres, 0, GL_LUMINANCE,	GL_UNSIGNED_SHORT, pixbuf );	break;			
			case ImageOp::RGB8:		glTexImage2D ( target, 0, GL_RGB8, info->mXres, info->mYres,		0, GL_RGB,			GL_UNSIGNED_BYTE, pixbuf );		break;
			case ImageOp::BGR8:		glTexImage2D ( target, 0, GL_RGB8, info->mXres, info->mYres,		0, GL_BGR_EXT,		GL_UNSIGNED_BYTE, pixbuf );	break;
			case ImageOp::RGBA32F:	glTexImage2D ( target, 0, GL_RGBA32F, info->mXres, info->mYres,		0, GL_RGBA,			GL_FLOAT, pixbuf );				break;
			case ImageOp::F32:		glTexImage2D ( target, 0, GL_R32F, info->mXres, info->mYres,		0, GL_LUMINANCE,	GL_FLOAT, pixbuf );	break;
			};			
			// Generate mipmaps
			if ( info->MipFilter() )	glGenerateMipmap ( target );			
		}
		// Turn ARB debugging back on.
		//rendGL->setARBDebug ( 1 );		

		img->clearUpdate ( ObjectX::fAll );
	}
	return true;
}

void Image::render_data ( bufRef b )
{	
	 glPushAttrib ( GL_LIGHTING_BIT );
	glDisable ( GL_LIGHTING );		
	glEnable  ( GL_TEXTURE_2D );
	glEnable ( GL_BLEND );
	glColor3f ( 1, 1, 1 );

	GLuint* VBO = (GLuint*) getVBO();

	//glLoadIdentity ();
	//rendGL->getCurrCam()->setModelMatrix ();

	if ( getInfo()->IsCube() ) {		
		glEnable ( GL_TEXTURE_CUBE_MAP_EXT );
		glBindTexture ( GL_TEXTURE_CUBE_MAP, VBO[0] );		
		//glTranslatef ( 0, 0, 0 );
		glScalef ( 160.0, 160.0, 160.0 );
		//glCallList ( sph );
		glDisable ( GL_TEXTURE_CUBE_MAP_EXT );
	} else {
		//glMultMatrixf ( m_Obj->getTformData() );
		glBindTexture ( GL_TEXTURE_2D, VBO[0] );
		glBegin ( GL_QUADS );
		glTexCoord2f ( 0, 0 ); glVertex3f ( -10, 0.01, -10 );
		glTexCoord2f ( 1, 0 ); glVertex3f ( -10, 0.01, 10 );
		glTexCoord2f ( 1, 1 ); glVertex3f (  10, 0.01, 10 );
		glTexCoord2f ( 0, 1 ); glVertex3f (  10, 0.01, -10 );
		glEnd ();	
			
	}
	glDisable  ( GL_BLEND );
	glDisable  ( GL_TEXTURE_2D );
	glPopAttrib (); 
}

void Image::Draw ()	
{
	glEnable ( GL_TEXTURE_2D );
	glBindTexture ( GL_TEXTURE_2D, (GLuint) m_GLID );		
	glBegin ( GL_QUADS );
	glTexCoord2f ( 0, 0 );	glVertex2f ( 0, 0 );
	glTexCoord2f ( 1, 0 );	glVertex2f ( 1, 0 );
	glTexCoord2f ( 1, 1 );  glVertex2f ( 1, 1 );
	glTexCoord2f ( 0, 1 );  glVertex2f ( 0, 1 );
	glEnd (); 
}

void Image::createTexture ()
{
	glGenTextures ( 1, (GLuint*) &m_GLID );	
	glBindTexture ( GL_TEXTURE_2D, m_GLID );
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );	
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1);	
}

*/
