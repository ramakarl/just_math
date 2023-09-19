
#include "image_info.h"
#include "datax.h"		// for DT_ types

ImageInfo::ImageInfo ( int xr, int yr, ImageOp::Format ef )
{
	SetFormat ( xr, yr, ef );
}
ImageInfo::ImageInfo ()
{
	Reset ();	
}

void ImageInfo::Reset ()
{
	mXres = 0; mYres = 0;
	eFlags = 0;
}


// Flags - options based on format
unsigned int ImageInfo::GetFlags ( ImageOp::Format ef )
{
	// Format	Color, Masked, Alpha, AMerg, Depth,	DMerged, Indexed, Low,    High, True, Channels
	// BW1		-		-		-		-		-		-		T		-		-		-		-
	// BW8		-		-		-		-		-		-		-		T		-		-		-
	// RGB8		T		-		-		-		-		-		T		-		-		-		-
	// RGBA8	T		-		-		-		-		-		T		-		-		-		-
	// RGB12	T		-		-		-		-		-		-		T		-		-		-
	// RGB16	T		-		-		-		-		-		-		-		T		-		-	
	// RGBA32F	T		-		T		T		-		-		-		-		-		T		-
	unsigned int flags = 0;
	switch (ef) {
	case ImageOp::RGB8: case ImageOp::BGR8: {
		SetFlag ( flags, ImageOp::Color, true);		
		SetFlag ( flags, ImageOp::Alpha, false);
		SetFlag ( flags, ImageOp::AlphaMerged, true);
		SetFlag ( flags, ImageOp::Depth, false);
		SetFlag ( flags, ImageOp::DepthMerged, true);
		SetFlag ( flags, ImageOp::Trueclr, true);
	} break;
	case ImageOp::RGBA8: {
		SetFlag ( flags, ImageOp::Color, true);
		SetFlag ( flags, ImageOp::Trueclr, true);
		SetFlag ( flags, ImageOp::Alpha, true);
		SetFlag ( flags, ImageOp::AlphaMerged, true);
		SetFlag ( flags, ImageOp::Depth, false);
		SetFlag ( flags, ImageOp::DepthMerged, true);
	} break;
	case ImageOp::BW8: {
		SetFlag ( flags, ImageOp::Color, false);
		SetFlag ( flags, ImageOp::Alpha, false);
		SetFlag ( flags, ImageOp::Lowclr, true);
	} break;
	case ImageOp::BW16: {
		SetFlag ( flags, ImageOp::Color, false);
		SetFlag ( flags, ImageOp::Alpha, false);
		SetFlag ( flags, ImageOp::Highclr, true);
	} break;
	case ImageOp::BW32: {
		SetFlag ( flags, ImageOp::Color, false);
		SetFlag ( flags, ImageOp::Alpha, false);
		SetFlag ( flags, ImageOp::Trueclr, true);
	} break;
	case ImageOp::F32: {
		SetFlag ( flags, ImageOp::Color, false);		
		SetFlag ( flags, ImageOp::Alpha, false);
		SetFlag ( flags, ImageOp::AlphaMerged, true);
		SetFlag ( flags, ImageOp::Depth, false);
		SetFlag ( flags, ImageOp::DepthMerged, true);
		SetFlag ( flags, ImageOp::Trueclr, true);
	} break;
	}
	return flags;
}



// Data type - value depends on format
unsigned char ImageInfo::GetDataType (ImageOp::Format ef)
{
	switch (ef) {
	case ImageOp::BW8:								return DT_UCHAR;	break;
	case ImageOp::BW16:								return DT_USHORT;	break;
	case ImageOp::BW32:								return DT_UINT;		break;	
	case ImageOp::RGB16:							return DT_USHORT3;	break;
	case ImageOp::RGB8: case ImageOp::BGR8:			return DT_UCHAR3;	break;
	case ImageOp::RGBA8:							return DT_UCHAR4;	break;		
	case ImageOp::F32:								return DT_FLOAT;	break;
	case ImageOp::RGBA32F:							return DT_FLOAT4;	break;	
	}
	return 0;
}

// Filtering Options
void ImageInfo::SetFilter ( ImageOp::Filter ef )
{
	switch (ef) {
	case ImageOp::NoFilter:			SetFlag ( eFlags, ImageOp::FilterLo, false ); SetFlag ( eFlags, ImageOp::FilterHi, false ); break;
	case ImageOp::Linear:			SetFlag ( eFlags, ImageOp::FilterLo, true ); SetFlag ( eFlags, ImageOp::FilterHi, false ); break;
	case ImageOp::MipNoFilter:		SetFlag ( eFlags, ImageOp::FilterLo, false ); SetFlag ( eFlags, ImageOp::FilterHi, true ); break;
	case ImageOp::MipLinear:		SetFlag ( eFlags, ImageOp::FilterLo, true); SetFlag ( eFlags, ImageOp::FilterHi, true ); break;
	};
}

ImageOp::Filter ImageInfo::GetFilter ()
{
	if (HasFlag (eFlags, ImageOp::FilterHi)) {
		// Mipmap
		if (HasFlag (eFlags, ImageOp::FilterLo)) return ImageOp::MipLinear;
		else return ImageOp::MipNoFilter;
	} else {
		// Standard
		if (HasFlag (eFlags, ImageOp::FilterLo)) return ImageOp::Linear;
		else return ImageOp::NoFilter;
	}
}

// Get smallest power greater than x
int ImageInfo::QuerySmallestPower (int x)
{
	int i = 1;
	for (; i < x; i <<= 1);
	return i;
}

bool ImageInfo::QueryClip ( int& sx1, int& sy1, int& sx2, int& sy2, int cx1, int cy1, int cx2, int cy2 )
{
    if (sx1 <= cx2 && sx2 >= cx1) {
		if (sy1 <= cy2 && sy2 >= cy1) {
			if (sx1 < cx1) sx1 = cx1;
			if (sx2 > cx2) sx2 = cx2;
			if (sy1 < cy1) sy1 = cy1;
			if (sy2 > cy2) sy2 = cy2;
			return true;
		}		
	}
	return false;
}

void ImageInfo::QueryRect (ImageOp::Format f, int src_x, XBYTE* src, int sx1, int sy1, int sx2, int sy2, 
						   XBYTE*& src_start, XBYTE*& src_end, int& src_wid, int& src_pitch)
{
	// First row of rectangle
	src_start = src + (sy1 * (unsigned long) src_x + sx1) * GetBytesPerPix ( f );
	
	// Last row of rectangle
	src_end = src + (sy2 * (unsigned long) src_x + sx1) * GetBytesPerPix ( f );

	// Width of rectangle (in bytes)
	src_wid = GetBytesPerPix  ( f ) * (unsigned long) (sx2-sx1+1);			// width of row (in bytes)

	// Pitch between rows (in bytes), from end of last span to beginning of next span
	src_pitch = GetBytesPerPix  ( f ) * (unsigned long) src_x - src_wid;
}

// QueryPaste
// Source is the image to be copies. Dest is the target paste area.
// This functions does the following:
// - Clips source rectangle to source image size (determine valid source sub-region)
// - Clips dest rectangle to destination image size
// - Computes source start/end offsets for rectangle (for any pixel format)
// - Computes dest start/end offsets for rectangle (for any pixel format)
bool ImageInfo::QueryPaste (ImageOp::Format src_f, int src_x, int src_y, XBYTE* src_dat, int sx1, int sy1, int sx2, int sy2,
							ImageOp::Format dest_f, int dest_x, int dest_y, XBYTE* dest_dat, int dx1, int dy1, 
							XBYTE*& src_start, XBYTE*& src_end, int& src_wid, int& src_pitch, 
							XBYTE*& dest_start, XBYTE*& dest_end, int& dest_wid, int& dest_pitch)		
{
	int sxl1 = sx1, syl1= sy1;	
	if (QueryClip (sx1, sy1, sx2, sy2, 0, 0, src_x-1, src_y-1)) {
		dx1 += sx1 - sxl1;
		dy1 += sy1 - syl1;
		int dxl1 = dx1, dyl1 = dy1;
		int dx2 = dx1 + (sx2-sx1);
		int dy2 = dy1 + (sy2-sy1);
		if (QueryClip (dx1, dy1, dx2, dy2, 0, 0, dest_x-1, dest_y-1)) {
			sx1 += dx1 - dxl1;
			sy1 += dy1 - dyl1;
			sx2 = sx1 + (dx2 - dx1);
			sy2 = sy1 + (dy2 - dy1);			
			QueryRect (src_f, src_x, src_dat, sx1, sy1, sx2, sy2, src_start, src_end, src_wid, src_pitch);
			QueryRect (dest_f, dest_x, dest_dat, dx1, dy1, dx2, dy2, dest_start, dest_end, dest_wid, dest_pitch);
			return true;			
		}
	}	
	return false;
}

void ImageInfo::QueryPowerSize (int &x, int &y)
{	
	x = QuerySmallestPower (mXres);
	y = QuerySmallestPower (mYres);
}
