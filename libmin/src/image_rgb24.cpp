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
#include "image.h"

#include <assert.h>

void Image::SetPixelRGB24 (int x, int y, XBYTE r, XBYTE g, XBYTE b, XBYTE a)
{
	if ( x>=0 && y>=0 && x < getInfo()->mXres && y < getInfo()->mYres ) {
		XBYTE* pix = (XBYTE*) GetData() + ( (y * getInfo()->mXres + x) * getInfo()->GetBytesPerPix() );
		*pix++ = r;
		*pix++ = g;
		*pix++ = b;
		//  setNotify ( 1 ); // *** NOTIFY(1)
	}	
}
void Image::GetPixelRGB24 (int x, int y, XBYTE& r, XBYTE& g, XBYTE& b, XBYTE& a )
{
	if ( x>=0 && y>=0 && x < getInfo()->mXres && y < getInfo()->mYres ) {
		XBYTE* pix = (XBYTE*) GetData() + ( (y * getInfo()->mXres + x) * getInfo()->GetBytesPerPix() );
        r = *pix++;
        g = *pix++;
        b = *pix++;
		a = 255;
	}
}

void Image::FillRGB24 (XBYTE r, XBYTE g, XBYTE b, XBYTE a)
{
	// Temporary fill buffer
	assert ( getInfo()->GetBytesPerRow() <= 16384 );
	for (uint x=0; x < getInfo()->GetBytesPerRow();) {
		fillbuf[x++] = r; 
		fillbuf[x++] = g; 
		fillbuf[x++] = b;	
	}
    
    XBYTE *dest_pix, *dest_pix_stop;
	dest_pix = (XBYTE*) GetData();
	dest_pix_stop = dest_pix + getInfo()->GetSize();
	for (; dest_pix < dest_pix_stop;) {
		memcpy (dest_pix, fillbuf, getInfo()->GetBytesPerRow());
		dest_pix += getInfo()->GetBytesPerRow();
	}	
	// setNotify ( 1 ); // *** NOTIFY(1)
}


void Image::RemapRGB24 ( unsigned int vmin, unsigned int vmax )
{
	XBYTE* src = (XBYTE*) GetData();
	XBYTE* src_stop = src + (getInfo()->mXres*getInfo()->mYres);	
	
	unsigned long mMin, mMax;
	mMin = ((unsigned long) 1 << 16)-1;
	mMax = 0;
	for (; src < src_stop; ) {
		if ( *src < mMin ) mMin = *src;
		if ( *src > mMax ) mMax = *src;
		src++;
	}
	if ( vmin == vmax ) return;
	
	src = (XBYTE*) GetData();
	unsigned int vdelta = vmax-vmin;
	for (; src < src_stop; ) {
		*src = (XBYTE) ( float(vmin) + float(*src - mMin)*vdelta / (mMax-mMin) );
		src++;
	}
}


// Reformat - Reformats given pixel format to
// another pixel format
void Image::ReformatRGB24 ( ImageOp::Format eFormat )
{
	Image* new_img = new Image (  getInfo()->mXres, getInfo()->mYres, eFormat );
	
	XBYTE* src = (XBYTE*) GetData();
	XBYTE* src_stop = src + getInfo()->GetSize();	
	XBYTE* dest = new_img->GetData ();		

	if ( eFormat==ImageOp::RGBA32 ) {		// Target format RGBA32
		for (; src < src_stop; ) {
			*dest++ = *src++;
			*dest++ = *src++;
			*dest++ = *src++;
			*dest++ = 255;
		}		
	} else if ( eFormat==ImageOp::BGR24 ) {	// Target format BGR24
		for (; src < src_stop; ) {
			*dest++ = *(src++ + 2);
			*dest++ = *src++;
			*dest++ = *(src++ - 2);
		}
	} else if ( eFormat==ImageOp::BW8 ) {	// Target format BW8		
		for (; src < src_stop; ) {
			*dest++ = (*src + int(*(src+1)) + *(src+2)) / 3 ;
			src += 3;
		}
	}
	TransferFrom ( new_img );
	delete ( new_img );
}

// Paste - Pastes a portion of an image into another image.
// Subselection: yes	- Allows sub-selection of source
// Cropping:     yes	- Allows cropping into target
// Offsetting:   yes	- Allows offsetting into target
// Scaling:      no		- Allows rescaling of source
// Filtering:    no		- Allows filtering of source
// Rotation:	 no		- Allows rotation of source
void Image::PasteRGB24 ( int x1, int y1, int x2, int y2, int offx, int offy, XBYTE* dest, ImageOp::Format dest_format, int destx, int desty )
{
	XBYTE *src, *src_start, *src_end;
	XBYTE *dest_start, *dest_end;
	int src_wid, src_pitch;
	int dest_wid, dest_pitch, dest_bpr;


	if ( dest_format==ImageOp::RGB24 ) {
		if ( getInfo()->QueryPaste ( GetFormat(), GetWidth(), GetHeight(), GetData(), x1, y1, x2, y2,
						  dest_format, destx, desty, dest, offx, offy,
						  src_start, src_end, src_wid, src_pitch,
						  dest_start, dest_end, dest_wid, dest_pitch) ) 
		{
			dest_bpr = GetBytesPerRow ( destx, desty, dest_format );
			for (src = src_start, dest = dest_start; src < src_end;) {
				memcpy (dest, src, src_wid);		
				src += GetBytesPerRow ();
				dest += dest_bpr;
			}
		}	
	}	
}

// Scale - Rescales an image into another image
// Subselection: no		- Allows sub-selection of source
// Cropping:     no		- Allows cropping into target
// Offsetting:   no		- Allows offsetting into target
// Scaling:      yes	- Allows rescaling of source
// Filtering:    yes	- Allows filtering of source
// Rotation:	 no		- Allows rotation of source
// Limitations: Rescaled size must match target buffer
void Image::ScaleRGB24 ( XBYTE* dest, ImageOp::Format dest_format, int destx, int desty )
{
	assert ( GetFormat()==dest_format );

	// Limitation: Rescaled size must match target buffer
	int nx = destx;
	int ny = desty;

	// Compute pixel addresses	
	int dest_bpr = GetBytesPerRow(destx,desty,dest_format);
	XBYTE* src;												// Current pixel in source
	XBYTE* src_row;											// Current row in source (start of row)	
	XBYTE* dest_rowend = dest + dest_bpr;					// End of row in dest
	XBYTE* dest_end = dest + GetSize(destx,desty,dest_format);			// End of entire image in dest
	double delta_x = double(getInfo()->mXres-2) / double(nx);	// Filtering distances
	double delta_y = double(getInfo()->mYres-2) / double(ny);
	int iDiffX = int(delta_x*0.5) * 3;						// Filtering deltas (in bytes)
	int iDiffY = getInfo()->GetBytesPerRow();
	float sx, sy;											// Current pixel (with sub-pixel accuracy)

	sy = 1.0;	
	for (; dest < dest_end;) {
		src_row = (XBYTE*) GetData() + int(sy) * getInfo()->GetBytesPerRow();
		sx = 1.0;
		for (; dest < dest_rowend;) {
			src = src_row + (int) sx*3;
			// Filtering (Fast, poor quality but better than nothing)
			// RED
			*dest = *(src - iDiffX) >> 3;				// left
			*dest += *(src) >> 1;						// center
			*dest += *(src+iDiffX) >> 3;				// right
			*dest += *(src-iDiffY) >> 3;				// up
			*dest++ += *(src+iDiffY) >> 3;				// down			
			src++;

			// GREEN
			*dest = *(src - iDiffX) >> 3;				// left
			*dest += *(src) >> 1;						// center
			*dest += *(src+iDiffX) >> 3;				// right
			*dest += *(src-iDiffY) >> 3;				// up
			*dest++ += *(src+iDiffY) >> 3;				// down			
			src++;
				
			// BLUE
			*dest = *(src - iDiffX) >> 3;				// left
			*dest += *(src) >> 1;						// center
			*dest += *(src+iDiffX) >> 3;				// right
			*dest += *(src-iDiffY) >> 3;				// up
			*dest++ += *(src+iDiffY) >> 3;				// down	

			sx += (float) delta_x;
		}
		sy += (float) delta_y;				
		dest_rowend += dest_bpr;
	}	
}

// Alpha Paste - Copies alpha from another source
void Image::AlphaRGB24 ( int x1, int y1, int x2, int y2, XBYTE* src, ImageOp::Format src_format, int src_x, int src_y )
{
	// No alpha channel !

	return;
}

