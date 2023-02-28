
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

#define DEBUG_TIF	false		// enable this for tif debug output

//**************************************
//
// TIFF Format
//
// NOTE: CURRENT CAPABILITIES
//		- Load supports:
//					1 bits/channel			BW
//					8,16,32 bit/channel		Grayscale (no alpha)
//					8,16,32 bit/channel		RGB (with or without alpha)
//		- Save supports:
//					8 bit					RGB (no alpha)
//					16 bit					RGB (no alpha)
//
// * NO LZW COMPRESSION *
//
//**************************************

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "image.h"
#include "imageformat_tiff.h"

CImageFormatTiff::CImageFormatTiff () : CImageFormat()
{
	m_DebugTif = DEBUG_TIF;
	m_eTiffStatus = TifOk;
	m_eTag = TifDocname;
	m_eCompression = TifCompNone;
	m_ePhoto = TifRgb;
	m_eMode = TifColor;
	m_Xres = 0;
	m_Yres = 0;
	m_bHasAlpha = false;
	m_NumChannels = 0;
	m_BitsPerChannelOffset = 0;
	m_NumStrips = 0;
	m_StripOffsets = 0;
	m_StripCounts = 0;
	m_RowsPerStrip = 0;
	m_BitsPerPixel = 0;
	m_BytesPerRow = 0;
}

bool CImageFormatTiff::Load ( char *filename, Image* pImg )
{
	StartLoad ( filename, pImg );
	bool result = LoadTiff ( filename );
	if (result) FinishLoad ();
	return result;
}

bool CImageFormatTiff::Save (char *filename, Image* pImg)
{
	m_pOrigImage = pImg;
	m_pNewImage = 0;
	m_eStatus = ImageOp::Saving;
	strcpy (m_Filename, filename);
	return SaveTiff (filename);
}



// Function: LoadTiff
//
// Input:
//		m_name				Set to TIFF_FORMAT 
//		m_filename			Name of TIFF file to load
// Output:
//		m_success			Set to TRUE of FALSE
//		m_mode				Format mode (BW, GRAY, RGB, INDEX)
//		m_num_chan			Number of channels present
//		m_bpc[n]			Bits per channel for channel [n]
//		m_compress			Compression mode (NONE, PACKBITS, LZW)
//		m_photo			Photometric interpretation
//		m_num_strips		Number of strips
//		m_rps				Rows per strip
bool CImageFormatTiff::LoadTiff (char *filename)
{
	unsigned long pnt;

	m_Tif = fopen ( filename, "rb" );	
	if ( m_Tif == 0x0 ) {
		m_eStatus = ImageOp::FileNotFound;		
		return false;
	}
	fseek ( m_Tif, 0, SEEK_END );
	uint64_t sz = ftell ( m_Tif );
	fseek ( m_Tif, 0, SEEK_SET );	
	m_Buf.attachFromFile (m_Tif, sz);
	m_Buf.startRead ();

	m_bHasAlpha = false;
	m_Xres = 0;
	m_Yres = 0;		
	unsigned short byteorder = m_Buf.getUShort();
	if (byteorder  != (XBYTE2) TIFF_BYTEORDER) {
		//m_Buf.SetOrder (BUFFER_ORDER_MBF);				
	}
	unsigned short magic = m_Buf.getUShort ();
	if ( magic != TIFF_MAGIC) {
		m_eTiffStatus = TifNoMagic; m_eStatus = ImageOp::InvalidFile;
		m_pNewImage = 0x0; fclose ( m_Tif );
		return false;
	}

	pnt = m_Buf.getULong ();
	
	m_Buf.setPos (pnt);

	if ( !LoadTiffDirectory () ) { m_pNewImage = 0x0; fclose ( m_Tif ); return false; }		
	
	// success
	m_eTiffStatus = TifOk;
	m_eStatus = ImageOp::Successs;
	return true;
}

void CImageFormatTiff::ComputeMinMaxData (unsigned long count, unsigned long offset, int row)
{
	char in[TIFF_BUFFER];
	int x, y, lastrow;	

	m_Buf.setPos (offset);						// Set file position to TIFF data
	lastrow = row + m_RowsPerStrip - 1;
	if (lastrow > m_Yres-1) lastrow = m_Yres-1;		// Determine last row to process
	
	switch (m_eMode) {
	case TifBw: {							// Black & White TIFF		
		m_Min = 0; m_Max = 1;
	} break;
	case TifGrayscale: {						// Grayscale TIFF
		switch (m_BitsPerChannel[TifGray]) {
		case 8: {									// 8 Bits per Channel			
			uchar* pIn;			
			for (y = row; y <= lastrow; y++) {
				m_Buf.getBuf ( in, m_BytesPerRow );
				pIn = (uchar*) in;
				for (x=0; x < m_Xres; x++) {
					if ( *pIn < m_Min ) m_Min = *pIn;
					if ( *pIn > m_Max ) m_Max = *pIn;
					pIn++;
				}
			}
		} break;
		case 16: {									// 16 Bits per Channel			
			XBYTE2 *pIn;			
			for (y = row; y <= lastrow;	y++) {
				m_Buf.getBuf ( in, m_BytesPerRow );
				pIn = (XBYTE2 *) in;
				for (x=0; x < m_Xres; x++) {
					if ( *pIn < m_Min ) m_Min = *pIn;
					if ( *pIn > m_Max ) m_Max = *pIn;
					pIn++;
				}
			}
		 } break;
		case 32: {									// 32 Bits per Channel
			XBYTE4 *pIn;
			for (y = row; y <= lastrow;	y++) {
				m_Buf.getBuf (in, m_BytesPerRow );
				pIn = (XBYTE4 *) in;
				for (x=0; x < m_Xres; x++ ) {
					if ( *pIn < m_Min ) m_Min = *pIn;
					if ( *pIn > m_Max ) m_Max = *pIn;
					pIn++;
				}
			}
		} break;
		}
	} break;
	}
}

void CImageFormatTiff::ComputeMinMax ()
{
	unsigned long pos, row;	
	unsigned long pos_offsets, pos_counts;		// These are not pointers, must increment by 4!
	unsigned long count, offset, strip;
	
	// Reset min/max
	m_Min = 0xFFFFFFFF;
	m_Max = 0;

	pos = m_Buf.getPosInt ();
	pos_counts = m_StripCounts;
	pos_offsets = m_StripOffsets;
	if (m_NumStrips==1) {		
		count = pos_counts;
		offset = pos_offsets;
		row = 0;			
		ComputeMinMaxData (count, offset, row);
	} else {		
		row = 0;		
		for (strip=0; strip < m_NumStrips; strip++) {
			m_Buf.setPos (pos_counts);
			count = m_Buf.getUShort();
			m_Buf.setPos  (pos_offsets);
			offset = m_Buf.getULong();

			ComputeMinMaxData (count, offset, row);
			
			row += m_RowsPerStrip;
			pos_counts+=4;						// These are not pointers, must increment by 4!
			pos_offsets+=4;
		}
	}				
	m_Buf.setPos (pos);
}

bool CImageFormatTiff::LoadTiffDirectory ()
{
	unsigned int num;	
	unsigned long offset, pos;	
		
	// Read Number of TIFF Directory Entries
	num = m_Buf.getUShort();	

	// Read TIFF Directory Entries to fill ImageFormat Info
	for (unsigned int n = 0; n <= num; n++)	{
		if (!LoadTiffEntry ()) {
			// Error set by LoadTiffEntry
			return false;
		}
	}

	// Make sure we can support this TIF file
	if (m_eCompression == TifCompLzw) {		
		m_eTiffStatus = TifLzwNotSupported;
		m_eStatus = ImageOp::FeatureNotSupported;		
		return false;
	}

	// Create and prepare Image
	ImageOp::Format eNewFormat;

	switch (m_eMode) {						
	case TifColor: {									// Color Image
		pos = m_Buf.getPosInt();
		m_Buf.setPos (m_BitsPerChannelOffset);			// Get bits per channel
		m_BitsPerChannel[TifRed] =		m_Buf.getUShort();
		m_BitsPerChannel[TifGreen] =	m_Buf.getUShort();
		m_BitsPerChannel[TifBlue] =		m_Buf.getUShort();
		if ( m_DebugTif ) {
			dbgprintf ( "BPC Red:   %d\n", m_BitsPerChannel[TifRed] );
			dbgprintf ( "BPC Green: %d\n", m_BitsPerChannel[TifGreen] );
			dbgprintf ( "BPC Blue:  %d\n", m_BitsPerChannel[TifBlue] );
		}

		eNewFormat = ImageOp::RGB8;	
		if ( m_bHasAlpha ) {
			eNewFormat = ImageOp::RGBA8;							
			m_BitsPerChannel[TifAlpha] = m_Buf.getUShort();	// Get bits for alpha channel
			if ( m_DebugTif ) dbgprintf ( "BPC Alpha: %d\n", m_BitsPerChannel[TifAlpha] );
		} else {
			m_BitsPerChannel[TifAlpha] = 0;		// No alpha channel.
		}

		// Create ImageX to load data into
		CreateImage ( m_pNewImage, m_Xres, m_Yres, eNewFormat);		// Create RGBA Image		
		
		m_Buf.setPos (pos);			// Calculate bits per pixel and bytes per row
		m_BitsPerPixel = m_BitsPerChannel[TifRed] + m_BitsPerChannel[TifGreen] + m_BitsPerChannel[TifBlue] + m_BitsPerChannel[TifAlpha];
		m_BytesPerRow = (int) floor ( (m_Xres * m_BitsPerPixel) / 8.0);
	} break;
	case TifGrayscale: {
		
		m_BitsPerPixel = m_BitsPerChannel[TifGray];
		m_BytesPerRow = (int) floor ( (m_Xres * m_BitsPerPixel) / 8.0);

		if ( m_BitsPerChannel[TifGray] == 16 ) {
			eNewFormat = ImageOp::BW16;
		} else {
			eNewFormat = ImageOp::BW8;
		}		
		// Create ImageX to load data into
		if ( m_pOrigImage != 0x0 ) {		// Can we use existing image structure? (faster for movies)
			if ( GetWidth(m_pOrigImage)==m_Xres && GetHeight(m_pOrigImage)==m_Yres && GetFormat(m_pOrigImage) == eNewFormat )
				m_pNewImage = m_pOrigImage;
			else
				CreateImage ( m_pNewImage, m_Xres, m_Yres, eNewFormat );	// Create Grayscale Image
		} else {		
			CreateImage ( m_pNewImage, m_Xres, m_Yres, eNewFormat);		// Create Grayscale Image
		}		
	} break;	
	}

	// Load actual TIFF Image Data into Image
	LoadTiffStrips ();

	// Check if there are multiple images present in TIFF file	
	offset = m_Buf.getULong();
	if (offset!=0) {
		// Warning only.
		if ( m_DebugTif ) dbgprintf ( "ImageFormatTiff::LoadTiff: (Warning only) File contains multiple tiff images - reading only first.\n");
	}
	
	// Success
	m_eStatus = ImageOp::Successs;
	return true;
}

bool CImageFormatTiff::LoadTiffEntry ()
{
	unsigned int tag, typ;
	unsigned long int count, offset;	
		
	// Read Entry Tag (WIDTH, HEIGHT, EXTRASAMPLES, BITSPERSAMPLE, COMPRESSION, etc.)
	tag = m_Buf.getUShort();
	// Read Entry Type and Count (SHORT or LONG)
	typ = m_Buf.getUShort();
	count = m_Buf.getULong();
	if (typ == (int) TifShort && count==1) {
		offset = m_Buf.getUShort();
		m_Buf.getUShort();
	} else {
		offset = m_Buf.getULong();
	}
	// DEBUG OUTPUT
	// printf ("tag:%u type:%u count:%lu offset:%lu\n", tag, typ, count, offset);
	
	// Add information to ImageFormat info based on Entry Tag
	switch (tag) {	
	case TifImageWidth:		
		m_Xres = offset;			
		if (m_DebugTif) dbgprintf ( "XRes: %d\n", m_Xres );		
		break;
	case TifImageHeight:		
		m_Yres = offset;
		if (m_DebugTif) dbgprintf ( "YRes: %d\n", m_Yres );		
		break;
	case TifExtraSamples:		
		m_bHasAlpha = true;		
		break;
	case TifBitsPerSample:
		// NOTE: The TIFF specification defines 'samples',
		// while the Image class defines 'channels'.
		// Thus, bits-per-sample (in TIFF) is the same as bits-per-channel
		// And, samples-per-pixel (in TIFF) is the same as channels-per-pixel
		
		// Detemine samples-per-pixel here 
		// (this is just "count" of how many bits-per-sample 
		//  entries are present)		
		m_NumChannels = count;
		switch (count) {		
		case 1: {			// One channel: B&W or GRAYSCALE
			m_BitsPerChannel[TifGray] = offset;
			m_BitsPerChannelOffset = 0;
			if (offset==1)	m_eMode = TifBw;
			else			m_eMode = TifGrayscale;
		} break;
		case 2: {			// Two channels: GRAYSCALE with ALPHA
			m_BitsPerChannel[TifGray] = offset;
			m_BitsPerChannelOffset = offset;
			m_eMode = TifGrayscale;
		} break;		
		case 3:				// Three channels: RGB
		case 4: {			// Four channels: RGB with ALPHA
			m_BitsPerChannelOffset = offset;	
			if (m_DebugTif) dbgprintf ( "BPC Offset: %d\n", m_BitsPerChannelOffset );			
			m_eMode = TifColor;
		} break;
		default: {
			m_eTiffStatus = TifUnknownTiffMode;
			m_eStatus = ImageOp::FeatureNotSupported;
			return false;
		} break;
		}
		break;
	case TifCompression:
		if (offset == (int) TifCompNone) {		// No compression
			m_eCompression = TifCompNone;
		} else {											// LZW compression
			m_eCompression = TifCompLzw;	
		}
		break;	
	case TifPhotometric:
		if (offset == (int) TifPalette)
			m_eMode = TifIndex;		
		break;
	case TifStripOffsets:
		m_StripOffsets = offset; 
		if (m_DebugTif) dbgprintf ( "Pos Offsets: %d\n", m_StripOffsets );		
		break;
	case TifRowsPerStrip:
		m_RowsPerStrip = offset;
		m_NumStrips = (unsigned long) int ((m_Yres + m_RowsPerStrip - 1) / m_RowsPerStrip);		
		if (m_DebugTif) dbgprintf ( "Rows/Strip:  %d\n", m_RowsPerStrip );
		if (m_DebugTif) dbgprintf ( "Num Strips:  %d\n", m_NumStrips );		
		break;
	case TifStripByteCounts:
		m_StripCounts = offset; 		
		if (m_DebugTif) dbgprintf ( "Strip Counts:  %d\n", m_StripCounts );		
		break;
	case TifPlanarConfiguration:
		m_PlanarConfig = offset;
		break;
	}
	return true;
}

void CImageFormatTiff::LoadTiffStrips ()
{	
	unsigned long pos, row;	
	unsigned long pos_offsets, pos_counts;		// These are not pointers, must increment by 4!
	unsigned long count, offset, strip;
	
	pos = m_Buf.getPosInt();
	pos_counts = m_StripCounts;
	pos_offsets = m_StripOffsets;
	if (m_NumStrips==1) {		
		count = pos_counts;
		offset = pos_offsets;
		row = 0;			
		LoadTiffData (count, offset, row);
	} else {		
		row = 0;		
		for (strip=0; strip < m_NumStrips; strip++) {
			m_Buf.setPos (pos_counts);
			count =		m_Buf.getUShort();
			offset =	m_Buf.getULong();

			LoadTiffData (count, offset, row);
			
			row += m_RowsPerStrip;
			pos_counts += 4;						// These are not pointers, must increment by 4!
			pos_offsets += 4;
		}
	}				
	m_Buf.setPos (pos);
}

void CImageFormatTiff::LoadTiffData (unsigned long count, unsigned long offset, int row)
{
	char in[TIFF_BUFFER];
	char *pData;
	int x, y, lastrow;	

	pData = (char*) GetData ( m_pNewImage ) + row * GetBytesPerRow ( m_pNewImage );
	
	if (m_DebugTif) dbgprintf ( "Data %d: pos %d, num %d\n", row, offset, count );	

	m_Buf.setPos (offset);						// Set file position to TIFF data
	lastrow = row + m_RowsPerStrip - 1;
	if (lastrow > m_Yres-1) lastrow = m_Yres-1;		// Determine last row to process
	
	switch (m_eMode) {
	case TifBw: {							// Black & White TIFF		
		char* pIn;
		
		dbgprintf ( "ERROR: B&W TIFF NOT SUPPORTED. See code.\n" );
		exit(-18);

		for (y = row; y <= lastrow; y++) {			// Assume 1 Bit Per Pixel
			m_Buf.getBuf (in, m_BytesPerRow);					// Read row in TIFF
			
			//code.UnstuffEachBit (in, out, m_BytesPerRow);	// Unstuff row from bits to bytes
			//code.Scale (out, m_BytesPerRow*8, 255);			// Rescale row from 0-1 to 0-255
			
			//if (m_ePhoto == TifWhiteZero)		// Invert if photometrically stored
			//				code.Invert (out, m_BytesPerRow*8);				
			
			pIn = in;
			for (x=0; x < m_Xres; x++) {
				*pData++ = (XBYTE) *pIn;				// Copy Black/White byte data 
				*pData++ = (XBYTE) *pIn;				// into Red, Green and Blue bytes.
				*pData++ = (XBYTE) *pIn++;
			}
		}		
	} break;
	case TifGrayscale: {						// Grayscale TIFF
		switch (m_BitsPerChannel[TifGray]) {
		case 8: case 16: case 32: {									// 8 Bits per Channel				
			for (y = row; y <= lastrow; y++) {
				m_Buf.getBuf ( pData, m_BytesPerRow );				
				pData += m_BytesPerRow;
			}
		} break;
		}
	} break;
	case TifColor: {									// Full Color TIFF
		switch (m_BitsPerChannel[TifRed]) {					
		case 8: {											// 8 Bits per Channel
			int bpr = GetBytesPerPixel( m_pNewImage ) * m_Xres;
			for (y = row; y <= lastrow; y++) {
				m_Buf.getBuf (pData, bpr);
				pData += GetBytesPerRow( m_pNewImage );
			}
			//tiff.Read (m_BytesPerRow * m_RowsPerStrip, pData);			
		} break;
		case 16: {											// 16 Bits per Channel
			XBYTE2 *pIn;
			if (m_bHasAlpha) {								// Alpha included.										
				for (y = row; y <= lastrow;	y++) {
					m_Buf.getBuf (in, m_BytesPerRow);
					pIn = (XBYTE2 *) in;
					for (x=0; x < m_Xres; x++) {
						*pData++ = (XBYTE) (*pIn++ >> 8); 
						*pData++ = (XBYTE) (*pIn++ >> 8);
						*pData++ = (XBYTE) (*pIn++ >> 8); 
						*pData++ = (XBYTE) (*pIn++ >> 8);
					}
				}
			} else {
				for (y = row; y <= lastrow; y++) {			// No Alpha.
					m_Buf.getBuf (in, m_BytesPerRow);
					pIn = (XBYTE2 *) in;
					for (x=0; x < m_Xres; x++) {
						*pData++ = (XBYTE) (*pIn++ >> 8); 
						*pData++ = (XBYTE) (*pIn++ >> 8);
						*pData++ = (XBYTE) (*pIn++ >> 8);					
					}
				}
			}
		} break;
		case 32: {									// 32 Bits per Channel
 			XBYTE4 *pIn;
			if (m_bHasAlpha) {				
				for (y = row; y <= lastrow;	y++) {
					m_Buf.getBuf (in, m_BytesPerRow);
					pIn = (XBYTE4 *) in;
					for (x=0; x < m_Xres; x++) {
						*pData++ = (XBYTE) (*pIn++ >> 24); 
						*pData++ = (XBYTE) (*pIn++ >> 24);
						*pData++ = (XBYTE) (*pIn++ >> 24); 
						*pData++ = (XBYTE) (*pIn++ >> 24);
					}
				}
			} else {
				
				for (y = row; y <= lastrow; y++) {
					m_Buf.getBuf (in, m_BytesPerRow);
					pIn = (XBYTE4 *) in;
					for (x=0; x < m_Xres; x++) {
						*pData++ = (XBYTE) (*pIn++ >> 24); 
						*pData++ = (XBYTE) (*pIn++ >> 24);
						*pData++ = (XBYTE) (*pIn++ >> 24);					
					}
				}
			}
		} break;
		}
	} break;
	}
}


bool CImageFormatTiff::SaveTiffData ()
{	
	uchar* pData;
	int y;

	pData = GetData( m_pOrigImage );						// Get ImageX color data	

	if (m_DebugTif) dbgprintf ( "Data Start: pos: %ld\n", m_Buf.getPosInt () );	
	// DEBUG OUTPUT
	// printf ("Data pos: %lu\n", tiff.GetPosition ());
	// printf ("Est. pos: %lu\n", TIFF_SAVE_POSDATA);	

	switch (m_eMode) {
	case TifGrayscale: {
		switch (m_BitsPerChannel[TifGray]) {
		case 16: {									// 16 Bits per Channel
			XBYTE2 out[TIFF_BUFFER];						
			for (y=0; y < m_Yres; y++) {				
				memcpy ( out, pData, m_BytesPerRow );				
				m_Buf.attachBuf ( (char*) out, m_BytesPerRow );
				pData += m_BytesPerRow;				
			}			
		} break;		
		case 32: {									// 32 Bits per Channel
			XBYTE4 out[TIFF_BUFFER];						
			for (y=0; y < m_Yres; y++) {				
				memcpy ( out, pData, m_BytesPerRow );				
				m_Buf.attachBuf ( (char*) out, m_BytesPerRow );
				pData += m_BytesPerRow;				
			}			
		} break;		
		}
		} break;
	case TifColor: {								// Save Full Color TIFF		
		switch (m_BitsPerChannel[TifRed]) {
		case 8: {									// 8 Bits per Channel
			XBYTE out[TIFF_BUFFER];			
			for (y=0; y < m_Yres; y++) {								
				memcpy ( out, pData, m_BytesPerRow );				
				m_Buf.attachBuf ( (char*) out, m_BytesPerRow );
				pData += m_BytesPerRow;
			}			
		} break;	
		case 16: {
			XBYTE2 out[TIFF_BUFFER];
			for (y=0; y < m_Yres; y++) {								
				memcpy ( out, pData, m_BytesPerRow );				
				m_Buf.attachBuf ( (char*) out, m_BytesPerRow );
				pData += m_BytesPerRow;
			}
		} break;
		default: {		
			printf ("SaveTiff: Given pixel depth and color mode not supported.\n");
			exit(-8);		
		} break;
		}
		} break;
	}
	return true;
}

bool CImageFormatTiff::SaveTiffEntry (enum TiffTag eTag)
{
	enum TiffType eType;	
	unsigned long int iCount, iOffset;	
		
	switch (eTag) {	
	case TifNewSubType: {
		eType = TifLong;
		iCount = 1; 
		iOffset = 0;
	} break;
	case TifImageWidth: {
		eType = TifShort;
		iCount = 1;
		iOffset = m_Xres;
		if (m_DebugTif) dbgprintf ( "XRes: %d\n", m_Xres );
	} break;
	case TifImageHeight: {
		eType = TifShort;
		iCount = 1;
		iOffset = m_Yres;
		if (m_DebugTif) dbgprintf ( "YRes: %d\n", m_Yres );
	} break;		
	case TifBitsPerSample: {
		// NOTE: The TIFF specification defines 'samples',
		// while the Image class defines 'channels'.
		// Thus, bits-per-sample (in TIFF) is the same as bits-per-channel
		// And, samples-per-pixel (in TIFF) is the same as channels-per-pixel
		
		switch (m_eMode) {
		case TifBw: {
			eType = TifShort;
			iCount = 1; 
			iOffset = 1;
		} break;
		case TifGrayscale: {
			eType = TifShort;
			iCount = 1;
			iOffset = m_BitsPerPixel;
		} break;
		case TifColor: {
			eType = TifShort;
			iCount = 3;
			iOffset = TIFF_SAVE_POSBPC;
		} break;
		default: {
			dbgprintf ("Save:Tiff: Unknown TIFF mode.\n");
			exit(-8);
		} break;
		}		
		m_NumChannels = iCount;
	} break;
	case TifCompression: {
		eType = TifShort;
		iCount = 1; 
		iOffset = (int) TifCompNone;
	} break;
	case TifPhotometric: {
		eType = TifShort;
		iCount = 1; 
		switch (m_eMode) {
		case TifBw: iOffset = (int) TifBlackZero; break;
		case TifGrayscale: iOffset = (int) TifBlackZero; break;
		case TifColor: iOffset = (int) TifRgb; break;
		case TifIndex: iOffset = (int) TifPalette; break;
		}
		m_ePhoto = (enum TiffPhotometric) iOffset;
	} break;
	case TifStripOffsets: {
		eType = TifLong;
		iCount = m_Yres;							// how many in list
		iOffset = TIFF_SAVE_POSOFFSETS + m_Yres*4;	// where is the list
		if (m_DebugTif) dbgprintf ( "Num Strips:  %d\n", iCount );
		if (m_DebugTif) dbgprintf ( "Pos Offsets: %d\n", iOffset );
	} break;		
	case TifSamplesPerPixel: {
		eType = TifShort;
		iCount = 1;
		iOffset = m_NumChannels;
	} break;
	case TifRowsPerStrip: {
		eType = TifShort;
		iCount = 1;
		iOffset = 1;
		m_RowsPerStrip = 1;
		m_NumStrips = m_Yres;
	} break;
	case TifStripByteCounts: {
		eType = TifLong;
		iCount = m_Yres;					// how many in list 
		iOffset = TIFF_SAVE_POSOFFSETS;		// where is the list
		switch (m_eMode) {
		case TifBw: m_BytesPerRow = (int) floor ((m_Xres / 8.0) + 1.0); break;
		case TifGrayscale: m_BytesPerRow = (int) floor ((m_Xres * m_BitsPerPixel) / 8.0); break;
		case TifColor: m_BytesPerRow = (int) floor ((m_Xres * m_BitsPerPixel) / 8.0); break;
		}		
		if (m_DebugTif) dbgprintf ( "Pos Counts:   %d\n", iOffset );
		//m_StripCounts = m_BytesPerRow;
	} break;
	case TifXres: {
		eType = TifRational;
		iCount = 1;
		iOffset = 0;
	} break;
	case TifYres: {
		eType = TifRational; 
		iCount = 1; 
		iOffset = 0;
	} break;
	case TifResUnit: {
		eType = TifShort;
		iCount = 1;
		iOffset = 2;
	} break;
	case TifColorMap: {
		eType = TifShort;
		iCount = 0; 
		iOffset = 0;
	} break;
	case TifPlanarConfiguration: {
		eType = TifShort;
		iCount = 1;
		iOffset = 1;
	} break;
	}

	m_Buf.attachUShort( eTag );
	m_Buf.attachUShort( eType );	
	m_Buf.attachULong ( iCount );
	
	if (eType==TifShort && iCount==1) {		
		m_Buf.attachUShort ( iOffset );		
		m_Buf.attachUShort ( 0 );
	} else {
		m_Buf.attachULong ( iOffset );
	}
	// DEBUG OUTPUT
	// printf ("-- tag:%u type:%u count:%lu offset:%lu\n", tag, typ, count, offset);
	return true;
}

bool CImageFormatTiff::SaveTiffExtras (enum TiffTag eTag)
{
	switch (eTag) {
	case TifBitsPerSample: {
		// DEBUG OUTPUT
		// printf ("BPC pos: %u\n", tiff.GetPosition());
		// printf ("Est.BPC pos: %u\n", TIFF_SAVE_POSBPC);		
		int bpc = (m_BitsPerPixel==24) ? 8 : 16;
		m_Buf.attachUShort ( bpc );
		m_Buf.attachUShort ( bpc );
		m_Buf.attachUShort ( bpc );
	} break;	
	case TifXres: {
		// DEBUG OUTPUT
		// printf ("Xres pos: %u\n", tiff.GetPosition());
		// printf ("Est.Xres pos: %u\n", TIFF_SAVE_POSXRES);		
		m_Buf.attachULong ( 1 );
		m_Buf.attachULong ( 1 );
	} break;
	case TifYres: {
		// DEBUG OUTPUT
		// printf ("Yres pos: %u\n", tiff.GetPosition());
		// printf ("Est.Yres pos: %u\n", TIFF_SAVE_POSYRES);		
		m_Buf.attachULong ( 1 );
		m_Buf.attachULong ( 1 );
	} break;
	}
	return true;
}

bool CImageFormatTiff::SaveTiffDirectory ()
{
	// *NOTE*: TIFF Spec 6.0 appears to be incorrect. 
	// The maximum entry number should be written (base 0). NOT the number of entries.
	//
	m_Buf.attachUShort ( TIFF_SAVE_ENTRIES-1 );

	switch (m_eMode) {
	case TifBw: 
		m_BitsPerChannel[TifGray] = 1; 
		break;
	case TifGrayscale: 
		m_BitsPerChannel[TifGray] = m_BitsPerPixel; 
		m_BytesPerRow = (int) floor ((m_Xres * m_BitsPerPixel) / 8.0); 
		break;
	case TifColor: {
		int bpc = (m_BitsPerPixel==24) ? 8 : 16;
		m_BitsPerChannel[TifRed] = bpc;
		m_BitsPerChannel[TifGreen] = bpc;
		m_BitsPerChannel[TifBlue] = bpc;
		m_BytesPerRow = (int) floor ((m_Xres * m_BitsPerPixel) / 8.0); 
	} break;
	}
	
	SaveTiffEntry (TifImageWidth);
	SaveTiffEntry (TifImageHeight);
	SaveTiffEntry (TifBitsPerSample);
	SaveTiffEntry (TifCompression);
	SaveTiffEntry (TifPhotometric);
	SaveTiffEntry (TifStripOffsets);
	SaveTiffEntry (TifSamplesPerPixel);	
	SaveTiffEntry (TifRowsPerStrip);
	SaveTiffEntry (TifStripByteCounts);	
	SaveTiffEntry (TifPlanarConfiguration);	

	m_Buf.attachULong ( 0 );

	SaveTiffExtras (TifBitsPerSample);

	// Strip counts -- List of # of bytes in each strip. 2-byte list
	for (int n=0; n < m_Yres; n++) {
		if (m_DebugTif) dbgprintf ( "Counts #%d: pos: %d, val: %d\n", n, m_Buf.getPos(), m_BytesPerRow );
		m_Buf.attachUShort ( m_BytesPerRow );		
		m_Buf.attachUShort ( 0 );
	}
	
	// Strip offsets -- List of strip offsets. 4-byte list
	for (int n=0; n < m_Yres; n++) {
		if (m_DebugTif) dbgprintf ( "Offset #%d: pos: %d, val: %d\n", n, m_Buf.getPos(), TIFF_SAVE_POSOFFSETS + m_Yres*4*2 + n * m_BytesPerRow);
		m_Buf.attachULong ( TIFF_SAVE_POSOFFSETS + m_Yres*4*2 + n * m_BytesPerRow );
	}

	SaveTiffData ();

	return true;
}

// Function: SaveTiff
//
// Input:
//		m_filename			Name of file to save
//		m_mode				Format mode (BW, GRAY, RGB, INDEX)
//		m_compress			Compression mode
//		m_bpp				Bits per pixel
// Output:
//		m_status
//
// Example #1 - 160 x 120.. Saved /w Photoshop
//   TIF: XRes: 160
//   TIF: YRes: 120
//   TIF: BPC Offset: 242
//   TIF: Pos Offsets: 7178
//   TIF: Rows/Strip:  120
//   TIF: Num Strips:  1
//   TIF: Pos Counts:  57600
//   TIF: BPC Red:   8
//   TIF: BPC Green: 8
//   TIF: BPC Blue:  8
//   TIF: Data 0: pos 7178, num 57600
//
// Example #2 - 160 x 120.. Saved with this class
//
//   TIF: XRes: 160
//   TIF: YRes: 120
//   TIF: BPC Offset: 134
//   TIF: Pos Offsets: 620
//   TIF: Rows/Strip:  1
//   TIF: Num Strips:  120
//   TIF: Pos Counts:  140
//   TIF: BPC Red:   8
//   TIF: BPC Green: 8
//   TIF: BPC Blue:  8
//

bool CImageFormatTiff::SaveTiff (char *filename)
{
	m_Tif = fopen ( filename, "wb" );
	if ( m_Tif == 0x0 ) { 
		dbgprintf ( "ERROR: Unable to create TIF file %s\n", filename );
		return false;
	}
	//m_Buf.Reset ();
	
	switch ( GetFormat( m_pOrigImage) ) {
	case ImageOp::RGB8: case ImageOp::RGBA8: case ImageOp::RGBA32F: case ImageOp::RGB16: 
		m_eMode = TifColor;
		break;
	case ImageOp::BW8: case ImageOp::BW16: case ImageOp::BW32: case ImageOp::F32:
		m_eMode = TifGrayscale;
		break;
	}
	m_Xres = GetWidth( m_pOrigImage );
	m_Yres = GetHeight( m_pOrigImage );
	m_BitsPerPixel = GetBitsPerPixel ( m_pOrigImage );

	m_Buf.attachUShort ( TIFF_BYTEORDER );
	m_Buf.attachUShort  ( TIFF_MAGIC );
	m_Buf.attachULong ( TIFF_SAVE_POSIFD );
	
	SaveTiffDirectory ();			// Write IFD
	
	// write buffer to file
	fwrite ( m_Buf.getData (), 1, m_Buf.getDataLength(), m_Tif );

	fclose ( m_Tif );
	m_eTiffStatus = TifOk;

	return true;
}



