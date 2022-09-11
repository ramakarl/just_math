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
// BMP Format
//
// NOTE: CURRENT CAPABILITIES:
//		- Load supports:
//					 8 bit indexed color    (no alpha)
//					16 bit X1-R5-G5-B5 (without alpha)
//					24 bit    R8-G8-B8 (without alpha)
//					32 bit A8-R8-G8-B8 (with alpha)
//		- Save supports:
//					16 bit X1-R5-G5-B5 (without alpha)
//					32 bit A8-R8-G8-B8 (with alpha)
//
//**************************************

#ifdef BUILD_BMP

#include "imageformat_bmp.h"


bool CImageFormatBmp::Load ( char *filename, Image* pImg )
{
	StartLoad ( filename, pImg );
	bool result = LoadBmp ( filename );	
	if (result) FinishLoad ();
	return result;
}

bool CImageFormatBmp::Save (char *filename, Image*  pImg)
{
	m_pOrigImage = pImg;
	m_pNewImage = 0x0;
	m_eStatus = ImageOp::Saving;
	strcpy (m_Filename, filename);
	return SaveBmp (filename);	
}

// Read BMP palette
int CImageFormatBmp::readPaletteBMP ( FILE* fp, RGBQUAD*& palette, int bit_count )
{
	bool bColor;
	int palSize = 0;
	switch ( bit_count ) {
	case 1:	palSize = 2;	break;
	case 4: palSize = 16;	break;
	case 8: palSize = 256;	break;
	}
	if ( palSize != 0 ) {
		palette = new RGBQUAD[ palSize ];
		memset(palette, 0, palSize * sizeof(RGBQUAD));
		fread ( (void*) palette, sizeof(RGBQUAD), palSize, fp );
		bColor = false;
		for (int i = 0; !bColor && i < 2; ++i) {
			bColor = bColor || (palette[i].rgbRed  != palette[i].rgbGreen) || (palette[i].rgbBlue != palette[i].rgbGreen);
		}
	} else {
		palette = 0x0;
	}
	return palSize;
}


// BMP - Some magic numbers 

#define BMP_BF_TYPE 0x4D42
// word BM 

#define BMP_BF_OFF_BITS 54
// 14 for file header + 40 for info header (not sizeof(), but packed size) 

#define BMP_BI_SIZE 40		
// packed size of info header 


// Reads a WORD from a file in little endian format 
static WORD WordReadLE (FILE* fp)
{
    WORD lsb, msb;

    lsb = fgetc(fp);
    msb = fgetc(fp);
    return (msb << 8) | lsb;
}

// Writes a WORD to a file in little endian format 
static void WordWriteLE(WORD x, FILE* fp)
{
    BYTE lsb, msb;

    lsb = (BYTE) (x & 0x00FF);
    msb = (BYTE) (x >> 8);
    fputc(lsb, fp);
    fputc(msb, fp);
}

// Reads a DWORD word from a file in little endian format
static DWORD DWordReadLE(FILE* fp)
{
    DWORD b1, b2, b3, b4;

    b1 = fgetc(fp);
    b2 = fgetc(fp);
    b3 = fgetc(fp);
    b4 = fgetc(fp);
    return (b4 << 24) | (b3 << 16) | (b2 << 8) | b1;
}

// Writes a DWORD to a file in little endian format 
static void DWordWriteLE(DWORD x, FILE* fp)
{
    unsigned char b1, b2, b3, b4;

    b1 = (x & 0x000000FF);
    b2 = ((x >> 8) & 0x000000FF);
    b3 = ((x >> 16) & 0x000000FF);
    b4 = ((x >> 24) & 0x000000FF);
    fputc(b1, fp);
    fputc(b2, fp);
    fputc(b3, fp);
    fputc(b4, fp);
}

// Reads a LONG word from a file in little endian format 
static LONG LongReadLE(FILE* fp)
{
    LONG b1, b2, b3, b4;

    b1 = fgetc(fp);
    b2 = fgetc(fp);
    b3 = fgetc(fp);
    b4 = fgetc(fp);
    return (b4 << 24) | (b3 << 16) | (b2 << 8) | b1;
}

// Writes a LONG to a file in little endian format 
static void LongWriteLE(LONG x, FILE* fp)
{
    char b1, b2, b3, b4;

    b1 = (x & 0x000000FF);
    b2 = ((x >> 8) & 0x000000FF);
    b3 = ((x >> 16) & 0x000000FF);
    b4 = ((x >> 24) & 0x000000FF);
    fputc(b1, fp);
    fputc(b2, fp);
    fputc(b3, fp);
    fputc(b4, fp);
}


int bitcount (DWORD w)
{
   w = (0x55555555 & w) + (0x55555555 & (w>> 1));
   w = (0x33333333 & w) + (0x33333333 & (w>> 2));
   w = (0x0f0f0f0f & w) + (0x0f0f0f0f & (w>> 4));
   w = (0x00ff00ff & w) + (0x00ff00ff & (w>> 8));
   w = (0x0000ffff & w) + (0x0000ffff & (w>>16));
   return w;
}


bool CImageFormatBmp::LoadBmp ( char* filename )
{
	RGBQUAD*	palette;
	int			palSize;
	int         index;	
	DWORD       bluemask, greenmask, redmask;
	int         bluewidth, greenwidth;
	bool        done;

    BITMAPFILEHEADER bmfh;
    BITMAPINFOHEADER bmih;

	// Open file
	FILE* fp = fopen(filename, "rb");
	if ( !fp ) {
		m_eStatus = ImageOp::FileNotFound;
		return false;
	}

    // Read file header
    bmfh.bfType = WordReadLE(fp);
    bmfh.bfSize = DWordReadLE(fp);
    bmfh.bfReserved1 = WordReadLE(fp);
    bmfh.bfReserved2 = WordReadLE(fp);
    bmfh.bfOffBits = DWordReadLE(fp);   

    // Check file header
    if (bmfh.bfType != BMP_BF_TYPE) {
		m_eStatus = ImageOp::InvalidFile;
		return false;
    }

    // Read info header
    bmih.biSize = DWordReadLE(fp);
    bmih.biWidth = LongReadLE(fp);
    bmih.biHeight = LongReadLE(fp);
    bmih.biPlanes = WordReadLE(fp);
    bmih.biBitCount = WordReadLE(fp);
    bmih.biCompression = DWordReadLE(fp);
    bmih.biSizeImage = DWordReadLE(fp);
    bmih.biXPelsPerMeter = LongReadLE(fp);
    bmih.biYPelsPerMeter = LongReadLE(fp);
    bmih.biClrUsed = DWordReadLE(fp);
    bmih.biClrImportant = DWordReadLE(fp);

    if ((bmih.biWidth <= 0)  || (bmih.biHeight <= 0) || (bmih.biPlanes != 1)) {
		m_eStatus = ImageOp::InvalidFile;
		return false;
    }

    // Get image size
	m_Xres = bmih.biWidth;
	m_Yres = bmih.biHeight;

	// Read palette (if there is one)		
	palSize = readPaletteBMP ( fp, palette, bmih.biBitCount );

	// Determine format
	int channels;
	ImageOp::Format eNewFormat;	

	switch ( bmih.biBitCount ) {
	case 1:		eNewFormat = ImageOp::BW1;	channels = 1;	break;		// 1 byte per bit - *bad storage, 2 color (B & W)
	case 4:		eNewFormat = ImageOp::RGB24;	channels = 1;	break;		// 1 byte per 4-bits - not great, 16 color, index mode
	case 8:		eNewFormat = ImageOp::RGB24;	channels = 1;	break;		// 1 byte per pixel, 256 color, index mode
	case 16:	eNewFormat = ImageOp::RGB24;	channels = 3;	break;		// 2 byte per pixel, 16-bit color
	case 24:	eNewFormat = ImageOp::RGB24;	channels = 3;	break;		// 3 byte per pixel (RGB), 24-bit color 
	case 32:	eNewFormat = ImageOp::RGBA32;	channels = 3;	break;		// 4 byte per pixel, 32-bit color, compressed down to 24-bit
	}

	// Create ImageX to load data into
	CreateImage ( m_pNewImage, m_Xres, m_Yres, eNewFormat );

	// Determine line length
	int scanlinelength;
	if ((m_Xres * bmih.biBitCount) % 32 == 0)	scanlinelength = m_Xres * bmih.biBitCount / 8;
	else										scanlinelength = (m_Xres * bmih.biBitCount / 32 + 1) * 4;

	// Start of pixel data
	int bpr = GetBytesPerPixel( m_pNewImage ) * m_Xres;
	XBYTE* pix = GetData ( m_pNewImage );

	// Read all the color info / data
	switch (bmih.biBitCount) {
	case 1:		// 1 bit - monochrome, index mode
		BYTE* scanlineByte;
		scanlineByte = (BYTE*) malloc ( scanlinelength );
		fseek(fp, bmfh.bfOffBits, SEEK_SET);
		for (int y = 0; y < m_Yres; ++y) {
			fread( (void*) scanlineByte, scanlinelength, 1, fp);
			pix = GetData ( m_pNewImage ) + (m_Yres-1-y) * bpr;
			for (int x = 0; x < m_Xres; ++x) {
				index = (scanlineByte[x/8] >> (7 - (x % 8))) & 0x01;			
				*pix++ = palette[index].rgbRed;
			}
		}
		free ( scanlineByte );
		break;
    case 4:		// 4 bit - 16 color, index mode 		
		if (bmih.biCompression == BI_RGB) {		// 4-bit, uncompressed data
			BYTE* scanlineByte;
			scanlineByte = (BYTE*) malloc (scanlinelength );
			fseek(fp, bmfh.bfOffBits, SEEK_SET);
	        for (int y = 0; y < m_Yres; ++y) {
				fread((void*) scanlineByte, scanlinelength, 1, fp);
				pix = GetData ( m_pNewImage ) + (m_Yres-1-y) * bpr;
				for (int x = 0; x < m_Xres; ++x) {
					if (x % 2 == 0)	index = (scanlineByte[x/2] >> 4) & 0x0F;
					else			index = scanlineByte[x/2] & 0x0F;
					*pix++ = palette[index].rgbRed / 255.0;
				}
			}
			free ( scanlineByte );
		} else if (bmih.biCompression == BI_RLE4) {	// 4-bit RLE compression
			unsigned char rleCode[2];
			int curx = 0;
			int cury = 0;
			done = false;
			fseek(fp, bmfh.bfOffBits, SEEK_SET);
			while (!done && fp) {
				fread((void*) rleCode, sizeof(char), 2, fp);
				if (ferror(fp)) done = true;
				if (rleCode[0] == 0 && rleCode[1] < 3) {	// escape code (next byte is how)
					if (rleCode[1] == 0) {				// code 0 - goto next line
						curx = 0;
						++cury;
						if (cury >= m_Yres) done = true;
					} else if (rleCode[1] == 1) {		// code 1 - finished image
						done = true;
					} else {							// otherwise - two bytes reposition read
						curx += fgetc(fp);
						cury += fgetc(fp);
					}
				} else if (rleCode[0] == 0)	{				// absolute code (next byte is length)
					BYTE byte;
					for (int i = 0; i < (rleCode[1] + 1) / 2; ++i) {
						byte = fgetc(fp);
						index = (byte >> 4) & 0x0F;
						pix = GetData ( m_pNewImage ) + (m_Yres-1-cury) * bpr + curx;							
						*pix++ = palette[index].rgbRed;							
						*pix++ = palette[index].rgbGreen;
						*pix++ = palette[index].rgbBlue;
						++curx;
						index = byte & 0x0F;
						*pix++ = palette[index].rgbRed;							
						*pix++ = palette[index].rgbGreen;
						*pix++ = palette[index].rgbBlue;						
						++curx;
					}
					if (((rleCode[1] + 1) / 2) % 2 != 0)	// byte align
						fgetc(fp);
				} else {
					for (int i = 0; i < rleCode[0]; ++i) {
						if (i % 2 == 0)	index = (rleCode[1] >> 4) & 0x0F;
						else				index = rleCode[1] & 0x0F;

						pix = GetData ( m_pNewImage ) + (m_Yres-1-cury) * bpr + curx;							
						*pix++ = palette[index].rgbRed;							
						*pix++ = palette[index].rgbGreen;
						*pix++ = palette[index].rgbBlue;						
						++curx;
					}
				}
			}
		} // 8-bit compression cases
		break;
	case 8:	// 8 bit - 256 color, index mode
		// read the palette 
		if (bmih.biCompression == BI_RGB) {		// uncompressed data
			BYTE* scanlineByte;
			scanlineByte = (BYTE*) malloc (scanlinelength);
			fseek(fp, bmfh.bfOffBits, SEEK_SET);
			for (int y = 0; y < m_Yres; ++y) {
				fread((void*) scanlineByte, scanlinelength, 1, fp);
				pix = GetData ( m_pNewImage ) + (m_Yres-1-y) * bpr;
				for (int x = 0; x < m_Xres; ++x) {
					index = scanlineByte[x];
					*pix++ = palette[index].rgbRed;							
					*pix++ = palette[index].rgbGreen;
					*pix++ = palette[index].rgbBlue;						
				}
			}
			free ( scanlineByte );
		} else if (bmih.biCompression == BI_RLE8) {	// 8-bit RLE compression
			unsigned char rleCode[2];
			int curx = 0;
			int cury = 0;
			done = false;

			fseek(fp, bmfh.bfOffBits, SEEK_SET);
			while (!done && fp) {
				fread((void*) rleCode, sizeof(char), 2, fp);
				if (ferror(fp)) done = true;
				if (rleCode[0] == 0 && rleCode[1] < 3) {	// escape
					if (rleCode[1] == 0) {
						curx = 0;
						++cury;
						if (cury >= m_Yres) done = true;
					} else if (rleCode[1] == 1) {
						done = true;
					} else {
						curx += fgetc(fp);
						cury += fgetc(fp);
					}
				} else if (rleCode[0] == 0) {			// absolute mode
					for (int i = 0; i < rleCode[1]; ++i) {
						index = fgetc(fp);
						pix = GetData ( m_pNewImage ) + (m_Yres-1-cury) * bpr + curx;
						*pix++ = palette[index].rgbRed;							
						*pix++ = palette[index].rgbGreen;
						*pix++ = palette[index].rgbBlue;						
						++curx;
					}
					if (rleCode[1] % 2 != 0) fgetc(fp);
				} else {		// encoded mode
					pix = GetData ( m_pNewImage ) + (m_Yres-1-cury) * bpr + curx;
					for (int i = 0; i < rleCode[0]; ++i) {
						*pix++ = palette[index].rgbRed;							
						*pix++ = palette[index].rgbGreen;
						*pix++ = palette[index].rgbBlue;
					}
					curx += rleCode[0];
				} 
			}	// while not done
		} // 8-bit compression cases
		break;
    case 16:	// 16 bit - 2^16 color, rgb mode 
		if (bmih.biCompression == BI_BITFIELDS) {	// user specified
			redmask    = DWordReadLE(fp);
			greenmask  = DWordReadLE(fp);
			bluemask   = DWordReadLE(fp);
			bluewidth  = bitcount(bluemask);
			greenwidth = bitcount(greenmask);
		} else { // bmih.biCompression == BI_RGB	// using default values
			bluemask   = 0x001F;
			bluewidth  = 5;
			greenmask  = 0x03E0;
			greenwidth = 5;
			redmask    = 0x7C00;
		}
		WORD* scanlineWord;
		scanlineWord = (WORD*) malloc ( sizeof(WORD)*(scanlinelength+1)/2 );
		fseek(fp, bmfh.bfOffBits, SEEK_SET);

		for (int y = 0; y < m_Yres; ++y) {
			fread( (void*) scanlineWord, scanlinelength, 1, fp);
			pix = GetData ( m_pNewImage ) + (m_Yres-1-y) * bpr;
			for (int x = 0; x < m_Xres; ++x) {				
				*pix++   = (scanlineWord[x] & redmask) >> (bluewidth + greenwidth);
				*pix++ = (scanlineWord[x] & greenmask) >> bluewidth;
				*pix++  = (scanlineWord[x] & bluemask);
			}
		}
		free ( scanlineWord );
		break;
    case 24:	// 24 bit - 2^24 color, rgb mode 
		RGBTRIPLE  *scanline24;
		scanline24 = (RGBTRIPLE*) malloc ( sizeof(RGBTRIPLE)*(scanlinelength+2)/3 );
		fseek(fp, bmfh.bfOffBits, SEEK_SET);

		for (int y = 0; y < m_Yres; ++y) {
			fread((void*) scanline24, scanlinelength, 1, fp);
			pix = GetData ( m_pNewImage ) + (m_Yres-1-y) * bpr;
			for (int x = 0; x < m_Xres; ++x) {
				*pix++ = scanline24[x].rgbtRed;
				*pix++ = scanline24[x].rgbtGreen;
				*pix++ = scanline24[x].rgbtBlue ;
			}
		}
		free ( scanline24 );
		break;
    case 32:	// 32 bit - 2^32 color, rgb mode
		if (bmih.biCompression == BI_RGB) {		// default encoding
			RGBQUAD* scanline32;
			scanline32 = (RGBQUAD*) malloc ( sizeof(RGBQUAD)*(scanlinelength+3)/4 );
			fseek(fp, bmfh.bfOffBits, SEEK_SET);
			for (int y = 0; y < m_Yres; ++y) {
				fread((void*) scanline32, scanlinelength, 1, fp);
				pix = GetData ( m_pNewImage ) + (m_Yres-1-y) * bpr;
				for (int x = 0; x < m_Xres; ++x) {
					*pix++ = scanline32[x].rgbRed;
					*pix++ = scanline32[x].rgbGreen;
					*pix++ = scanline32[x].rgbBlue;
				}
			}
			free ( scanline32 );
		} else if (bmih.biCompression == BI_BITFIELDS) {	// user specified
			// get masks and shifts
			redmask    = DWordReadLE(fp);
			greenmask  = DWordReadLE(fp);
			bluemask   = DWordReadLE(fp);
			bluewidth  = bitcount(bluemask);
			greenwidth = bitcount(greenmask);
			DWORD* scanlineDword;
			scanlineDword = (DWORD*) malloc ( sizeof(DWORD)*(scanlinelength+3)/4 );
			fseek(fp, bmfh.bfOffBits, SEEK_SET);
			
			for (int y = 0; y < m_Yres; ++y) {
				fread((void*) scanlineDword, scanlinelength, 1, fp);
				pix = GetData ( m_pNewImage ) + (m_Yres-1-y) * bpr;
				for (int x = 0; x < m_Xres; ++x) {
					*pix++   = (scanlineDword[x] & redmask) >> (bluewidth + greenwidth);
					*pix++ = (scanlineDword[x] & greenmask) >> bluewidth;
					*pix++ = (scanlineDword[x] & bluemask);
				}
			}
			free ( scanlineDword );
		}
		break;
	};			// close select

	if ( palette != 0x0 ) delete [] palette;
    return true;
}



bool CImageFormatBmp::SaveBmp ( char* filename )
{
	// Open file
	FILE* fp = fopen(filename, "wb");

	m_Xres = GetWidth ( m_pOrigImage );
	m_Yres = GetHeight ( m_pOrigImage );
	XBYTE* pix = GetData ( m_pOrigImage );
	int bpr = GetBytesPerPixel ( m_pOrigImage ) * m_Xres;

	BITMAPFILEHEADER bmfh;
    BITMAPINFOHEADER bmih;
    int lineLength = m_Xres * 3;
    if ((lineLength % 4) != 0) 
      lineLength = (lineLength / 4 + 1) * 4;

    // Write file header 

    bmfh.bfType = BMP_BF_TYPE;
    bmfh.bfSize = BMP_BF_OFF_BITS + lineLength * m_Yres;
    bmfh.bfReserved1 = 0;
    bmfh.bfReserved2 = 0;
    bmfh.bfOffBits = BMP_BF_OFF_BITS;

    //if (channels == 1) bmfh.bfOffBits += 256 * 4;

    WordWriteLE(bmfh.bfType, fp);
    DWordWriteLE(bmfh.bfSize, fp);
    WordWriteLE(bmfh.bfReserved1, fp);
    WordWriteLE(bmfh.bfReserved2, fp);
    DWordWriteLE(bmfh.bfOffBits, fp);

    // Write info header 

    bmih.biSize = BMP_BI_SIZE;
    bmih.biWidth = m_Xres;
    bmih.biHeight = m_Yres;
    bmih.biPlanes = 1;
	bmih.biBitCount = 24;
    //bmih.biBitCount = (channels == 1) ? 8 : 24;
    bmih.biCompression = BI_RGB;
    bmih.biSizeImage = lineLength * (DWORD) bmih.biHeight;
    bmih.biXPelsPerMeter = 2925;
    bmih.biYPelsPerMeter = 2925;
	bmih.biClrUsed = 0;
    //bmih.biClrUsed = (channels == 1) ? 256 : 0;	
    bmih.biClrImportant = 0;

    DWordWriteLE(bmih.biSize, fp);
    LongWriteLE(bmih.biWidth, fp);
    LongWriteLE(bmih.biHeight, fp);
    WordWriteLE(bmih.biPlanes, fp);
    WordWriteLE(bmih.biBitCount, fp);
    DWordWriteLE(bmih.biCompression, fp);
    DWordWriteLE(bmih.biSizeImage, fp);
    LongWriteLE(bmih.biXPelsPerMeter, fp);
    LongWriteLE(bmih.biYPelsPerMeter, fp);
    DWordWriteLE(bmih.biClrUsed, fp);
    DWordWriteLE(bmih.biClrImportant, fp);

    // Write pixels
	for (int y = 0; y < m_Yres; ++y) {        
		pix = GetData ( m_pOrigImage ) + (m_Yres - y - 1) * bpr;
        for (int x = 0; x < m_Xres; ++x) {
			fputc( (int) *pix++, fp);
			fputc( (int) *pix++, fp);
			fputc( (int) *pix++, fp);
        }	
	}

    /*if (channels == 1) {
      // Write 8-bit grayscale palette
      unsigned char palettecolor[4];
      for (int i = 0; i < 256; ++i) {
        memset(palettecolor, (unsigned char) i, 4);
        fwrite((void*) palettecolor, sizeof(char), 4, fp);
      }
      // Write image data
      for (int y = 0; y < height; ++y) {
        int nbytes = 0;
        for (int x = 0; x < width; ++x) {
          getPixel(x, height - y - 1, pixel);
          fputc((int) (pixel.r * 255), fp);
          nbytes++;
        }
        while ((nbytes % 4) != 0) 
          fputc(0, fp);  nbytes++;        
      }
    }*/
    
    return true;
}

#endif


