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

#ifndef DEF_IMAGEFORMAT_TIFF
	#define	DEF_IMAGEFORMAT_TIFF

	#include "imageformat.h"
	#include "event.h"

	const int	TIFF_BUFFER =				(32767);
	const int	TIFF_BYTEORDER =			0x4949;
	const int	TIFF_MAGIC =				42;

	// Tiff Save settings
	const int	TIFF_SAVE_ENTRIES =			10;
	const int	TIFF_SAVE_SIZEHEAD =		8;			// 8 bytes
	const int	TIFF_SAVE_SIZEIFD =			(2 + (TIFF_SAVE_ENTRIES*12) + 4);
	const int	TIFF_SAVE_SIZEBPC =			(2*3);		// 3 ints
	
	const int	TIFF_SAVE_POSHEAD =			0;
	const int	TIFF_SAVE_POSIFD =			(TIFF_SAVE_POSHEAD + TIFF_SAVE_SIZEHEAD);
	const int	TIFF_SAVE_POSBPC =			(TIFF_SAVE_POSIFD + TIFF_SAVE_SIZEIFD);
	const int	TIFF_SAVE_POSOFFSETS =		(TIFF_SAVE_POSBPC + TIFF_SAVE_SIZEBPC);		


	class HELPAPI CImageFormatTiff : public CImageFormat {
	public:
		enum TiffStatus {
			TifOk = 0,
			TifNoMagic = 1,
			TifLzwNotSupported = 2,
			TifNonRgbNotSupported = 3,
			TifUnknownTiffMode = 4
		};
		enum TiffTag {
			TifNewSubType = 254,
			TifSubType = 255,
			TifImageWidth = 256,
			TifImageHeight = 257,
			TifBitsPerSample = 258,
			TifCompression = 259,
			TifPhotometric = 262,
			TifThresholding = 263,
			TifDocname = 269,
			TifStripOffsets = 273,
			TifOrientation = 274,
			TifSamplesPerPixel = 277,
			TifRowsPerStrip = 278,
			TifStripByteCounts = 279,
			TifXres = 282,
			TifYres = 283,
			TifPlanarConfiguration = 284,
			TifResUnit = 296,
			TifColorMap = 320,
			TifExtraSamples = 338		
		};
		enum TiffCompression {
			TifCompNone = 1,
			TifCompLzw = 5,
			TifCompPackbits = 32773
		};
		enum TiffPhotometric {
			TifWhiteZero = 0,			
			TifBlackZero = 1,
			TifRgb = 2,
			TifPalette = 3,
			TifTrans = 4,
			TifCmyk = 5
		};
		enum TiffType {
            TifByte = 1,
			TifAscii = 2,
			TifShort = 3,
			TifLong = 4,
			TifRational = 5			
		};
		enum TiffMode {
			TifBw = 0,
			TifGrayscale = 1,
			TifColor = 2,
			TifIndex = 3
		};
		enum TiffChannel {
			TifGray = 0,
			TifRed = 1,
			TifGreen = 2,
			TifBlue = 3,
			TifAlpha = 4
		};

		CImageFormatTiff ();

		virtual bool Load (char *filename, Image* pImg );
		virtual bool Save (char *filename, Image* pImg );	
		void DebugTif ( bool v )	{ m_DebugTif = v; }
		
	private:
		bool LoadTiff (char *filename);
		bool LoadTiffDirectory ();
		bool LoadTiffEntry ();		
		void LoadTiffStrips ();
		void LoadTiffData (unsigned long count, unsigned long offset, int row);
		
		bool SaveTiff (char *filename);
		bool SaveTiffDirectory ();
		bool SaveTiffExtras (enum TiffTag eTag);
		bool SaveTiffEntry ( enum TiffTag eTag);
		bool SaveTiffData ();

		void ComputeMinMaxData (unsigned long count, unsigned long offset, int row);
		void ComputeMinMax ();

		FILE*				m_Tif;
		Event				m_Buf;

		TiffStatus			m_eTiffStatus;
		TiffTag				m_eTag;
		TiffCompression		m_eCompression;
		TiffPhotometric		m_ePhoto;
		TiffType			m_eType;
		TiffMode			m_eMode;
		
		bool				m_bHasAlpha;
		int					m_NumChannels;
        unsigned long		m_BitsPerChannelOffset;
		int					m_BitsPerChannel[5];
		unsigned long		m_NumStrips;
		unsigned long		m_StripOffsets;
		unsigned long		m_StripCounts;
		int					m_RowsPerStrip;		
		int					m_PlanarConfig;
		bool				m_DebugTif;

		XBYTE4				m_Min, m_Max;
	};

#endif



