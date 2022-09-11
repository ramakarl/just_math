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
#ifndef DEF_IMAGEFORMAT_TGA
	#define	DEF_IMAGEFORMAT_TGA

	#include "imageformat.h"

	class HELPAPI CImageFormatTga : public CImageFormat {
	public:		
		virtual bool Load (char *filename, Image* pImg );
		virtual bool Save (char *filename, Image* pImg );
	
	private:
		unsigned char *getRGBA(FILE *s, unsigned char* rgba, int size);
		unsigned char *getRGB(FILE *s, unsigned char* rgb, int size);
		unsigned char *getGray(FILE *s, unsigned char* grayData, int size);
		void           writeRGBA(FILE *s, const unsigned char *externalImage, int size);
		void           writeRGB(FILE *s, const unsigned char *externalImage, int size);
		void           writeGrayAsRGB(FILE *s, const unsigned char *externalImage, int size);
		void           writeGray(FILE *s, const unsigned char *externalImage, int size);

		bool LoadTga (char *filename);
		//bool SaveTga (char *filename);		
	};

#endif


