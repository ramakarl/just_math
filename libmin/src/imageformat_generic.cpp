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

#include <olectl.h>

#include "imageformat_generic.h"

//--------- MUST BE UPDATED
/*
bool CImageFormatGeneric::Load (char *filename, const Image* pOrigImg, Image*& pNewImg)
{
	m_pOrigImage = (Image*) pOrigImg;
	m_pNewImage = 0x0;
	m_eStatus = FormatStatus::Loading;
	strcpy (m_Filename, filename);	
	bool result = LoadGeneric ( filename );
	pNewImg = m_pNewImage;
	return result;	
}

bool CImageFormatGeneric::Save (char *filename, const Image* pOrigImg)
{
	m_pOrigImage = (Image*) pOrigImg;
	m_eStatus = FormatStatus::Saving;
	strcpy (m_Filename, filename);
	return SaveGeneric ( filename );
}

CImageFormatGeneric::CImageFormatGeneric ()
{
}

bool CImageFormatGeneric::LoadGeneric ( char *filename )
{
	IPicture* picture;
	IStream* stream;
	HGLOBAL hGlobal;
	FILE* file;

	// Open file to load
	file = fopen ( filename, "rb" ); // open file in read only mode
	if (file == NULL) {
		m_eStatus = FormatStatus::FileNotFound;
		return false;
	}

	// Allocates global memory same size as file to be loaded
	fseek ( file, 0, SEEK_END);
	int iFileSize = ftell ( file );
	fseek ( file, 0, SEEK_SET);
	hGlobal = GlobalAlloc(GPTR, iFileSize);			
	if ( hGlobal == NULL) {
		fclose ( file );
        m_eStatus = FormatStatus::InvalidFile;
		return false;
	}	
	// Read file data into global memory
	fread ( (void*) hGlobal, 1, iFileSize, file);
	fclose ( file );

	// Create a stream on global memory
	CreateStreamOnHGlobal( hGlobal, false, &stream);
	if ( stream == NULL ) {
		GlobalFree ( hGlobal );
		m_eStatus = FormatStatus::InvalidFile;
		return false;
	}

	// Decompress and load the JPG or GIF into Picture COM object
	OleLoadPicture( stream, 0, false, IID_IPicture, (void**)&picture );
	if ( picture == NULL ) { 
		stream->Release();
		GlobalFree( hGlobal );
		m_eStatus = FormatStatus::InvalidFile;
		return false;
	}

	// Release the stream 
	stream->Release();
	
	// Free the global memory
	GlobalFree( hGlobal );

	// Get a Windows bitmap from the Picture COM object
	HBITMAP hSrcBitmap = 0;
	picture->get_Handle( (unsigned int*)&hSrcBitmap );
	if ( hSrcBitmap == 0 ) {
		m_eStatus = FormatStatus::InvalidFile;
		return false;
	}

	// Copy Windows bitmap into a new one (why?)
	HBITMAP hDestBitmap = (HBITMAP) CopyImage ( hSrcBitmap, IMAGE_BITMAP, 0, 0, LR_COPYRETURNORG);
	if ( hDestBitmap == 0) {
		m_eStatus = FormatStatus::InvalidFile;
		return false;
	}

	// Success up to this point!
	// Release the Picture COM object
	picture->Release();

	// Transfer Windows bitmap into ImageX
	// (note: Format status will be set by TransferBitmap)
	bool bSuccess = TransferBitmap ( hDestBitmap );

	// Delete Windows bitmaps
	if ( hSrcBitmap ) DeleteObject ( hSrcBitmap );
	if ( hDestBitmap ) DeleteObject ( hDestBitmap );

	return bSuccess;
}

bool CImageFormatGeneric::SaveGeneric ( char *filename )
{
	// Not implemented.
	m_eStatus = FormatStatus::NotImplemented;
	return false;
}
*/
