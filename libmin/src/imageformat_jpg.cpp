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

#ifdef BUILD_JPG

//**************************************
//
// JPG Formats 
//
//**************************************

#include <assert.h>
 
#include "imageformat_jpg.h"

#include "libjpg/jpeglib.h"
#include "libjpg/jerror.h"

//************************************ JPEG Error Handling


METHODDEF(void) extended_error_exit(j_common_ptr cinfo)
{
	extended_error_ptr err = (extended_error_ptr)cinfo->err;
	(*cinfo->err->output_message)(cinfo);
	
	longjmp(err->setjmp_buffer, 1);
}

METHODDEF(void) extended_reset_error_mgr(j_common_ptr cinfo)
{
	cinfo->err->num_warnings = 0;
	cinfo->err->msg_code = 0;

	//strError = EMPTY_STRING;
}

METHODDEF(void) extended_output_message(j_common_ptr cinfo)
{
  char buffer[JMSG_LENGTH_MAX];
  (*cinfo->err->format_message)(cinfo, buffer);
  
  //strError = buffer;
}
//*************************************


bool CImageFormatJpg::Load ( char *filename, Image* pImg )
{
	StartLoad ( filename, pImg );
	bool result = LoadJpg ( filename, false );
	if (result) FinishLoad ();
	return result;
}

bool CImageFormatJpg::LoadIncremental ( char* filename, Image* pImg )
{
	StartLoad ( filename, pImg );	
	bool result = LoadJpg (filename, true );
	return result;
}

bool CImageFormatJpg::Save (char *filename, Image* pImg)
{
	m_pOrigImage = pImg;
	m_eStatus = ImageOp::Saving;
#ifdef WIN32
	strcpy_s (m_Filename, FILE_NAMELEN, filename);
#else
	strcpy (m_Filename, filename);
#endif
	return SaveJpg (filename, 95);		// <-- jpeg quality	
}

bool CImageFormatJpg::LoadJpg (char *filename, bool bIncremental)
{
	// Attempt to open file
#ifdef WIN32
	fopen_s ( &m_jpeg_file, filename, "rb");
#else
	m_jpeg_file = fopen ( filename, "rb");
#endif

	// Error opening (does not exist, etc.)
	if ( m_jpeg_file == 0x0 ) {		
		m_eStatus = ImageOp::FileNotFound;
		return false;
	}

	// Error handling
	m_jpeg_dinfo.err = jpeg_std_error (&m_jerr.pub);
	m_jerr.pub.error_exit = extended_error_exit;
	m_jerr.pub.output_message = extended_output_message;
	m_jerr.pub.reset_error_mgr = extended_reset_error_mgr;	
	if (setjmp(m_jerr.setjmp_buffer)) {		
		jpeg_destroy_decompress (&m_jpeg_dinfo);		
		fclose ( m_jpeg_file );
		m_eStatus = ImageOp::InvalidFile;
		return false;
	}

	// Create decompressor
	jpeg_create_decompress (&m_jpeg_dinfo);
	jpeg_stdio_src (&m_jpeg_dinfo, m_jpeg_file);
	jpeg_read_header (&m_jpeg_dinfo, TRUE);

	// Adjust decompression parameters
	m_jpeg_dinfo.out_color_space = JCS_RGB;
	m_jpeg_dinfo.quantize_colors = FALSE;
	m_jpeg_dinfo.dct_method = JDCT_IFAST;
	m_jpeg_dinfo.scale_num = 1;
	m_jpeg_dinfo.scale_denom = 1;
	m_jpeg_dinfo.do_fancy_upsampling = FALSE;
	m_jpeg_dinfo.do_block_smoothing = FALSE;

	// Start decompression
	jpeg_start_decompress (&m_jpeg_dinfo);

	// Determine output parameters
	int nx, ny;
	nx = m_jpeg_dinfo.output_width;
	ny = m_jpeg_dinfo.output_height;
	ImageOp::Format eNewFormat;
	switch ( m_jpeg_dinfo.output_components ) {
	case 1:	eNewFormat = ImageOp::BW8;		break;		
	case 3: eNewFormat = ImageOp::RGB8;		break;
	case 4: eNewFormat = ImageOp::RGB8;		break;	// does not indicate alpha, but baseline-diff color jpg
	default: 
		m_eStatus = ImageOp::DepthNotSupported;
		jpeg_destroy_decompress (&m_jpeg_dinfo);		
		fclose ( m_jpeg_file );
		return false;		
		break;
	}

	// Quantization not yet supported
	if ( m_jpeg_dinfo.quantize_colors == TRUE) {
		m_eStatus = ImageOp::DepthNotSupported;
		jpeg_destroy_decompress (&m_jpeg_dinfo);		
		fclose ( m_jpeg_file );
		return false;		
	}
	
	// Allocate new image space or use existing one
	CreateImage ( m_pNewImage, nx, ny, eNewFormat );

	// Load image now
	if ( !bIncremental ) {		
		// Compute bytes per block for load
		int	bytes_per_block;
		int	result;
		bytes_per_block = m_jpeg_dinfo.output_width * m_jpeg_dinfo.output_components;	// This is bytes per row...
		//bytes_per_block *= jpeg_info.rec_outbuf_height;							// and number of rows per block

		XBYTE *dest = GetData ( m_pNewImage );	

		if (m_jpeg_dinfo.quantize_colors==false) {			
			if (m_jpeg_dinfo.output_components==1) {
				// Load black & white jpeg (8-bit)
				while (m_jpeg_dinfo.output_scanline < m_jpeg_dinfo.output_height) {
					result = jpeg_read_scanlines (&m_jpeg_dinfo, (JSAMPARRAY) &dest, 1);
					assert ( result == 1);
					dest += bytes_per_block;
				}
			} else if (m_jpeg_dinfo.output_components==3) {			
				// Load color jpeg (24-bit)
				while (m_jpeg_dinfo.output_scanline < m_jpeg_dinfo.output_height) {
					result = jpeg_read_scanlines (&m_jpeg_dinfo, (JSAMPARRAY) &dest, 1);
					assert ( result == 1);
					dest += bytes_per_block;
				}
			} else if (m_jpeg_dinfo.output_components==4) {
				// Load color jpeg (32-bit, baseline-difference color jpeg)
				// Make a one-row-high sample array that will go away when done with image 				
				XBYTE *src, *src_stop;
				JSAMPARRAY in_buf;	
				int k;
				in_buf = (*m_jpeg_dinfo.mem->alloc_sarray) ((j_common_ptr) &m_jpeg_dinfo, JPOOL_IMAGE, bytes_per_block, m_jpeg_dinfo.rec_outbuf_height);
				while (m_jpeg_dinfo.output_scanline < m_jpeg_dinfo.output_height) {
					(void) jpeg_read_scanlines (&m_jpeg_dinfo, in_buf, m_jpeg_dinfo.rec_outbuf_height);
					src = (XBYTE*) in_buf[0];
					src_stop = (XBYTE*) in_buf[0] + bytes_per_block;
					for (; src < src_stop;){
						k = *(src+3);
						*dest++ = (k * *(src+2)) >> 8;
						*dest++ = (k * *(src+1)) >> 8;
						*dest++ = (k * *(src+0)) >> 8;
						src+=4;				
					}
				}
			}
		}
		// Finish decompression	
		jpeg_finish_decompress (&m_jpeg_dinfo);
		jpeg_destroy_decompress (&m_jpeg_dinfo);

		// Close image file
		fclose (m_jpeg_file);

	} else {
		// Make sure incremental loader can handle given format
		if (m_jpeg_dinfo.output_components==4) {
			m_eStatus = ImageOp::DepthNotSupported;
			return false;
		}
		m_pDest = GetData ( m_pNewImage );
		m_eStatus = ImageOp::Loading;
	}
	return true;
}

bool CImageFormatJpg::SaveJpg (char *filename, int iQuality)
{
	struct jpeg_compress_struct		jpeg_info;
	struct extended_error_mgr		jerr;
	JSAMPROW						row_pointer[1];
	int								iPitch;

	// Open file for output
	FILE* jpeg_file;
#ifdef WIN32
	fopen_s ( &jpeg_file, filename, "wb" );
#else
	jpeg_file = fopen ( filename, "rb");
#endif
	if ( jpeg_file == NULL) {		
		return false;
	}

	// Create a compressor
	jpeg_create_compress( &jpeg_info);

	// Error handling
	jpeg_info.err = jpeg_std_error (&jerr.pub);
	jerr.pub.error_exit = extended_error_exit;
	jerr.pub.output_message = extended_output_message;
	jerr.pub.reset_error_mgr = extended_reset_error_mgr;	
	if (setjmp(jerr.setjmp_buffer)) {
		jpeg_destroy_compress(&jpeg_info);
		fclose( jpeg_file );
		return false;
	}
	
	jpeg_stdio_dest( &jpeg_info, jpeg_file );
	
	jpeg_info.image_width = GetWidth ( m_pOrigImage );
	jpeg_info.image_height = GetHeight ( m_pOrigImage );
	
	XBYTE* pBuffer = 0x0;

	unsigned long mMin, mMax;
	int adj_size;
	bool bReformatted;
	mMin = ((unsigned long) 1 << 32) - 1;
	mMax = 0;

	pBuffer = GetData ( m_pOrigImage );				// Original data OK to save
	iPitch = GetBytesPerRow ( m_pOrigImage );

	switch ( GetFormat ( m_pOrigImage ) ) {
	case ImageOp::BW8: {
		jpeg_info.input_components = 1;	
		jpeg_info.in_color_space = JCS_GRAYSCALE; 
		adj_size = jpeg_info.image_width * jpeg_info.image_height;				
		bReformatted = false;
		} break;
	case ImageOp::BW16: {
		jpeg_info.input_components = 1;	
		jpeg_info.in_color_space = JCS_GRAYSCALE; 
		adj_size = jpeg_info.image_width * jpeg_info.image_height;		
		iPitch = jpeg_info.image_width;
		
		XBYTE2* pIn = (XBYTE2*) pBuffer;
		pBuffer = (XBYTE*) malloc ( adj_size );		
		XBYTE* pOut = pBuffer;		
		for (int n=0; n < (int) jpeg_info.image_width * jpeg_info.image_height; n++ ) {			
			*pOut++ = (XBYTE) (*pIn >> 8 ); pIn++;
		}
		bReformatted = true;
		} break;
	case ImageOp::BW32: {
		jpeg_info.input_components = 1;	
		jpeg_info.in_color_space = JCS_GRAYSCALE; 	
		adj_size = jpeg_info.image_width * jpeg_info.image_height;		
		iPitch = jpeg_info.image_width;

		XBYTE4* pIn = (XBYTE4*) pBuffer;
		pBuffer = (XBYTE*) malloc ( adj_size );
		XBYTE* pOut = pBuffer;
		for (int n=0; n < (int) jpeg_info.image_width * jpeg_info.image_height; n++ ) {
			*pOut++ = (XBYTE) (*pIn >> 16 ); pIn++;
		}
		bReformatted = true;
		} break;
	case ImageOp::RGB8: {
		jpeg_info.input_components = 3;	
		jpeg_info.in_color_space = JCS_RGB; 	
		adj_size = jpeg_info.image_width * jpeg_info.image_height * 3;		
		bReformatted = false;
		} break;
	case ImageOp::RGBA8: {
		jpeg_info.input_components = 3;			// JPEG cannot save alpha!
		jpeg_info.in_color_space = JCS_RGB; 	
		adj_size = jpeg_info.image_width * jpeg_info.image_height * 3;
		iPitch = jpeg_info.image_width * 3;
		
		XBYTE* pIn = (XBYTE*) pBuffer;
		pBuffer = (XBYTE*) malloc ( adj_size );
		XBYTE* pOut = pBuffer;		
		for (int n=0; n < (int) jpeg_info.image_width * jpeg_info.image_height; n++ ) {
			*pOut++ = *pIn++;
			*pOut++ = *pIn++;
			*pOut++ = *pIn++;			
			pIn++;
		}
		bReformatted = true;
		} break;
	}
	
	
	/*	Color spaces
	JCS_UNKNOWN,		// error/unspecified 
	JCS_GRAYSCALE,		// monochrome 
	JCS_RGB,		// red/green/blue 
	JCS_YCbCr,		// Y/Cb/Cr (also known as YUV) 
	JCS_CMYK,		// C/M/Y/K 
	JCS_YCCK		// Y/Cb/Cr/K 
	*/

	jpeg_set_defaults(&jpeg_info);
	jpeg_set_quality(&jpeg_info, iQuality, TRUE);

	jpeg_start_compress (&jpeg_info, TRUE);

	while (jpeg_info.next_scanline < jpeg_info.image_height) {
		row_pointer[0] = (JSAMPROW) &pBuffer[jpeg_info.next_scanline * iPitch];
		jpeg_write_scanlines( &jpeg_info, row_pointer, 1);
	}

	if ( bReformatted ) 
		free (pBuffer);

	jpeg_finish_compress(&jpeg_info);
	fclose (jpeg_file);
	jpeg_destroy_compress(&jpeg_info);

	return true;
}

ImageOp::FormatStatus CImageFormatJpg::LoadNextRow ()
{
	if (m_eStatus!=ImageOp::Loading) return ImageOp::LoadNotReady;

	// Load next row
	jpeg_read_scanlines (&m_jpeg_dinfo, (JSAMPARRAY) &m_pDest, 1);
	m_pDest += m_jpeg_dinfo.output_width * m_jpeg_dinfo.output_components;

	if ( m_jpeg_dinfo.output_scanline >= m_jpeg_dinfo.output_height) {
		// Done
		jpeg_finish_decompress (&m_jpeg_dinfo);
 		jpeg_destroy_decompress (&m_jpeg_dinfo);
		fclose (m_jpeg_file);
		FinishLoad ();
		return ImageOp::LoadDone;
	} else {
		return ImageOp::LoadOk;
	}		
}



/*BOOL WriteBitmapIntoJpegFile(const CString& strOutFileName, const int nQuality, HBITMAP hBitmap) 
{
	CSTScreenBuffer screenBuffer;
	screenBuffer.Create(hBitmap);

	BYTE* pBitmapBytes = new BYTE[screenBuffer.GetWidth() * screenBuffer.GetHeight() * 3];

	int i = 0;
	for (int nY = 0; nY < screenBuffer.GetHeight(); nY++) {
		for (int nX = 0; nX < screenBuffer.GetWidth(); nX++) {
			BGRColor color = screenBuffer.GetPoint(nX, nY);
			pBitmapBytes[i++] = color.m_R;
			pBitmapBytes[i++] = color.m_G;
			pBitmapBytes[i++] = color.m_B;
		}
	}
	BOOL res = WriteRGBBytesIntoJpegFile(strOutFileName, screenBuffer.GetWidth(), screenBuffer.GetHeight(), nQuality, pBitmapBytes);

	delete[] pBitmapBytes;

	return res;
}*/

	
	/*

struct my_error_mgr {
  struct jpeg_error_mgr pub;			// "public" fields 
  jmp_buf setjmp_buffer;				// for return to caller 
};
typedef struct my_error_mgr * my_error_ptr;

// Here's the routine that will replace the standard error_exit method:
 
METHODDEF(void) my_error_exit (j_common_ptr cinfo)
{
  // cinfo->err really points to a my_error_mgr struct, so coerce pointer 
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  // Always display the message. 
  // We could postpone this until after returning, if we chose. 
  (*cinfo->err->output_message) (cinfo);

  // Return control to the setjmp point 
  longjmp(myerr->setjmp_buffer, 1);
}


GLOBAL(int) read_JPEG_file (char * filename)
{
  // This struct contains the JPEG decompression parameters and pointers to
  // working space (which is allocated as needed by the JPEG library).
  struct jpeg_decompress_struct cinfo;
  
  // We use our private extension JPEG error handler.
  // Note that this struct must live as long as the main JPEG parameter
  // struct, to avoid dangling-pointer problems.  
  struct my_error_mgr jerr;

  FILE * infile;			// source file 
  JSAMPARRAY buffer;		// Output row buffer 
  int row_stride;			// physical row width in output buffer

  // In this example we want to open the input file before doing anything else,
  // so that the setjmp() error recovery below can assume the file is open.
  // VERY IMPORTANT: use "b" option to fopen() if you are on a machine that
  // requires it in order to read binary files.

  if ((infile = fopen(filename, "rb")) == NULL) {
    fprintf(stderr, "can't open %s\n", filename);
    return 0;
  }

  // Step 1: allocate and initialize JPEG decompression object
  // We set up the normal JPEG error routines, then override error_exit. 
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = my_error_exit;
  // Establish the setjmp return context for my_error_exit to use. 
  if (setjmp(jerr.setjmp_buffer)) {  
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    return 0;
  }
  */
/*

jpeg_decompress_struct cInfo;
     FILE *filePtr;
               
     unsigned char     *imageData;
     unsigned int     imageWidth;
     unsigned int     imageHeight;
     unsigned char     bitDepth;

     filePtr = fopen(path, "rb");
               
     jpeg_error_mgr jerr;
     cInfo.err = jpeg_std_error(&jerr);
     jpeg_create_decompress(&cInfo);
     jpeg_stdio_src(&cInfo, filePtr);

     jpeg_read_header(&cInfo, TRUE);
     jpeg_start_decompress(&cInfo);
     bitDepth = cInfo.num_components;
     imageWidth = cInfo.image_width;
     imageHeight = cInfo.image_height;

     imageData = ((unsigned char*)malloc(sizeof(unsigned char)*imageWidth*imageHeight*bitDepth));

     jpeg_finish_decompress(&cInfo);

     jpeg_destroy_decompress(&cInfo);
     free(imageData);
     fclose(filePtr);*/

// if (jpeg_info.jpeg_color_space==JCS_GRAYSCALE){
	//	SetGrayPalette();
	//	head.biClrUsed =256;
	//} else {
	//	if (jpeg_info.quantize_colors==TRUE){
	//		//SetPalette (cinfo.actual_number_of_colors, cinfo.colormap[0], cinfo.colormap[1], cinfo.colormap[2]);
	//		head.biClrUsed = jpeg_info.actual_number_of_colors;
	//	} else {
	//		head.biClrUsed = 0;
	//	}
	//}

#endif