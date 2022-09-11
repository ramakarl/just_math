
//#include <olectl.h>		-- move to ImageFormatGeneric

#include "image.h"
#include "imageformat.h"

CImageFormat::CImageFormat ()
{
	m_pOrigImage = 0x0;
	m_pNewImage = 0x0;
	m_eStatus = ImageOp::Idle;
}

CImageFormat::~CImageFormat ()
{
	// Do not del original image as CImageFormat does not own it.
	// Delete new image (temporary).
	if (m_pNewImage != 0x0 && m_pNewImage != m_pOrigImage ) {
		printf ( "Deleting image.\n" );
		delete ( m_pNewImage );
		m_pNewImage = 0x0;
	}
}

// Windows bitmaps not cross-platform
#if defined(BUILD_VC)

// Transfer a Windows bitmap into an ImageX
bool CImageFormat::TransferBitmap ( HBITMAP hBmp )
{
	/*BITMAP srcBitmap;
	HBITMAP hDestBmp = 0x0;	
	HDC hSrcDC;
	HDC hDestDC;
	XBYTE* pSrcData;
	XBYTE* pDestData;
	int x, y;

	if ( hBmp ) {
		// Get the bitmap
		GetObject ( hBmp, sizeof( srcBitmap ), &srcBitmap );

		if ( srcBitmap.bmBits != NULL) {
			// If the bits are conveniently where one would expect them to be (BMP)...
			pSrcData = (XBYTE*) srcBitmap.bmBits;
		} else {
			// If we have to do a little more work to locate the bits (JPG, GIF)...			
			hSrcDC = CreateCompatibleDC (NULL);									// DC for Src Bitmap
			if ( hSrcDC ) {
				HBITMAP hBmpOld = (HBITMAP) SelectObject ( hSrcDC, hBmp );		// hSrcDC contains the bitmap
				hDestDC = CreateCompatibleDC (NULL);						// DC for working

				if ( hDestDC ) {
					// Setup a destination bitmap to get the data bytes into
					BITMAPINFO bitmapinfo; 
					ZeroMemory(&bitmapinfo, sizeof(BITMAPINFO));
					bitmapinfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
					bitmapinfo.bmiHeader.biWidth = srcBitmap.bmWidth;
					bitmapinfo.bmiHeader.biHeight = srcBitmap.bmHeight;
					bitmapinfo.bmiHeader.biPlanes = srcBitmap.bmPlanes;
					bitmapinfo.bmiHeader.biBitCount = 24;

					// Create a destination bitmap to get the data bytes into							
					UINT* pixels;
					hDestBmp = CreateDIBSection( hDestDC, (BITMAPINFO *) &bitmapinfo, DIB_RGB_COLORS, (void **)&pixels, NULL, 0);

					if ( hDestBmp ) {
						HBITMAP hBmpOld2 = (HBITMAP) SelectObject ( hDestDC, hDestBmp );
						// Copy pixels 
						BitBlt ( hDestDC, 0, 0, srcBitmap.bmWidth, srcBitmap.bmHeight, hSrcDC, 0, 0, SRCCOPY); // 42,768 -> 44,816
						SelectObject ( hDestDC, hBmpOld2);						
						pSrcData = (XBYTE*) pixels; 						
						srcBitmap.bmBitsPixel = bitmapinfo.bmiHeader.biBitCount;
						srcBitmap.bmWidthBytes = bitmapinfo.bmiHeader.biWidth * (srcBitmap.bmBitsPixel / 8);
						DeleteDC ( hDestDC ); 
						SelectObject ( hSrcDC, hBmpOld );
						// Success. Everything ok. 
					} else {
						// Bad DestBMP
						DeleteDC ( hDestDC );	// DestDC was ok, but clean it up.
						DeleteDC ( hSrcDC );	// SrcDC was ok, but clean it up.
						m_eStatus = FormatStatus::InvalidFile;
						return false;
					}					
				} else {
					// Bad DestDC
					DeleteDC ( hSrcDC );	// SrcDC was ok, but clean it up.
					m_eStatus = FormatStatus::InvalidFile;
					return false;
				}				
			} else {
				// Bad SrcDC
				m_eStatus = FormatStatus::InvalidFile;
				return false;
			}			
		}

		// Get bitmap information		
		int bitmapPitch = srcBitmap.bmWidthBytes;		
		m_BitsPerPixel = srcBitmap.bmBitsPixel;
		m_Xres = srcBitmap.bmWidth;
		m_Yres = srcBitmap.bmHeight;

		// Transfer bitmap data into ImageX memory		
		switch ( m_BitsPerPixel ) {
		/*case 8: {
			HDC hdc = CreateCompatibleDC (NULL);
			HBITMAP hOldBitmap = (HBITMAP)SelectObject(hdc, hbmp);
			RGBQUAD palette[256];
			GetDIBColorTable ( hdc, 0, 256, palette);
			m_pImg->Size(bitmap.bmWidth, bitmap.bmHeight, IMG_TRUECOLOR);
			XBYTE * pData = (XBYTE *) m_pImg->m_data;
			XBYTE * bmpData = (XBYTE *) bits;
			for(int y = 0 ; y < m_pImg->m_yres ; y++)
			for(int x = 0 ; x < m_pImg->m_xres ; x++) {
				int n = (x+y*m_pImg->m_xres)*3; // pixel address -- destination
				int yi = m_pImg->m_yres-y-1; // inverted y, because bmp's are stored upside-down
				int ni = x+yi*pitch; // inverted pixel address -- source
				pData[n+2] = palette[bmpData[ni]].rgbBlue;   // blue
				pData[n+1] = palette[bmpData[ni]].rgbGreen; // green
				pData[n]   = palette[bmpData[ni]].rgbRed; // red
			}
			SelectObject(hdc, hOldBitmap);
			if(hdc) DeleteDC(hdc);
			break;
		}
		case 16: {
			int bmp_mask_red = 0x7C00, bmp_mask_green = 0x03E0, bmp_mask_blue = 0x001F;
			int bmp_shift_red, bmp_shift_green;
 
			bmp_shift_red   = GameX.GetShift(bmp_mask_red);
			bmp_shift_green = GameX.GetShift(bmp_mask_green);

			m_pImg->Size(bitmap.bmWidth, bitmap.bmHeight, IMG_HIGHCOLOR);
			XBYTE2 * pData = (XBYTE2 *) m_pImg->m_data;
			XBYTE2 * bmpData = (XBYTE2 *) bits;
			for(int y = 0 ; y < m_pImg->m_yres ; y++)
			for(int x = 0 ; x < m_pImg->m_xres ; x++) {
				int n = x+y*m_pImg->m_xres; // pixel address -- destination
				int yi = m_pImg->m_yres-y-1; // inverted y, because bmp's are stored upside-down
				int ni = x+yi*pitch/2; // inverted pixel address -- source
				int clr = bmpData[ni] ;
				int blue  = clr & bmp_mask_blue;  // decode blue
				int green = (clr & bmp_mask_green) >> (bmp_shift_green-1);  // decode green
				int red   = (clr & bmp_mask_red) >> bmp_shift_red; // decode red
				pData[n] = ((red << 11) + (green << 5) + blue); // re-encode
			}
		 } break;
		case 24: {
			if (m_pOrigImage->GetBitsPerPixel()==24 && m_pOrigImage->GetWidth()==m_Xres && m_pOrigImage->GetHeight()==m_Yres) {
				// Same dimensions. For speed, use original image.
				m_pNewImage = m_pOrigImage;				
			} else {
				// New image does not have same dimensions as old one. Create new image.
				// (old one will be deleted automatically by Image::Load)
				m_pNewImage = new ImageRGB24 ( m_Xres, m_Yres );		// Create RGBA Image
			}
			pDestData = (XBYTE *) m_pNewImage->GetData();
			m_BytesPerRow = m_pNewImage->GetBytesPerRow();
			
			for ( y = 0 ; y < m_Yres ; y++)
				for ( x = 0 ; x < m_Xres*3; x+=3) {
					int srcOffs = x + (m_Yres-y-1) * bitmapPitch;	// inverted pixel address -- source
					int destOffs = x + y * m_BytesPerRow;			// pixel address -- destination					
					pDestData[destOffs+2] = pSrcData[srcOffs];		// blue (not only is it upside-down, but RGB is backwards)
					pDestData[destOffs+1] = pSrcData[srcOffs+1];	// green
					pDestData[destOffs]   = pSrcData[srcOffs+2];	// red
				}
			
		 } break;
		case 32: {
			if (m_pOrigImage->GetBitsPerPixel()==32 && m_pOrigImage->GetWidth()==m_Xres && m_pOrigImage->GetHeight()==m_Yres) {
				// Same dimensions. For speed, use original image.
				m_pNewImage = m_pOrigImage;				
			} else {
				// New image does not have same dimensions as old one. Create new image.
				// (old one will be deleted automatically by Image::Load)
				m_pNewImage = new ImageRGBA32 ( m_Xres, m_Yres );		// Create RGBA Image
			}				
			pDestData = (XBYTE *) m_pNewImage->GetData ();
			m_BytesPerRow = m_pNewImage->GetBytesPerRow();
			
			for(int y = 0 ; y < m_Yres ; y++)
				for(int x = 0 ; x < m_Xres*4; x+=4) {
					int srcOffs = x + (m_Yres-y-1) * bitmapPitch;	// inverted pixel address -- source
					int destOffs = x + y * m_BytesPerRow;			// pixel address -- destination					
					pDestData[destOffs+3] = pSrcData[srcOffs+3];	// alpha
					pDestData[destOffs+2] = pSrcData[srcOffs+2];	// red
					pDestData[destOffs+1] = pSrcData[srcOffs+1];	// green
					pDestData[destOffs]   = pSrcData[srcOffs];		// blue
			}
		 } break;
		}
		if ( hDestBmp!=0x0 ) DeleteObject ( hDestBmp );
		m_eStatus = FormatStatus::Successs;
		return true;
	} else {
		// Bad Bitmap object passed in.
		m_eStatus = FormatStatus::InvalidFile;
		return false;
	}*/
	return true;
}

#endif

void CImageFormat::GetStatusMsg (char* msg)
{
#ifdef WIN32
	switch ( m_eStatus) {
	case ImageOp::Idle:					sprintf_s (msg, 1000, "Idle." ); break;
	case ImageOp::Loading:				sprintf_s (msg, 1000, "Loading." ); break;
	case ImageOp::Saving:				sprintf_s (msg, 1000, "Saving." ); break;
	case ImageOp::Successs:				sprintf_s (msg, 1000, "Success." ); break;
	case ImageOp::DepthNotSupported:	sprintf_s (msg, 1000, "Depth Not Supported." ); break;
	case ImageOp::FeatureNotSupported:	sprintf_s (msg, 1000, "Feature Not Supported." ); break;
	case ImageOp::InvalidFile:			sprintf_s (msg, 1000, "Invalid File Type." ); break;
	case ImageOp::LibVersion:			sprintf_s (msg, 1000, "Library Version Error." ); break;
	case ImageOp::FileNotFound:			sprintf_s (msg, 1000, "File Not Found." ); break;
	case ImageOp::NotImplemented:		sprintf_s (msg, 1000, "Not Implemented." ); break;
	}
#else
	switch ( m_eStatus) {
	case ImageOp::Idle:					sprintf (msg, "Idle." ); break;
	case ImageOp::Loading:				sprintf (msg, "Loading." ); break;
	case ImageOp::Saving:				sprintf (msg, "Saving." ); break;
	case ImageOp::Successs:				sprintf (msg, "Success." ); break;
	case ImageOp::DepthNotSupported:	sprintf (msg, "Depth Not Supported." ); break;
	case ImageOp::FeatureNotSupported:	sprintf (msg, "Feature Not Supported." ); break;
	case ImageOp::InvalidFile:			sprintf (msg, "Invalid File Type." ); break;
	case ImageOp::LibVersion:			sprintf (msg, "Library Version Error." ); break;
	case ImageOp::FileNotFound:			sprintf (msg, "File Not Found." ); break;
	case ImageOp::NotImplemented:		sprintf (msg, "Not Implemented." ); break;
	}
#endif

}

void CImageFormat::CreateImage ( Image*& img, int xr, int yr, ImageOp::Format fm )
{
	// Can we use existing image structure? (faster)
	if ( m_pOrigImage != 0x0 ) {
		if ( GetWidth( m_pOrigImage ) == xr && GetHeight( m_pOrigImage ) == yr && GetFormat( m_pOrigImage ) == fm ) {
			img = m_pOrigImage;
			return;
		}
	} 
	img = new Image ( xr, yr, fm );	
}

void CImageFormat::StartLoad ( char* filename, Image* pImg )
{
	m_pOrigImage = pImg;
	m_eStatus = ImageOp::Loading;
#ifdef WIN32
	strcpy_s ( m_Filename, FILE_NAMELEN, filename);
#else
	strcpy ( m_Filename, filename);
#endif

	if ( m_pNewImage != 0x0 ) {
		delete ( m_pNewImage );
		m_pNewImage = 0x0;
	}
}

void CImageFormat::FinishLoad ()
{
	// Get master image control

	if (m_pNewImage == 0x0) return;			// Load failed
	
	if ( m_pOrigImage != m_pNewImage ) {
		
		if ( m_pOrigImage == 0x0 ) {	// no original (empty) 
			m_pOrigImage = m_pNewImage;
		} else {						// new image, transfer data from it to original image		
			m_pOrigImage->TransferFrom ( m_pNewImage );		
			delete ( m_pNewImage );
			m_pNewImage = 0x0;
		}	

	} else {		
		//m_pOrigImage->Update ( 1 );			// -- This is also done in TransferFrom
	}

	m_eStatus = ImageOp::Successs;
}
