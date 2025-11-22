//--------------------------------------------------------
// JUST MATH:
// Wang Tiles (2D)
// 
// Wang Tiles are a method for generating a known distribution with 
// equal sample spacing that is scale invariant. 
// This implementation is based on the paper:
//    Kopf, Cohen-Or, Deussen, Lischinski, "Recursive Wang Tiles for Real-Time Blue Noise", Siggraph 2006
//
// Given a spatial distribution function f(x,y), we would like a sampling
// that is non-periodic (looks random), deterministic (same each run), scale invariant, and fast.
// The method uses carefully pre-generated, recursive tiles that can be constructed in real-time.
//
// The example shown is a 2D image that is half-toned using dots (samples), where the 
// density of dots should match the intensity of the image, and should *maintain* that density as we zoom in.
// Demo input:
//   Use the left mouse to pan.
//   Use the right mouse to zoom. Zoom very slowly to see sample generation interactively.
// 

//--------------------------------------------------------------------------------
// Copyright 2019-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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

//--------------------------------------------------
// Wang Tiles
// 
//
// A "Just Math" Demo
// by Rama Karl Hoetzlein (c) 2021
// License CC-BY.
//
//--------------------------------------------------

#include <time.h>
#include "main.h"			// window system 
#include "gxlib.h"			// gui system
using namespace glib;

#include "wang_tiles.h"

class Sample : public Application {
public:
	virtual bool init();
	virtual void startup ();
	virtual void display();
	virtual void reshape(int w, int h);
	virtual void motion (AppEnum button, int x, int y, int dx, int dy);	
	virtual void keyboard(int keycode, AppEnum action, int mods, int x, int y);
	virtual void mouse (AppEnum button, AppEnum state, int mods, int x, int y);	
	virtual void mousewheel(int delta);
	virtual void shutdown();

	void		drawGrid();

	WangTiles	wt;

	ImageX		img;
	float*		density;

	Vec3F	m_view;
	Vec4F	m_pal[16];
	
	int			mouse_down;
};
Sample obj;


bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width & height

	addSearchPath ( ASSET_PATH );
	init2D ( "arial" );	
	setTextSz ( 16, 1 );			

	//-- 16-color palette
	for (int n = 0; n < 16; n++) {
		m_pal[n].Set(int(n >> 2) % 2, int(n >> 1) % 2, n % 2, 1);
	}

	//-- Load source image
	std::string errmsg;
  std::string fpath;
  bool ok = false;
  if ( getFileLocation ( "woman.png", fpath) ) 
    if ( img.Load(fpath, errmsg) ) 
      ok = true;
    
  if (!ok) { dbgprintf("ERROR: Unable to load woman.png %s\n" ); exit(-1); }
  
	img.Commit (DT_GLTEX);

	//-- Load wang tiles
  ok = false;
  if ( getFileLocation("wang_tileset.dat", fpath) ) 
    if ( wt.LoadTileSet( fpath.c_str() ) ) 
      ok = true;
	
  if (!ok) { dbgprintf("ERROR: Unable to load wang_tileset.dat\n"); exit(-2); }	
	
	//-- Construct density function
	Vec3F pix;
	float v;
	int xres = img.GetWidth();
	int yres = img.GetHeight();
	
	float* density = (float*) malloc ( xres*yres* sizeof(float) );
	for (int y = 0; y < yres; y++)
		for (int x = 0; x < xres; x++) {
			pix = img.GetPixelUV( float(x)/xres, float(y)/yres );	// UV = using range [0,1]
			v = Vec3F(pix).LengthFast();    									// convert color to grayscale value
			density[y*xres+x] = v - 0.3;													//	adjust gamma and offset here
		}

	wt.SetDensityFunc ( density, img.GetWidth(), img.GetHeight() );		// assign density function to Wang Tile sampler

	m_view.Set(0, 0, 1);

	return true;
}


void Sample::drawGrid()
{
	int w = getWidth(), h = getHeight();
	for (int n = 0; n <= w; n+=50 ) 	drawLine ( Vec2F(n, 0), Vec2F(n, h), Vec4F(1, 1, 1, .7) );
	for (int n = 0; n <= h; n+=50)		drawLine( Vec2F(0, n), Vec2F(w, n), Vec4F(1, 1, 1, .7) );	
}

void Sample::display ()
{	
	Vec3F pnt;
	Vec4F clr;
	Vec3F cmin, cmax;
	float zoom;

	int w = getWidth();
	int h = getHeight();
	Vec3F imgres ( img.GetWidth(), img.GetHeight(), 0 );

	clearGL();

	start2D( w, h );
		
		drawImg ( &img, Vec2F(0,0), Vec2F(200,200), Vec4F(1,1,1,1) );
		
		// Determine normalized view domain (portion of image to sample)
		cmin = Vec3F(m_view.x, m_view.y, 0) / Vec3F(imgres.x, imgres.y, 0);
		cmax = Vec3F(m_view.x + imgres.x / m_view.z, m_view.y + imgres.y / m_view.z, 0 ) / Vec3F(imgres.x, imgres.y, 0 );
		float toneScalar = 2000.0;
		
		// Generate points using Wang Tiles, given view domain, zoom factor, and tone scale
		int pnts = wt.RecurseTileImage(cmin, cmax, m_view.z, toneScalar);			
		
		Vec2F pt, pos;
		Vec4F pix (1,1,1,1);
		
		// Draw points
		for (int n=0; n < pnts; n++ ) {
			pt = wt.getPnt(n);
			pos.x = (pt.x - cmin.x) * m_view.z * w;
			pos.y = (pt.y - cmin.y) * m_view.z * h;			
			//pix = img.GetPixelUV( pt.x, pt.y ) * (1.f/256);		//--- with color (slower)

			drawLine ( pos, pos+Vec2F(0.5,0.5), pix ); 			
			// drawPoint ( pos.x, pos.y, pix );
		}

	end2D();

	drawAll (); 	

	appPostRedisplay();								// Post redisplay since simulation is continuous
}


void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	int w = getWidth(), h = getHeight();				// window width & height

	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;

}


void Sample::motion (AppEnum button, int x, int y, int dx, int dy) 
{
	// Get camera for scene
	bool shift = (getMods() & KMOD_SHIFT);		// Shift-key to modify light
	float fine = 0.5f;
	Vec3F dang; 

	switch ( mouse_down ) {	
	case AppEnum::BUTTON_LEFT:  {	
	
		m_view.x += dx / m_view.z;		// pan view
		m_view.y += dy / m_view.z;
	
		} break;

	case AppEnum::BUTTON_MIDDLE: {
		} break; 

	case AppEnum::BUTTON_RIGHT: {
		
		m_view.z += dy * 0.01;			// zoom view
		appPostRedisplay();

		} break;	

	}
}

void Sample::mousewheel(int delta)
{

}

void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action == AppEnum::BUTTON_RELEASE) return;

	switch ( keycode ) {
	case '1':	wt.SetMaxPoints( 100000);	break;
	case '2':	wt.SetMaxPoints(1000000);	break;
	};
}

void Sample::reshape (int w, int h)
{
	glViewport ( 0, 0, w, h );
	setview2D ( w, h );
		
	appPostRedisplay();	
}

void Sample::startup ()
{
	int w = 1000, h = 1000;
	appStart ( "Wang Tiles", "Wang Tiles", w, h, 4, 2, 16, false );
}

void Sample::shutdown()
{
}

