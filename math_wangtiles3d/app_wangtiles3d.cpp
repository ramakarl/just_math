//--------------------------------------------------------
// JUST MATH:
// Wang Tiles in 3D
// 
// Wang Tiles are a method for generating a known distribution with 
// equal sample spacing that is scale invariant. 
// This implementation is based on the paper:
//    Kopf, Cohen-Or, Deussen, Lischinski, "Recursive Wang Tiles for Real-Time Blue Noise", Siggraph 2006
//
// This alternative application of Wang Tiles is to generate a consistent 
// sampling for the instancing of 3D geometry on a known distribution over a landscape.
// The known distribution is a density map of the 3D objects, such as a tree or vegetation density,
// and the instances (in this case cubes) are placed at the sample locations. 
// The result is a 'forest' of colored samples that maintains a consistent frame rate.
// 
// Different from the 2D example, the samples should only be generated in within
// the camera view frustum, and there should be a weighting term which places more samples
// closer to the camera. This can be seen easily by turning on the 2D map with spacebar.
// Use the W,S,A,D and right mouse to fly through the forest of samples.
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

#include <time.h>
#include "main.h"			// window system 
#include "nv_gui.h"			// gui system
#include "image.h"

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

	Image		img;
	float*		m_density;
	float		m_toneScalar;

	Vector3DF	m_view;
	Vector4DF	m_pal[16];
	
	Camera3D*	m_cam;

	bool		m_showmap;

	int			mouse_down;
};
Sample obj;


bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width & height

	m_toneScalar = 1200.0;		// gain factor for density

	m_showmap = false;

	addSearchPath ( ASSET_PATH );
	init2D ( "arial" );
	setview2D ( w, h );	
	setText ( 16, 1 );			

	//-- 16-color palette
	for (int n = 0; n < 16; n++) {
		m_pal[n].Set(int(n >> 2) % 2, int(n >> 1) % 2, n % 2, 1);
	}

	//-- Load source image
	std::string errmsg;
	if ( !img.Load ( "assets\\woman.png", errmsg ) ) {
		dbgprintf("ERROR: Unable to load woman.png: %s\n", errmsg.c_str() );
		exit(-1);
	}
	img.Commit (DT_GLTEX);

	//-- Load wang tiles
	if (!wt.LoadTileSet("assets\\wang_tileset.dat")) {
		dbgprintf("ERROR: Unable to find wang_tileset.dat\n");
		exit(-1);
	}
	
	//-- Construct density function
	Vector3DF pix;
	float v;
	int xres = img.GetWidth();
	int yres = img.GetHeight();
	
	m_density = (float*) malloc ( xres*yres* sizeof(float) );
	for (int y = 0; y < yres; y++)
		for (int x = 0; x < xres; x++) {
			pix = img.GetPixelUV( float(x)/xres, float(y)/yres );		// UV = using range [0,1]  (does not mean a different channel)
			m_density[y*xres+x] = pix.LengthFast() - 0.3;					// Adjust gamma and offset here.. LengthFast = grayscale from RGB vector
		}

	wt.SetDensityFunc ( m_density, img.GetWidth(), img.GetHeight() );		// assign density function to Wang Tile sampler

	m_view.Set(0, 0, 1);

	//--- Set camera
	m_cam = new Camera3D;
	m_cam->setFov ( 90 );
	m_cam->setOrbit ( Vector3DF(0,5,0), Vector3DF(388,2,480), 50, 1 );

	//--- Set light
	setLight (0, 500, 500, 500);
	
	return true;
}


void Sample::drawGrid()
{
	int w = getWidth(), h = getHeight();
	for (int n = 0; n <= 1000; n+=20 ) {
		drawLine3D ( n, 0, 0, n, 0, 1000, 1, 1, 1, .1);
		drawLine3D ( 0, 0, n, 1000, 0, n, 1, 1, 1, .1);
	}
}


void Sample::display ()
{	
	Vector3DF pnt;
	Vector4DF clr;
	Vector3DF cmin, cmax;
	float zoom;

	int w = getWidth(); 
	int h = getHeight();
	Vector3DF imgres ( img.GetWidth(), img.GetHeight(), 0 );
	
	Vector3DF a, b, c;
	Vector3DF pt, pos;
	Vector4DF pix(1, 1, 1, 1);
	int cl;
	int o;
	float d;

	clearGL();

	// Determine view settings
	float dst = 800.0;				// maximum camera distance to generate points
	float zm = 1.0;	

	// Generate points using Wang Tiles, given 3D camera, zoom factor, and tone scale
	int pnts = wt.Recurse3D (m_cam, zm, m_toneScalar, dst);

	// Draw 3D
	start3D(m_cam);
		
		drawGrid();

		for (int n = 0; n < pnts; n++) {
			
			pt = wt.getPnt(n);											// get generated point			
			
			a = Vector3DF(pt.x, 0, pt.y);								// get 3D position
			o = int(pt.y * imgres.x) + int(pt.x);						// offset into density func
			b = a + Vector3DF(.1, m_density[o] * 5.0, .1);				// get 3D box dimensions. height = density
			
			if (m_cam->boxInFrustum(a, b))	{							// cull box if outside camera frustum
				d = 5000.0 / (m_cam->getPos() - a).LengthFast();				// use distance as alpha falloff
				pix = img.GetPixelUV(pt.x / imgres.x, pt.y / imgres.y);		// get point color (sample original image)			
				drawCube3D ( a, b, pix.x*5, pix.y*5, pix.z*5, 1 );		// draw box
			}
		}

	end3D();

	// Draw 2D view
	start2D();

		setview2D(getWidth(), getHeight());
		drawText(1200, 20, "Wang Tiles", 1,1,1,1);
		drawText(1200, 35, "  Right mouse  Change direction", 1,1,1,1);
		drawText(1200, 50, "  W,S,A,D      Fly through map", 1,1,1,1);			
		drawText(1200, 65, "  Z,X          Adjust density", 1,1,1,1);			
		drawText(1200, 80, "  Spacebar     Show/hide 2D map", 1,1,1,1);			
		
		if ( m_showmap ) {
			
			// Draw camera frustum in 2D view (yellow)
			float sc=3.0;
			a = m_cam->getPos();  
			b = a + m_cam->inverseRay(0, 0, w, h) * dst;
			c = a + m_cam->inverseRay(w, h, w, h) * dst;
			drawLine ( a.x/sc, a.z/sc, b.x/sc, b.z / sc, 1, 1, 0, 1);
			drawLine ( a.x/sc, a.z/sc, c.x/sc, c.z / sc, 1, 1, 0, 1);
		
			// Draw points
			for (int n=0; n < pnts; n += 2 ) {			// skip every other pnt (faster)
				a = wt.getPnt(n);
				pix = img.GetPixelUV( a.x / imgres.x, a.y / imgres.y );	

				if ( !m_cam->pointInFrustum( a.x, 0, a.y ) ) {pix.x = pix.y = pix.z; }		// show points outside frustum as grey

				drawPnt ( a.x/sc, a.y/sc, pix );
			}
			drawRect ( 10, 10, imgres.x/sc, imgres.y/sc, 1, 1, 1, 1);
		
		} 

	end2D();

	//--- Actual rendering here
	draw3D();

	draw2D (); 	

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
	Vector3DF dang; 

	switch ( mouse_down ) {	
	case AppEnum::BUTTON_LEFT: case AppEnum::BUTTON_RIGHT: {

		Vector3DF angs = m_cam->getAng();				
		angs.x -= dx * 0.1;
		angs.y += dy * 0.1;
		m_cam->setAngles ( angs.x, angs.y, 0  );		// adjust camera direction
		appPostRedisplay();
		} break;

	case AppEnum::BUTTON_MIDDLE: {
		} break; 

	};
}

void Sample::mousewheel(int delta)
{
	m_view.z += delta*0.001;
	appPostRedisplay();
}

void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action == AppEnum::BUTTON_RELEASE) return;

	switch ( keycode ) {
	case ' ':	m_showmap = !m_showmap; break;
	case '1':	wt.SetMaxPoints( 100000);	break;
	case '2':	wt.SetMaxPoints(1000000);	break;

	case 'z':	m_toneScalar -= 100;	break;					// adjust overall density
	case 'x':	m_toneScalar += 100;	break;

	case 'w':	m_cam->moveRelative ( 0, 0, -1 ); break;		// WASD navigation keys
	case 'a':	m_cam->moveRelative ( -1, 0, 0); break;
	case 's':	m_cam->moveRelative (0, 0, +1); break;
	case 'd':	m_cam->moveRelative ( +1, 0, 0); break;
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
	int w = 1600, h = 1000;
	appStart ( "Wang Tiles 3D", "Wang Tiles 3D", w, h, 4, 2, 8, false );
}

void Sample::shutdown()
{
}

