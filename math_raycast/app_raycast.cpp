//--------------------------------------------------------
// JUST MATH:
// Raycast - raytracing a volume entirely on CPU
// 
// This demo shows how to raytrace a volume. 
// For this demo the input volume is generated automatically from a time-evolving 
// mathematical function which has both color and opacity at every voxel.
//
// Raytracing of a volume is an integral over the voxels for each ray.
// The method implemented here is front-to-back compositing. 
//    See: Arie Kaufman & Klaus Mueller, "Overview of Volume Rendering", 2005, eqn 12 & 13
// Discretely, the integral must accumulate the color and opacity while sampling along the ray.
// The two relevant lines of code are:
//    	clr += Vec4F(val.x,val.y,val.z, 0) * (1-clr.w) * alpha * kIntensity * pStep;	// accumulate color						
//		clr.w += alpha * kDensity * pStep;							// attenuate alpha					
// where
//      C(i) = val.xyz = the current voxel color, val.w is the voxel opacity
//      c(i) = clr.xyz = the accumulated color
//      alpha(i) = clr.w = the accumulated alpha
// A camera ray first hits the bounding box of the volume. That hit 't' provides the first sample on the ray.
// The sampling then proceed along t += dt with sample spacing pStep, until the ray exits the volume,
// or the ray is so opaque that new samples are below a contribution threshold, or a maximum # samples is reached.
// The spacing pStep is included in the equations above so that kIntensity and kDensity controls can be invariant with sample spacing.
// 
// Typically this code would be implemented on the GPU using either OpenGL/DirectX shaders, 
// or a GPU-compute language such as CUDA. Here I demonstrate the pure CPU code for instructional purposes,
// which may then be ported to either of these.

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


// Sample utils
#include <GL/glew.h>
#include <algorithm>
#include "main.h"			// window system 
#include "gxlib.h"			// gui system
using namespace glib;

#include "imagex.h"
#include "geom_helper.h"
#include "dataptr.h"

#define BUF_VOL			0

class Sample : public Application {
public:
	virtual void startup();
	virtual bool init();
	virtual void display();
	virtual void reshape(int w, int h);
	virtual void motion(AppEnum button, int x, int y, int dx, int dy);
	virtual void keyboard(int keycode, AppEnum action, int mods, int x, int y);
	virtual void mouse(AppEnum button, AppEnum state, int mods, int x, int y);
	virtual void mousewheel(int delta);
	virtual void shutdown();

	void		AllocBuffer(int id, Vec3I res, int chan=1);
	float		getVoxel ( int id, int x, int y, int z );
	Vec4F	getVoxel4 ( int id, int x, int y, int z );
	void		WriteFunc (int id, float time);
	void		RaycastCPU ( Camera3D* cam, int id, ImageX* img );

	int			mouse_down;	
	bool		m_run;
	bool		m_run_cuda;
	bool		m_save;
	float		m_frame;
	int			m_peak_iter;

	Camera3D*	m_cam;				// camera
	ImageX*		m_img;				// output image
	
	Vec3I	m_res;				// volume res

	DataPtr		buf[64];			// data buffers (CPU & GPU)
};
Sample obj;


void Sample::AllocBuffer (int id, Vec3I res, int chan)
{
	int cnt = res.x*res.y*res.z;
	int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
	buf[id].Resize( chan*sizeof(float), cnt, 0x0, flags );
}
float Sample::getVoxel ( int id, int x, int y, int z )
{
	float* dat = (float*) buf[id].getPtr ( (z*m_res.y + y)*m_res.x + x );
	return *dat;
}
Vec4F Sample::getVoxel4 ( int id, int x, int y, int z )
{
	Vec4F* dat = (Vec4F*) buf[id].getPtr ( (z*m_res.y + y)*m_res.x + x );	
	return *dat;
}

void Sample::WriteFunc (int id, float time)
{
	Vec3F c, d;
	float v;
	Vec4F* dat = (Vec4F*) buf[id].getData();
	Vec4F* vox = dat;
	
	c = m_res / 2;

	float maxv = sqrt(c.x*c.x + c.y*c.y + c.z*c.z );
	Vec3F s;

	for (int z=0; z < m_res.z; z++)
		for (int y=0; y < m_res.y; y++)
			for (int x=0; x < m_res.x; x++) {
				d.Set ( (x-c.x), (y-c.y), (z-c.z) );
				
				//v = max(0.6 - (sqrt(d.x*d.x + d.y*d.y + d.z*d.z) / maxv), 0);		// sphere
				
				v = sin(d.x*d.y*d.z*0.001 + time*0.1)*0.5+0.5;						// sin(x*y*z + t)
				v = v*v*v;
				
				/*s.x = sin (d.x*0.1 + time*0.1)*0.25+0.5;							// sin(x)+cos(z) < y
				s.z = cos (d.z*0.2 + time*0.1)*0.25+0.5;
				v = (y < (s.x+s.z)*c.y ) ? 1.0 : 0;	 */

				vox->x = float(x)/m_res.x;
				vox->y = float(y)/m_res.y;
				vox->z = float(z)/m_res.z;
				vox->w = v; 
				vox++;
			}

	if ( m_run_cuda )
		buf[id].Commit ();			// commit to GPU
}

void Sample::RaycastCPU ( Camera3D* cam, int id, ImageX* img )
{
	Vec3F rpos, rdir;
	Vec4F clr;

	Vec3F vmin (0,0,0);
	Vec3F vmax (1,1,1);
	Vec3F wp, dwp, p, dp;
	Vec3F vdel = m_res;
	Vec4F val;
	float t;
	int iter;
	float alpha, k;
	float pStep = 0.01;					// volume quality   - lower=better (0.01), higher=worse (0.1)
	float kDensity = 5.0;				// volume density   - lower=softer, higher=more opaque
	float kIntensity = 8.0;				// volume intensity - lower=darker, higher=brighter
	float kWidth = 4.0;					// transfer func    - lower=broader, higher=narrower (when sigmoid transfer enabled)

	int xres = img->GetWidth();
	int yres = img->GetHeight();
	
	// for each pixel in image..
	m_peak_iter = 0;
	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {
			
			// get camera ray
			rpos = cam->getPos();
			rdir = cam->inverseRay ( x, y, xres, yres );	
			rdir.Normalize();

			// intersect with volume box			
			clr.Set(0,0,0,0);
			if ( intersectLineBox ( rpos, rdir, vmin, vmax, t ) ) {
				// hit volume, start raycast...		
				wp = rpos + rdir * t + Vec3F(0.001, 0.001, 0.001);		// starting point in world space				
				dwp = (vmax-vmin) * rdir * pStep;								// ray sample stepping in world space
				p = Vec3F(m_res-1) * (wp - vmin) / (vmax-vmin);				// starting point in volume				
				dp = Vec3F(m_res-1) * rdir * pStep;							// step delta along ray
				
				// accumulate along ray
				for (iter=0; iter < 512 && clr.w < 0.95 && p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < m_res.x && p.y < m_res.y && p.z < m_res.z; iter++) {
					val = getVoxel4 ( BUF_VOL, p.x, p.y, p.z );					// get voxel value
					alpha = val.w;												// opacity = linear transfer
					//alpha = 1.0 / (1+exp(-(val.w-1.0)*kWidth));				// opacity = sigmoid transfer - accentuates boundaries at 0.5
					clr += Vec4F(val.x,val.y,val.z, 0) * (1-clr.w) * alpha * kIntensity * pStep;	// accumulate color						
					clr.w += alpha * kDensity * pStep;							// attenuate alpha					
					p += dp; 													// next sample
				}	
				if (iter > m_peak_iter) m_peak_iter = iter;
				if (clr.x > 1.0) clr.x = 1;
				if (clr.y > 1.0) clr.y = 1;
				if (clr.z > 1.0) clr.z = 1;		        
			}	
			// set pixel
			img->SetPixel ( x, y, clr );
		}
	}

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_GLTEX );			
	
	// optional write to disk
	if ( m_save ) {
		char savename[256];
		sprintf ( savename, "out%04d.png", (int) m_frame );
		img->Save ( savename );				
	}
}

bool Sample::init()
{
	addSearchPath(ASSET_PATH);
	init2D("arial");
	setTextSz(24,1);

	// options
	m_frame = 0;
	m_run = true;
	m_run_cuda = false;									// run cuda pathway
	m_save = false;										// save image sequence to disk
	
	m_res.Set (64, 64, 64);							// volume resolution

	m_cam = new Camera3D;								// create camera
	m_cam->SetOrbit ( 30, 20, 0, Vec3F(.5,.5,.5), 4, 1 );

	m_img = new ImageX;									// create image
	m_img->Resize ( 256, 256, ImageOp::RGBA8 );	// image resolution (output)

	#ifdef USE_CUDA		
		if ( m_run_cuda ) {
			CUcontext ctx; 
			CUdevice dev;
			cuStart ( DEV_FIRST, 0, dev, ctx, 0, true );		// start CUDA
		}
	#endif

	AllocBuffer ( BUF_VOL, m_res, 4 );					// allocate color volume (4 channel)

	return true;
}

void Sample::display()
{
	Vec3F a, b, c;
	Vec3F p, q, d;
	int w = getWidth(), h = getHeight();

	clearGL();
	
	// advance
	if (m_run) {									// time update
		WriteFunc ( BUF_VOL, m_frame );				// write to volume
		m_frame++;
	}

	// raycast
	RaycastCPU ( m_cam, BUF_VOL, m_img );			// raycast volume

	// draw grid in 3D
	start3D(m_cam);
	setLight3D( Vec3F(20, 100, 20), Vec4F(1,1,1,1) );	
	for (int i=-10; i <= 10; i++ ) {
		drawLine3D( Vec3F(i, 0, -10), Vec3F(i, 0, 10), Vec4F(1,1,1, .2) );
		drawLine3D( Vec3F(-10, 0, i), Vec3F(10, 0, i), Vec4F(1,1,1, .2) );
	}
	drawBox3D ( Vec3F(0,0,0), Vec3F(1,1,1), Vec4F(1,1,1,0.3) );
	end3D();

	// draw 2D
	start2D( w, h );
	
	// draw rectangular raycast image into view
	float margin = (w - h)/2;	
	drawImg ( m_img, Vec2F(margin, 0), Vec2F(w-margin,h), Vec4F(1,1,1,1) );		// draw raycast image 	
	
	end2D();

	drawAll();
	
	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
	float fine = 0.5;

	switch (mouse_down) {
	case AppEnum::BUTTON_LEFT: {

		appPostRedisplay();	// Update display
	} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
		appPostRedisplay();	// Update display
	} break;

	case AppEnum::BUTTON_RIGHT: {

		// Adjust camera orbit 
		Vec3F angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam->SetOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());		
		appPostRedisplay();	// Update display
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag
}

void Sample::mousewheel(int delta)
{
	// Adjust zoom
	float zoomamt = 1.0;
	float dist = m_cam->getOrbitDist();
	float dolly = m_cam->getDolly();
	float zoom = (dist - dolly) * 0.001f;
	dist -= delta * zoom * zoomamt;

	m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), dist, dolly);
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action==AppEnum::BUTTON_RELEASE) return;

	switch (keycode) {
	case ' ':	m_run = !m_run;	break;
	};
}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

	m_cam->setSize( w, h );
	m_cam->setAspect(float(w) / float(h));
	m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());	

	appPostRedisplay();
}

void Sample::startup()
{
	int w = 800, h = 800;
	appStart("Volume Raycast", "Volume Raycast", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}




