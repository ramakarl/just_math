//--------------------------------------------------------
// JUST MATH:
// Displace Sphere - raycast displacement mapping on a sphere
//
// Raycast displacement generates a displacement map rendering
// without any tesselation and with no additional memory overhead.
// Intended for use in a pixel shader or as a raytracing primitive.
// The demo shows:
// - Left sphere = tesselated geometric sphere for comparison (uses lots of triangles)
// - Right sphere = direct raycast displacement mapping (no triangles)
// 
//--------------------------------------------------------

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
#include "main.h"			// window system 
#include "nv_gui.h"			// gui system
#include "image.h"
#include "mersenne.h"
#include <GL/glew.h>
#include <algorithm>

#include "dataptr.h"
#include "common_cuda.h"

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

	void		RaycastDisplacementSphere ( Vector3DF p, float r, float bump_depth, Vector3DF lgt, Image* img );
	void		ComputeSphereUV ( Vector3DF p, Vector3DF hit, float& pu, float& pv );	
	Vector3DF	getSpherePnt ( float u, float v );
	void		DrawRayDemo ( float x, float y, Vector3DF p, float r, float bump_depth, Image* img ) ;

	void		DrawSphere ( Vector3DF p, float r, int res, float bump_depth );
	void		DrawGrid ();	

	Camera3D*	m_cam;				// camera
	Image*		m_bump_img;			// bump map
	Image*		m_out_img;

	Mersenne	m_rand;

	float		m_bump_depth;
	float		m_time;
	bool		m_run;
	int			mouse_down;


};
Sample obj;




bool Sample::init()
{
	addSearchPath(ASSET_PATH);
	init2D("arial");
	setText(24,1);
	m_time = 0;
	m_run = true;
	m_rand.seed ( 123 );

	m_cam = new Camera3D;								// create camera
	m_cam->setOrbit ( 30, 20, 0, Vector3DF(0,0,0), 20, 1 );

	std::string fpath;
	getFileLocation ( "bump_weave.png", fpath );
	m_bump_img = new Image;									// create image
	if ( !m_bump_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}
	
	m_out_img = new Image;
	m_out_img->ResizeImage ( 512, 512, ImageOp::RGBA32 );	// image resolution (output)

	return true;
}

void Sample::DrawGrid ()
{
	for (int i = -10; i <= 10; i++) {
		drawLine3D(float(i),-0.01f, -10.f, float(i), -0.01f, 10.f, .2f, .2f, .2f, 1.f);
		drawLine3D(-10.f,	-0.01f, float(i), 10.f, -0.01f, float(i), .2f, .2f, .2f, 1.f);
	}	
}

// intersectRaySphere - computes the nearest intersection of a ray and a sphere
bool intersectRaySphere ( Vector3DF lpos, Vector3DF ldir, Vector3DF spos, float sradius, Vector3DF& hit, Vector3DF& norm, float& t)
{
	double dsl = ldir.Dot ( spos - lpos );		// direction dot (spherepos - raypos)
	double ss = spos.Dot ( spos );
	double ll = lpos.Dot ( lpos );
	double rr = sradius * sradius;
	float tsqt = dsl*dsl - (ss + ll - rr - 2 * spos.Dot(lpos));
	if (tsqt < 0 ) return false;
	tsqt = sqrt( tsqt );
	t = std::min( dsl - tsqt, dsl + tsqt );
	hit = lpos + ldir * t;
	norm = (hit - spos).Normalize();

	return true;
}

#define PI2		(3.141592653589*2.0)

// SphereUV - given a sphere center, and sample point (hit), compute the spherical pu/pv coordinates in the range [0,1]
void Sample::ComputeSphereUV ( Vector3DF p, Vector3DF hit, float& pu, float& pv )
{
	double r = hit.Length();
	double lat = acos ( hit.y / r );			// latitude
	double lng = atan2 ( hit.x/r, hit.z/r );	// longitude
	lng = (lng <= 0 ) ? PI2+lng : lng;
	pu = lng / PI2;
	pv = lat / 3.141592;
}

// SpherePnt - given spherical pu/pv coordinates, return the XYZ cartesian coordinates
Vector3DF Sample::getSpherePnt ( float u, float v )
{
	double u2 = u * PI2;
	double v2 = v * 3.141592;
	return Vector3DF( sin(u2)*sin(v2), cos(v2), cos(u2)*sin(v2) );
}


void Sample::DrawRayDemo ( float x, float y, Vector3DF p, float r, float bump_depth, Image* img ) 
{
	float cr, h, tx, pu, pv;
	int i;
	float t;
	Vector3DF hit, norm;	
	Vector3DF rpos, rdir;
	Vector3DF dp(.01,.01,.01);
	int xres = img->GetWidth();
	int yres = img->GetHeight();

	float r1 = r + bump_depth;

	float dt = 0.05;

	// get camera ray
	rpos = Vector3DF(2,1,2);
	rdir = Vector3DF(0,-1,0) - rpos; 
	rdir.Normalize();

	// intersect with displacement surface			
	if ( intersectRaySphere( rpos, rdir, p, r1, hit, norm, t ) ) {

		// ray marching 
		for ( i=0; i < 128; i++) {

			cr = (hit - p).Length();			// current ray point radius

			ComputeSphereUV ( p, hit, pu, pv ); // recover pu,pv (spherical coordinates)
			
			// sample bump map
			tx = m_bump_img->GetPixelUV ( pu, pv ).x;			
			h = r + tx * bump_depth;				// displaced sphere radius
					
			if ( cr < h )
				break;

			hit += rdir * dt;					// advance ray point sampling
		}

		drawLine3D ( rpos, rpos + rdir*10.0f, Vector4DF(0.5,0.5,0.5,1) );
		drawBox3D (  hit-dp, hit+dp, 1,0,0,1 );
	}
}


void Sample::RaycastDisplacementSphere ( Vector3DF p, float r, float bump_depth, Vector3DF lgt, Image* img )
{
	float t, tx, diffuse, cr, dt, h;
	float t0, t1, cr1, h1, cr0, h0, u;
	float pu, pv, dpu, dpv;
	Vector3DF a,b,c;
	Vector3DF hit, norm;	
	Vector3DF rpos, rdir;
	int i;
	int xres = img->GetWidth();
	int yres = img->GetHeight();

	dt = 0.03;

	// expand sphere by bump depth	
	float r1 = r + bump_depth;

	lgt.Normalize();

	// clear the output image
	memset ( img->GetData(), 0, img->GetSize() );

	m_rand.seed ( 123);

	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {


			// get camera ray
			rpos = m_cam->getPos();
			rdir = m_cam->inverseRay ( x, y, xres, yres );	
			rdir.Normalize();


			// intersect with displacement surface			
			if ( intersectRaySphere( rpos, rdir, p, r1, hit, norm, t ) ) {

				// [optional] monte-carlo sampling of t phase for filtering due to dt jumping over thin features.
				// converts striations near thin features into sampling noise
				t += m_rand.randF() * dt;
				hit = rpos + rdir * t;

				// coarse ray marching 
				for ( i=0; i < 128; i++) {

					cr = (hit - p).Length();			// current ray point radius

					ComputeSphereUV ( p, hit, pu, pv ); // recover pu,pv (spherical coordinates)
					
					// sample bump map
					tx = m_bump_img->GetPixelUV ( pu, pv ).x; 									
					h = r + tx * bump_depth;			// displaced sphere radius
					
					if ( cr < h )                       // check if ray inside sphere
						break;					

					hit += rdir * dt;					// advance ray point sampling
				}

				// check for hit
				if (i < 128) {	

					// [optional] height interpolation
					cr1 = cr;							// recompute end point radius
					tx = m_bump_img->GetPixelFilteredUV ( pu, pv ).x;						
					h1 = r + tx *  bump_depth;			// recompute end point depth with filtering
					hit -= rdir * dt;					// backtrack above the surface
					cr0 = (hit - p).Length();			// current ray point radius
					ComputeSphereUV ( p, hit, pu, pv ); // recover pu,pv (spherical coordinates)
					tx = m_bump_img->GetPixelFilteredUV ( pu, pv ).x; 
					h0 = r + tx * bump_depth;			// recompute start point depth with filtering					
					t1 = (hit - rpos).Length();			
					t0 = t1 - dt;	
					u = (cr0-h0) / ((cr0-h0)-(cr1-h1));
					t1 = t0 + (t1 - t0) * u;			// interpolate between start t and end t based on zero crossing
					hit = rpos + rdir * t1;				// get interpolated hit point

					ComputeSphereUV ( p, hit, pu, pv );

					// sample neighbors to determine displaced normal
					dpu = 1.0 / (m_bump_img->GetWidth()-1);
					dpv = 1.0 / (m_bump_img->GetHeight()-1);
					a = getSpherePnt(pu-dpu,pv-dpv) * (1.0f + m_bump_img->GetPixelFilteredUV ( pu-dpu, pv-dpv ).x * bump_depth);
					b = getSpherePnt(pu+dpu,pv-dpv) * (1.0f + m_bump_img->GetPixelFilteredUV ( pu+dpu, pv-dpv ).x * bump_depth);
					c = getSpherePnt(pu+dpu,pv+dpv) * (1.0f + m_bump_img->GetPixelFilteredUV ( pu+dpu, pv+dpv ).x * bump_depth);

					norm = (c-a).Cross ( b-a ); norm.Normalize();
				
					// diffuse shading
					diffuse = (0.1 + 0.9 * std::max(0.0, norm.Dot( lgt ))) * 255.0;

					img->SetPixel ( x, y, diffuse, diffuse, diffuse, 255.0 );
				}
			}			
		}
	}

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_GLTEX );		
}


void Sample::DrawSphere ( Vector3DF p, float r, int res, float bump_depth )
{
	float cu, su, cu1, su1;
	float cv, sv, cv1, sv1;
	float pu, pv, dpu, dpv;
	Vector3DF a, b, c, d, n;	
	Vector4DF clr(1,1,1,1);	
	float s = 180.0f / res;	
	dpu = s / 360.0f;
	dpv = s / 180.0f;

	for (float v=0; v <= 180; v+= s ) {
		for (float u=0; u < 360; u += s ) {
			cu = cos(u * DEGtoRAD); cv = cos(v * DEGtoRAD); su = sin(u * DEGtoRAD); sv = sin(v * DEGtoRAD);
			cu1 = cos((u+s) * DEGtoRAD); cv1 = cos((v+s) * DEGtoRAD); su1 = sin((u+s) * DEGtoRAD); sv1 = sin((v+s) * DEGtoRAD);
			a.Set ( cu *  sv,  cv,  su *  sv);		
			b.Set (cu1 *  sv,  cv, su1 *  sv);				
			c.Set (cu1 * sv1, cv1, su1 * sv1);			
			d.Set ( cu * sv1, cv1,  su * sv1);				

			// dispalce along normal direction
			// note: special case, the points on a sphere are its own normals so we displace along normals a,b,c,d
			pu = u / 360.0;	pv = v / 180.0;			
			a += a * m_bump_img->GetPixelUV ( pu, pv ).x * bump_depth;
			b += b * m_bump_img->GetPixelUV ( pu+dpu, pv ).x * bump_depth;
			c += c * m_bump_img->GetPixelUV ( pu+dpu, pv+dpv ).x * bump_depth;
			d += d * m_bump_img->GetPixelUV ( pu,     pv+dpv ).x * bump_depth;
			
			// compute displaced normal of facet
			n = (b-a).Cross ( c-a ); n.Normalize();

			// transform to sphere location
			a = a*r+p; b = b*r+p; c = c*r+p; d=d*r+p;			

			drawFace3D(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x, d.y, d.z, n.x, n.y, n.z, clr.x, clr.y, clr.z, clr.w);
		}
	}
}


void Sample::display()
{
	clearGL();

	Vector3DF lgt (20, 100, 60);

	m_bump_depth = sin( m_time*2.0*3.141592/180.0)*0.1 + 0.1;

	if (m_run)
		m_time += 1.0;
	
	// Displacement raycasting
	RaycastDisplacementSphere ( Vector3DF(0,0,0), 1.0, m_bump_depth, lgt, m_out_img );

	// Draw geometric sphere in 3D
	start3D(m_cam);
	setLight(S3D, lgt.x, lgt.y, lgt.z);	

	  DrawGrid();

	  DrawSphere ( Vector3DF(-3,0,0), 1.0, 120, m_bump_depth );	
	  
	  //DrawRayDemo ( 256, 256, Vector3DF(0,0,0), 1.0, m_bump_depth, m_out_img ) ;

	end3D();		
	draw3D();										// complete 3D rendering to OpenGL

	// Draw raycast image
	start2D();
	setview2D(getWidth(), getHeight());		
	drawImg ( m_out_img->getGLID(), 0, 0, getWidth(), getHeight(), 1,1,1,1 );		// draw raycast image 	
	end2D();	
	draw2D();										// complete 2D rendering to OpenGL


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
		Vector3DF angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam->setOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());		
		appPostRedisplay();	// Update display
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	if (guiHandler(button, state, x, y)) return;
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

	m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), dist, dolly);
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action==AppEnum::BUTTON_RELEASE) return;

	switch (keycode) {
	case ' ': m_run = !m_run; break;
	}
}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

	m_cam->setSize( w, h );
	m_cam->setAspect(float(w) / float(h));
	m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
	m_cam->updateMatricies();

	appPostRedisplay();
}

void Sample::startup()
{
	int w = 1900, h = 1000;
	appStart("Displacement Map Sphere", "Displacement Map Sphere", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}




