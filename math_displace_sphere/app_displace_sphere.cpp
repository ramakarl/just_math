//--------------------------------------------------------
// JUST MATH:
// Displace Sphere - raycast displacement mapping on a sphere
//
// Raycast displacement generates a displacement map rendering without any tesselation and 
// with no additional memory overhead. Inspired by the paper:
//    Tesselation-Free Displacement Mapping for Raytracing, Thonat et al, 2021
// Demo here is inspired by but quite different from that. Notably the D-BVH acceleration
// structure is not shown, and the surface here is a simple sphere (implicit).
// Intended for use in a pixel shader or as a raytracing primitive.
// The demo shows:
// - Left sphere = Tesselated geometric sphere for comparison (uses lots of triangles)
// - Right sphere = Direct raycast displacement mapping (no triangles)
// Techniques demonstrated are:
// - Ray marching an implicit sphere
// - Ray-sphere intersection test
// - Monte-carlo for noise sampling fine features
// - Heightfield linear interpolation (reduces raymarching dt)
// - Displacement map bilinear interpolation 
// - Normal calculations for a displaced sphere
// - Tesselation and displacement of a geometric (faceted) sphere for comparison
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
#include <GL/glew.h>
#include <algorithm>

#include "main.h"			// window system 
#include "imagex.h"
#include "mersenne.h"
#include "dataptr.h"

#include "gxlib.h"
using namespace glib;

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

	void		RaycastDisplacementSphere ( Vec3F p, float r, float bump_depth, float dt, Vec3F lgt, ImageX* img );
	void		ComputeSphereUV ( Vec3F p, Vec3F hit, float& pu, float& pv );	
	Vec3F	  getSpherePnt ( float u, float v );
	void		DrawRayDemo ( float x, float y, Vec3F p, float r, float bump_depth, ImageX* img ) ;

	void		DrawTesselatedSphereGL ( Vec3F p, float r, int res, float bump_depth );
	void		DrawGrid ();	

  void    Update()    { m_update = true; }
	void		MoveCamera();

	Camera3D*	m_cam;				  // camera
  Vec3F     m_lgt;
	ImageX*		m_bump_img;			// bump map
	ImageX*		m_out_img;	


	Mersenne	m_rand;

	float		m_bump_depth;
	float		m_time;
	int			m_samples;
	bool		m_update;
	int			mouse_down;

};
Sample obj;




bool Sample::init()
{
	addSearchPath(ASSET_PATH);
	init2D ("arial");
	setTextSz (24, 1);
	m_time = 0;	
	m_rand.seed ( 123 );

	int res = 1024;

	// create a camera & light
	m_cam = new Camera3D;								
	m_cam->SetOrbit ( 30, 20, 0, Vec3F(0,0,0), 7, 1 );
  m_lgt = Vec3F(20, 200, 30);

	// load displacement map 
	std::string fpath;
	getFileLocation ( "bump_weave.png", fpath );
	dbgprintf ("loading: %s\n", fpath.c_str() );
	m_bump_img = new ImageX;									// create image
	if ( !m_bump_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}
	
	// create output image
	m_out_img = new ImageX;
	m_out_img->Resize ( res, res, ImageOp::RGBA8 );	
	
  m_bump_depth = 0.2;

  // detailed render 
  reshape( getWidth(), getHeight() );
	MoveCamera();
  RaycastDisplacementSphere(Vec3F(0, 0, 0), 1.0, m_bump_depth, 0.01, m_lgt, m_out_img);
  m_update = false;

	return true;
}
void Sample::DrawGrid ()
{
	for (int i = -10; i <= 10; i++) {
		drawLine3D(Vec3F(float(i),-0.01f, -10.f), Vec3F(float(i), -0.01f, 10.f), Vec4F(.2f, .2f, .2f, 1.f) );
		drawLine3D(Vec3F(-10.f,	-0.01f, float(i)), Vec3F(10.f, -0.01f, float(i)), Vec4F(.2f, .2f, .2f, 1.f) );
	}	
}

// intersectRaySphere - computes the nearest intersection of a ray and a sphere
bool intersectRaySphere ( Vec3F lpos, Vec3F ldir, Vec3F spos, float sradius, Vec3F& hit, Vec3F& norm, float& t)
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
#define PI		(3.141592653589)
#define PI2		(3.141592653589*2.0)

const unsigned int sin_lut[7] = {12540, 6393, 3212, 1608, 804, 402, 201};
const unsigned int cos_lut[7] = {30274, 32138, 32610, 32729, 32758, 32766, 32767};
const unsigned int tan_lut[6] = {32768, 13573, 6518, 3227, 1610, 804};

float fast_atan2 (float _x, float _y) 
{
	int x0 = _x*65536.0f; 
	int y0 = _y*65536.0f; 
	int x1, y1, phi, tmp;
	int k, currentAngle;
	int angle=0; 
	int sign=0;	
	if(y0<0) {x0=-x0; y0=-y0; angle=512;}
	if(x0<0) {tmp=x0; x0=y0; y0=-tmp; angle+=256;}
	if(y0>x0) {sign=1; angle+=256; tmp=x0; x0=y0; y0=tmp;}
	currentAngle=64; phi=0;	k=0;
	while (k<=6)
	{
		x1=(long) cos_lut[k]*(long)x0;
		x1+=(long)sin_lut[k]*(long)y0;
		y1=(long) cos_lut[k]*(long)y0;
		y1-=(long)sin_lut[k]*(long)x0;
		if(y1>=0) {x0=x1>>15; y0=y1>>15; phi+=currentAngle;}
		currentAngle = currentAngle>>1;
		k++;
    }
	float radius=x0;
	angle+=((sign)?-phi:+phi); //if(sign) {angle-=phi;} else {angle+=phi;}
	// angle 2pi = 1024
	angle=angle << 6;  // stretch to the size of int. angle resolution is: 64
	// angle 2pi now is 65536
	return float(angle*PI2/65536.0f);
  }


// SphereUV - given a sphere center, and sample point (hit), compute the spherical pu/pv coordinates in the range [0,1]
void Sample::ComputeSphereUV ( Vec3F p, Vec3F hit, float& pu, float& pv )
{
	hit.Normalize();
	float lat = acos ( hit.y );				// latitude
	float lng = atan2 ( hit.z, hit.x );		// longitude
	lng = (lng <= 0 ) ? PI2+lng : lng;
	pu = lng / PI2;
	pv = lat / PI;
}

// SpherePnt - given spherical pu/pv coordinates, return the XYZ cartesian coordinates
Vec3F Sample::getSpherePnt ( float u, float v )
{
	double u2 = u * PI2;
	double v2 = v * PI;
	return Vec3F( sin(u2)*sin(v2), cos(v2), cos(u2)*sin(v2) );
}


void Sample::DrawRayDemo ( float x, float y, Vec3F p, float r, float bump_depth, ImageX* img ) 
{
	float cr, h, tx, pu, pv;
	int i;
	float t;
	Vec3F hit, norm;	
	Vec3F rpos, rdir;
	Vec3F dp(.01,.01,.01);
	int xres = img->GetWidth();
	int yres = img->GetHeight();

	float r1 = r + bump_depth;

	float dt = 0.05;

	// get camera ray
	rpos = Vec3F(2,1,2);
	rdir = Vec3F(0,-1,0) - rpos; 
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

		drawLine3D ( rpos, rpos + rdir*10.0f, Vec4F(0.5,0.5,0.5,1) );
		drawBox3D (  hit-dp, hit+dp, Vec4F(1,0,0,1) );
	}
}


void Sample::RaycastDisplacementSphere ( Vec3F p, float r, float bump_depth, float dt, Vec3F lgt, ImageX* img )
{
	float t, t1, tx, diffuse, spec, cr, h;	
	float pu, pv, dpu, dpv, ndotL;
	Vec3F a,b,c;
	Vec3F hit, norm, V;
	Vec3F rpos, rdir;
	int raycost;	
	int xres = img->GetWidth();
	int yres = img->GetHeight();

	// maximal shell - expand sphere by bump depth	
	float r1 = r + bump_depth;

	lgt.Normalize();

	m_rand.seed ( m_samples );

	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {

			// get camera ray
			rpos = m_cam->getPos();
			rdir = m_cam->inverseRay ( x, y, xres, yres );	
			rdir.Normalize();

			// intersect with maximal height sphere
			if ( intersectRaySphere( rpos, rdir, p, r1, hit, norm, t ) ) {

				// [optional] monte-carlo sampling of t phase for filtering due to dt jumping over thin features.
				// converts striations near thin features into sampling noise
				t += m_rand.randF() * dt;

				hit = rpos + rdir * t;			

				// Ray marching 				
				h = r; 
				cr = r1;
				raycost = 0;
				for ( ; cr >= h && cr <= r1; ) {    // march until ray leaves displacement zone (hit or miss)

					hit += rdir * dt;                 // advance ray point sampling

					cr = (hit - p).Length();          // current ray point height above surface

					ComputeSphereUV ( p, hit, pu, pv ); // recover pu,pv (spherical coordinates)
					
					// sample bump map
					tx = m_bump_img->GetPixelUV ( pu, pv ).x; 									
					h = r + tx * bump_depth;          // displaced sphere height				

					raycost++;
				}

				// check for hit
				if (cr < h) {	

					t1 = (hit - rpos).Length();	
					
					ComputeSphereUV ( p, hit, pu, pv );

					// Normal for displaced sphere by neighbor sampling
					dpu = 1.0 / (m_bump_img->GetWidth()-1);
					dpv = 1.0 / (m_bump_img->GetHeight()-1);
					a = getSpherePnt(pu-dpu,pv-dpv) * (r + m_bump_img->GetPixelFilteredUV ( pu-dpu, pv-dpv ).x * bump_depth);
					b = getSpherePnt(pu+dpu,pv-dpv) * (r + m_bump_img->GetPixelFilteredUV ( pu+dpu, pv-dpv ).x * bump_depth);
					c = getSpherePnt(pu+dpu,pv+dpv) * (r + m_bump_img->GetPixelFilteredUV ( pu+dpu, pv+dpv ).x * bump_depth);

					norm = (c-a).Cross ( b-a ); norm.Normalize();
				
					// Shading
          V = (rpos - hit).Normalize();
          ndotL = norm.Dot( lgt );
					diffuse = 0.8f * std::max(0.0f, ndotL);
          spec = pow( std::max(0.0, (norm * (2.0f*ndotL) - lgt).Normalize().Dot ( V ) ), 20 );
          cr = std::min(1.0f, diffuse+spec);          

					img->SetPixel ( x, y, Vec4F(cr, cr, cr, 1.0) );          

					//-- visualize ray cost
					// img->SetPixel ( x, y, Vec4F(raycost*dt*255.0, 0, 0, 255.0) );   
				}
			}			
		}
	}

	m_samples++;

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_GLTEX );		
}


void Sample::DrawTesselatedSphereGL ( Vec3F p, float r, int res, float bump_depth )
{
	float cu, su, cu1, su1;
	float cv, sv, cv1, sv1;
	float pu, pv, dpu, dpv;
	Vec3F a, b, c, d, n;	
	Vec4F clr(1,1,1,1);	

	float s = 180.0f / res;	
	dpu = s / 360.0f;
	dpv = s / 180.0f;

	int faces = 0;
	for (float v=0; v <= 180; v+= s ) {
		for (float u=0; u < 360; u += s ) {
			cu = cos(u * DEGtoRAD); cv = cos(v * DEGtoRAD); su = sin(u * DEGtoRAD); sv = sin(v * DEGtoRAD);
			cu1 = cos((u+s) * DEGtoRAD); cv1 = cos((v+s) * DEGtoRAD); su1 = sin((u+s) * DEGtoRAD); sv1 = sin((v+s) * DEGtoRAD);
			a.Set ( cu *  sv,  cv,  su *  sv);		
			b.Set (cu1 *  sv,  cv, su1 *  sv);				
			c.Set (cu1 * sv1, cv1, su1 * sv1);			
			d.Set ( cu * sv1, cv1,  su * sv1);				

			// displace along normal direction
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

			drawFace3D( a, b, c, d, n, clr );
			faces++;
		}
	}	

}


void Sample::display()
{
	clearGL();
    
  if (m_update) {
    m_update = false;    

    // Camera update
    MoveCamera();

    // Displacement raycasting
    RaycastDisplacementSphere(Vec3F(0, 0, 0), 1.0, m_bump_depth, 0.08, m_lgt, m_out_img);    
	}

	// Draw geometric sphere in 3D
	start3D(m_cam);
	setLight3D ( m_lgt, Vec4F(1,1,1,1) );	
  setMaterial ( Vec3F(0,0,0), Vec3F(.8,.8,.8), Vec3F(1,1,1), 20, 1 );

	  DrawGrid();

	  DrawTesselatedSphereGL ( Vec3F(-3,0,0), 1.0, 200, m_bump_depth );	
	  
	  //DrawRayDemo ( 256, 256, Vec3F(0,0,0), 1.0, m_bump_depth, m_out_img ) ;

	end3D();		
	
	// Draw raycast image
	start2D(getWidth(), getHeight());	
	  drawImg ( m_out_img, Vec2F(0, 0), Vec2F(getWidth(), getHeight()), Vec4F(1,1,1,1) );		// draw raycast image 	
	end2D();	
	
  drawAll();										// complete 3D rendering to OpenGL

	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::MoveCamera()
{
	// clear the output image
	memset ( m_out_img->GetData(), 0, m_out_img->GetSize() );
	
	m_samples = 1;			// reset samples
	appPostRedisplay();		// update display
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
		Update();
	} break;

	case AppEnum::BUTTON_RIGHT: {

		// Adjust camera orbit 
		Vec3F angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam->SetOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());		
		m_samples = 0;
    Update();
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag

  if (state == AppEnum::BUTTON_RELEASE) {
    // updates end    
    m_update = false;    
    RaycastDisplacementSphere(Vec3F(0, 0, 0), 1.0, m_bump_depth, 0.01, m_lgt, m_out_img);
  }    
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
  Update();
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action==AppEnum::BUTTON_RELEASE) return;

}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

  if (m_cam==0x0) return;
	m_cam->setSize( w, h );
	m_cam->setAspect(float(w) / float(h));
	m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());

	appPostRedisplay();
}

void Sample::startup()
{
	int w = 1200, h = 700;
	appStart("Displacement Map Sphere", "Displacement Map Sphere", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}




