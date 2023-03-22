//--------------------------------------------------------
// JUST MATH:
// Displacement Map Generator
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
#include <time.h>

#include "dataptr.h"
#include "geom_helper.h"
#include "mesh.h"

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

	bool		intersect_tri ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& t, float& alpha, float& beta );
	bool		intersect_mesh ( Vector3DF rpos, Vector3DF rdir, float& t, int& fhit ) ;

	float		ComputeDisplacement ( Image* img, int x, int y, Vector3DF& v, Vector3DF& vhit );
	void		ComputeDisplacement ( Image* img );
	void		SaveImage ( Image* img );

	void		Resize();
	void		UpdateCamera();
	void		MoveCamera ( char t, Vector3DF del );
	void		DrawGrid();
	void		DrawMesh( MeshX* m, Vector4DF clr );
	void		DrawMeshUV ( MeshX* m, int w, int h, Vector4DF clr );

	Camera3D*	m_cam;				// camera
	Vector3DF   m_lgt;		

	MeshX*		m_mesh_src;			// high-res source 
	MeshX*		m_mesh_dest;		// low-res target

	Image*		m_displace_img;	
	Vector3DI	m_res;

	Mersenne	m_rand;

	Vector3DI	m_curr_pix;
	int			m_curr_tri;			// triangle cache (perf)
	int			m_curr_hit;

	float		m_displace_depth;
	float		m_time;
	int			mouse_down;
	bool		m_view_hires;
	bool		m_view_disp;
	bool		m_run;

};
Sample obj;


bool Sample::init()
{
	std::string fpath;

	addSearchPath(ASSET_PATH);
	init2D( "arial" );
	setText( .02, 1);
	
	m_time = 0;
	m_view_hires = false;
	m_view_disp = false;
	m_curr_tri = 0;
	
	m_rand.seed ( 123 );

	// create a camera
	m_cam = new Camera3D;								
	m_cam->setFov ( 40 );
	m_cam->setAspect ( 1 );
	
	m_cam->setOrbit ( -44, 30, 0, Vector3DF(0, 0, 0), 15, 1 );

	m_lgt = Vector3DF(-200, 150, 150);

	// set bump depth
	m_displace_depth = 0.20;

	// load hi-res source mesh
	getFileLocation ( "iso_sphere.obj", fpath );
	//getFileLocation ( "uv_sphere.obj", fpath );
	
	dbgprintf ( "Loading: %s\n", fpath.c_str());
	m_mesh_src = new MeshX;
	if ( !m_mesh_src->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-3);
	}


	// load lo-res target mesh
	getFileLocation ( "cube.obj", fpath );

	dbgprintf ( "Loading: %s\n", fpath.c_str());
	m_mesh_dest = new MeshX;
	if ( !m_mesh_dest->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-3);
	}


	// create displacement map (16-bit tiff)

	m_res.Set ( 256, 256, 0 );
	
	m_displace_img = new Image;
	m_displace_img->ResizeImage ( m_res.x, m_res.y, ImageOp::F32 );		

	memset ( m_displace_img->GetData(), 0, m_displace_img->GetSize() );

	m_displace_img->Commit ( DT_CPU | DT_GLTEX );
			
	UpdateCamera();

	m_curr_pix.Set(0,0,0);

	return true;
}

void Sample::DrawGrid ()
{
	for (int i = -10; i <= 10; i++) {
		drawLine3D(float(i),-0.01f, -10.f, float(i), -0.01f, 10.f, .2f, .2f, .2f, 1.f);
		drawLine3D(-10.f,	-0.01f, float(i), 10.f, -0.01f, float(i), .2f, .2f, .2f, 1.f);
	}	
}

void Sample::DrawMesh( MeshX* m, Vector4DF clr )
{
	Vector3DF n, V;
	Vector3DF v0, v1, v2;
	Vector3DF uv0, uv1, uv2;
	AttrV3* f;
	int num_tri = m->GetNumElem ( BFACEV3 );
	
	for (int i = 0; i < num_tri; i++)  {

		f = (AttrV3*) m->GetElem (BFACEV3, i );
		
		v0 = *m->GetVertPos( f->v1 );	v1 = *m->GetVertPos( f->v2 );	v2 = *m->GetVertPos( f->v3 );		
	
		n = (v2-v1).Cross ( v0-v1 );
		n.Normalize ();
		V = m_cam->getPos() - v0;

		if ( n.Dot ( V ) >= 0 ) {
			// draw triangle
			drawLine3D ( v0, v1, clr ); 
			drawLine3D ( v1, v2, clr );	
			drawLine3D ( v2, v0, clr ); 			
		} 	
					
	}	
}

void Sample::DrawMeshUV ( MeshX* m, int w, int h, Vector4DF clr )
{
	Vector3DF uv0, uv1, uv2;
	AttrV3* f;
	int num_tri = m->GetNumElem ( BFACEV3 );

	Vector3DF sz (w, h, 0);

	for (int i = 0; i < num_tri; i++)  {

		f = (AttrV3*) m->GetElem (BFACEV3, i );
		
		uv0 = *m->GetVertTex( f->v1 );	uv1 = *m->GetVertTex( f->v2 );	uv2 = *m->GetVertTex( f->v3 );
		uv0.y = 1-uv0.y; uv1.y = 1-uv1.y; uv2.y = 1-uv2.y;   // plot y+ upward

		// draw triangle		
		drawLine ( uv0*sz, uv1*sz, clr ); 
		drawLine ( uv1*sz, uv2*sz, clr );	
		drawLine ( uv2*sz, uv0*sz, clr ); 
	}
}

void Sample::Resize ()
{

}


bool Sample::intersect_tri ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& t, float& alpha, float& beta )
{
	Vector3DF e0 = v1 - v0;
    Vector3DF e1 = v0 - v2;
    Vector3DF n = e1.Cross ( e0 );
	float ndotr = n.Dot( dir );	
	Vector3DF e2 = (v0 - orig) / ndotr;
	t = n.Dot ( e2 );
	if ( t<0.0 ) return false;

	Vector3DF i = dir.Cross ( e2 );
	alpha =	i.x*e0.x + i.y*e0.y + i.z*e0.z;	
	beta =	i.x*e1.x + i.y*e1.y + i.z*e1.z;		

	return (alpha >= 0 && beta >= 0 && (alpha+beta)<=1);
}

bool Sample::intersect_mesh ( Vector3DF rpos, Vector3DF rdir, float& t, int& fhit ) 
{
	Vector3DF n0, n1, n2;
	Vector3DF v0, v1, v2;
	AttrV3* f;
	float a, b;	
	int num_tri = m_mesh_src->GetNumElem ( BFACEV3 );

	// attempt cached tri frist
	f = (AttrV3*) m_mesh_src->GetElem (BFACEV3, m_curr_hit );		
	v0 = *m_mesh_src->GetVertPos( f->v1 );		v1 = *m_mesh_src->GetVertPos( f->v2 );		v2 = *m_mesh_src->GetVertPos( f->v3 );			
	if ( intersect_tri ( rpos, rdir, v0, v1, v2, t, a, b ) ) {
		fhit = m_curr_hit;
		return true;
	}
	// search all tris
	for (int i = 0; i < num_tri; i++)  {
		f = (AttrV3*) m_mesh_src->GetElem (BFACEV3, i );		
		v0 = *m_mesh_src->GetVertPos( f->v1 );		v1 = *m_mesh_src->GetVertPos( f->v2 );		v2 = *m_mesh_src->GetVertPos( f->v3 );			

		if ( intersect_tri ( rpos, rdir, v0, v1, v2, t, a, b ) ) {
			m_curr_hit = i;
			fhit = i;
			return true;
		}
	}
	return false;
}

float sign (Vector3DF p1, Vector3DF p2, Vector3DF p3)
{
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool pointInTriangle (Vector3DF pt, Vector3DF p0, Vector3DF p1, Vector3DF p2, float& s, float& t)
{
	float area = 0.5*(-p1.y*p2.x + p0.y*(-p1.x+p2.x) + p0.x*(p1.y-p2.y) + p1.x*p2.y);
	float w;
	w = (0.5/area) * (p0.x*p1.y - p0.y*p1.x + (p0.y-p1.y)*pt.x + (p1.x - p0.x)*pt.y );
	t = (0.5/area) * (p0.y*p2.x - p0.x*p2.y + (p2.y-p0.y)*pt.x + (p0.x - p2.x)*pt.y );	
	s = (1-w-t);

    if ( s >=0 && s <=1 && t >= 0 && t <= 1 && (s+t) <= 1 ) 
		return true;

	return false;
}

float Sample::ComputeDisplacement ( Image* img, int x, int y, Vector3DF& v, Vector3DF& vhit )
{
	bool found = false;
	int fhit;
	float a, b, d, t;
	Vector3DF uv0, uv1, uv2;
	Vector3DF n0, n1, n2, n;
	Vector3DF v0, v1, v2;
	Vector3DF p;
	AttrV3* f;
	int num_tri = m_mesh_dest->GetNumElem ( BFACEV3 );

	Vector3DF uv (float(x) / img->GetWidth(), float(y) / img->GetHeight(), 0);

	//--- find triangle containing UV	
	//
	//- attempt current (cache) triangle first for performance	
	f = (AttrV3*) m_mesh_dest->GetElem (BFACEV3, m_curr_tri );		
	uv0 = *m_mesh_dest->GetVertTex( f->v1 );	uv1 = *m_mesh_dest->GetVertTex( f->v2 );	uv2 = *m_mesh_dest->GetVertTex( f->v3 );	

	if ( !pointInTriangle (uv, uv0, uv1, uv2, a, b ) ) {	
		//- search all triangles
		for (int i = 0; i < num_tri && !found; i++)  {
			f = (AttrV3*) m_mesh_dest->GetElem (BFACEV3, i );		
			uv0 = *m_mesh_dest->GetVertTex( f->v1 );	uv1 = *m_mesh_dest->GetVertTex( f->v2 );	uv2 = *m_mesh_dest->GetVertTex( f->v3 );	
			if ( pointInTriangle (uv, uv0, uv1, uv2, a, b ) ) {			
				m_curr_tri = i;
				found = true;
			}
		}
	} else {
		found = true;
	}
	
	d = 0;

	if ( found ) {

		//--- compute *smooth* normal	
		n0 = *m_mesh_dest->GetVertNorm( f->v1 );	n1 = *m_mesh_dest->GetVertNorm( f->v2 );	 n2 = *m_mesh_dest->GetVertNorm( f->v3 );	
		n = n0 * a + n1 * b + n2 * (1-(a+b));
		n.Normalize();
	
		v0 = *m_mesh_dest->GetVertPos( f->v1 );		v1 = *m_mesh_dest->GetVertPos( f->v2 );		v2 = *m_mesh_dest->GetVertPos( f->v3 );		
		v = v0 * a + v1 * b + v2 * (1-(a+b));

		//--- intersect normal with high-res mesh

		if ( intersect_mesh ( v, n, t, fhit ) && t > 0 ) {

			d = t;

			vhit = v + n * t;	// hit point
		}
	} 

	if ( d < 0 || d > 2 ) {
		dbgprintf ("ERROR: d = %f\n", d );
	}
	
	img->SetPixelF ( x, (img->GetHeight()-1)-y, d );
	
	return d;
}


void Sample::SaveImage ( Image* img )
{
	float d, dmax=0;
	int w = img->GetWidth();
	int h = img->GetHeight();
	
	// normalize the displacement map
	for (int y=0; y < h; y++) {
		for (int x=0; x < w; x++) {
			d = img->GetPixelF( x, y );
			if ( d > dmax ) dmax = d;
		}
	}


	// create 16-bit TIF
	Image* outimg = new Image;
	outimg->ResizeImage ( w, h, ImageOp::BW16 );
		
	for (int y=0; y < h; y++) {
		for (int x=0; x < w; x++) {
			d = img->GetPixelF( x, y );
			d = d * 65535 / dmax;
			outimg->SetPixel16 ( x, y, d );			
		}
	}
	// save it 
	outimg->Save ( "out.tif" );	

	printf ( "Saved. max depth = %f\n", dmax );

	delete outimg;
}


void Sample::ComputeDisplacement ( Image* img )
{
	Vector3DF v, vhit;
	int w = img->GetWidth();
	int h = img->GetHeight();
	float d;	

	for (int y=0; y < h; y++) {
		for (int x=0; x < w; x++) {
			d = ComputeDisplacement ( img, x, y, v, vhit );
			
			img->SetPixel16 ( x, h-y, d * 65535.0 );
		}
	}
	img->Commit ( DT_CPU | DT_GLTEX );
}


void Sample::display()
{
	Vector3DF v, vhit;
	clearGL();

	if (m_run) {
		//m_run = false;

		clock_t t1,t2;
		int iw = m_displace_img->GetWidth();
		int ih = m_displace_img->GetHeight();

		// Compute displacement
		ComputeDisplacement ( m_displace_img, m_curr_pix.x, m_curr_pix.y, v, vhit );

		if (++m_curr_pix.x >= iw ) {
			m_curr_pix.x = 0;
			if (++m_curr_pix.y >= ih ) {			
				// Done!
				SaveImage ( m_displace_img );
				m_curr_pix.Set(0,0,0);	
			}
		}
		m_displace_img->Commit ( DT_CPU | DT_GLTEX );
	}

	// Draw grid
	start3D(m_cam);
		setview2D(getWidth(), getHeight());
		
		drawLine3D ( v, vhit, Vector4DF(1,1,0,1) );

		DrawGrid();
	
		// Draw low-res mesh
		DrawMesh( m_mesh_dest, Vector4DF(1,1,1,1) );

		// Draw hi-res mesh
		if (m_view_hires)
			DrawMesh( m_mesh_src, Vector4DF(0,1,1,1) );

		// Draw current solution point & normal
		
	
	end3D();		
	draw3D();

	// Draw displacement map
	start2D();
		setview2D(getWidth(), getHeight());		
		
		int w = 512, h = 512;
		if (m_view_disp) {w = getWidth(); h = getHeight();}			

		drawImg ( m_displace_img->getGLID(), 0, 0, w, h, 1,1,1,1 );

		DrawMeshUV ( m_mesh_dest, w, h, Vector4DF(1,1,1,1) );
		
	end2D();	
	draw2D();	

	

	appPostRedisplay();	
}

void Sample::UpdateCamera()
{
	Vector3DF a, t;
	a = m_cam->getAng();
	t = m_cam->getToPos();
	// dbgprintf ( "angs: %3.4f %3.4f %3.4f, to: %3.4f %3.4f %3.4f\n", a.x,a.y,a.z, t.x,t.y,t.z );		

	appPostRedisplay();		// update display
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
	float fine = 0.5;
	

	switch (mouse_down) {
	case AppEnum::BUTTON_LEFT: {
	} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
		m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
		UpdateCamera();
	} break;

	case AppEnum::BUTTON_RIGHT: {

		// Adjust camera orbit 
		Vector3DF angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam->setOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());		
		m_cam->setOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());		
		UpdateCamera();
		
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	if (guiHandler(button, state, x, y)) return;
	
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag

	if ( mouse_down==AppEnum::BUTTON_LEFT) {
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

	m_cam->setOrbit( m_cam->getAng(), m_cam->getToPos(), dist, dolly);
	m_cam->setOrbit( m_cam->getAng(), m_cam->getToPos(), dist, dolly);

	UpdateCamera();
}


void Sample::MoveCamera ( char t, Vector3DF del )
{
	switch (t) {
	case 'p': {
		float orbit = m_cam->getOrbitDist() - del.z;
		m_cam->setOrbit( m_cam->getAng(), m_cam->getToPos(), orbit, m_cam->getDolly());
		UpdateCamera();
		} break;
	case 't':
		m_cam->moveRelative(float(del.x) * m_cam->getOrbitDist() / 1000, float(-del.y) * m_cam->getOrbitDist() / 1000, 0);	
		UpdateCamera();
		break;
	}
}

void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action==AppEnum::BUTTON_RELEASE) return;

	float s = (mods==1) ? 10.0 : 1.0;

	switch (keycode) {

	case ' ':
		m_run = !m_run;
		break;

	case 'h':
		m_view_hires = !m_view_hires;
		break;

	case 'd':
		m_view_disp = !m_view_disp;
		break;


	case 'w': case 'W':		MoveCamera('p', Vector3DF( 0, 0,+s)); break;		// WASD navigation keys
	case 's': case 'S':		MoveCamera('p', Vector3DF( 0, 0,-s)); break;

	/*case 'a': case 'A':		MoveCamera('t', Vector3DF(-s, 0, 0));	break;	
	case 'd': case 'D':		MoveCamera('t', Vector3DF(+s, 0, 0));	break;
	case 'q': case 'Q':		MoveCamera('t', Vector3DF(0, -s, 0));	break;
	case 'z': case 'Z':		MoveCamera('t', Vector3DF(0, +s, 0));	break;*/

	}
}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

	m_cam->setSize( w, h );
	m_cam->setAspect(float(w) / float(h));
	m_cam->setOrbit( m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
	m_cam->updateMatricies();

	appPostRedisplay();
}

void Sample::startup()
{
	int w = 2048, h = 2048;
	appStart("Displacement Map Generator", "Displacement Map Generator", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}




