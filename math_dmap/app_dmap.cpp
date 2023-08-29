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

#define NO_UVTRI		-1e10
#define NO_MESH			 1e10

#define BUF_VOL			0

struct Vis {
	char type;
	Vector3DF a, b;
	Vector4DF clr;
};

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

	bool		ComputeDisplacement ( Image* img, int x, int y, float& d, Vector3DF& v, Vector3DF& vhit );
	void		ComputeDisplacement ( Image* img );
	void		SaveImage ( Image* img );
	float		getNeighbor (Image* img, int x, int y );

	void		Resize();
	void		UpdateCamera();
	void		MoveCamera ( char t, Vector3DF del );
	void		DrawGrid();
	void		DrawMesh( MeshX* m, Vector4DF clr, bool normals=false );
	void		DrawMeshUV ( MeshX* m, int w, int h, Vector4DF clr );

	void		VisDot ( Vector3DF p, float r, Vector3DF clr );
	void		VisLine ( Vector3DF a, Vector3DF b, Vector3DF clr );
	void		DrawVis ();

	Camera3D*	m_cam;				// camera
	Vector3DF   m_lgt;		

	MeshX*		m_mesh_src;			// high-res source 
	MeshX*		m_mesh_dest;		// low-res target

	float		m_limit;	

	Image*		m_displace_img;	
	Image*		m_view_img;
	Vector3DI	m_res;

	Mersenne	m_rand;

	Vector3DI	m_curr_pix;
	int			m_curr_tri;			// triangle cache (perf)
	int			m_curr_hit;

	float		m_time;
	int			mouse_down;
	bool		m_view_hires;
	bool		m_view_disp;
	bool		m_view_uvs;
	bool		m_view_norms;
	bool		m_run;

	char		m_venable;			// visualize enabled	
	std::vector<Vis> m_debugvis;	// debug visualizations	

};
Sample obj;


bool Sample::init()
{
	std::string fpath;

	addSearchPath(ASSET_PATH);
	addSearchPath( "G:\\Assets_Models" );

	init2D( "arial" );
	setText( .02, 1);
	
	m_time = 0;
	m_run = true;
	m_view_hires = false;
	m_view_disp = false;
	m_view_uvs = false;
	m_view_norms = false;
	m_curr_tri = 0;
	
	m_rand.seed ( 123 );

	// create a camera
	m_cam = new Camera3D;								
	m_cam->setFov ( 40 );
	m_cam->setAspect ( 1 );
	
	m_cam->setOrbit ( -44, 30, 0, Vector3DF(0, 0, 0), 15, 1 );

	m_lgt = Vector3DF(-200, 150, 150);

	// set depth limit
	m_limit = 3.0;

	// load lo-res target mesh
	//getFileLocation ( "surface.obj", fpath );
	getFileLocation ( "cube_smooth.obj", fpath );
	//getFileLocation ( "armadillo_lores3.obj", fpath );

	dbgprintf ( "Loading: %s\n", fpath.c_str());
	m_mesh_dest = new MeshX;
	if ( !m_mesh_dest->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-3);
	}
	if ( m_mesh_dest->GetVertTex(0) == 0x0 ) {
		dbgprintf ( "ERROR: Low res mesh does not have UVs.\n" );
		exit(-4);
	}

	// load hi-res source mesh
	//getFileLocation ( "surface_smooth.obj", fpath );
	//getFileLocation ( "golfball.obj", fpath );
	getFileLocation ( "sphere_iso.obj", fpath );
	//getFileLocation ( "sphere_uv.obj", fpath );
	//getFileLocation ( "armadillo_hires.obj", fpath );
	
	dbgprintf ( "Loading: %s\n", fpath.c_str());
	m_mesh_src = new MeshX;
	if ( !m_mesh_src->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-3);
	}

	// create displacement map (16-bit tiff)

	m_res.Set ( 512, 512, 0 );
	
	m_displace_img = new Image;
	m_displace_img->ResizeImage ( m_res.x, m_res.y, ImageOp::F32 );		
	memset ( m_displace_img->GetData(), 0, m_displace_img->GetSize() );
	m_displace_img->Commit ( DT_CPU | DT_GLTEX );

	m_view_img = new Image;
	m_view_img->ResizeImage ( m_res.x, m_res.y, ImageOp::RGB8 );		
	memset ( m_view_img->GetData(), 0, m_view_img->GetSize() );
	m_view_img->Commit ( DT_CPU | DT_GLTEX );
			
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

void Sample::DrawMesh( MeshX* m, Vector4DF clr, bool normals )
{
	Vector3DF n, V;
	Vector3DF v0, v1, v2;
	Vector3DF n0, n1, n2;
	Vector3DF uv0, uv1, uv2;
	AttrV3* f;
	int num_tri = m->GetNumElem ( BFACEV3 );
	
	for (int i = 0; i < num_tri; i++)  {

		f = (AttrV3*) m->GetElem (BFACEV3, i );
		
		v0 = *m->GetVertPos( f->v1 );	v1 = *m->GetVertPos( f->v2 );	v2 = *m->GetVertPos( f->v3 );		
		n0 = *m->GetVertNorm( f->v1 );	n1 = *m->GetVertNorm( f->v2 );	n2 = *m->GetVertNorm( f->v3 );		
	
		n = (v2-v1).Cross ( v0-v1 );
		n.Normalize ();
		V = m_cam->getPos() - v0;

		if ( n.Dot ( V ) >= 0 ) {
			// draw triangle
			drawLine3D ( v0, v1, clr ); 
			drawLine3D ( v1, v2, clr );	
			drawLine3D ( v2, v0, clr ); 

			if ( normals ) {
				drawLine3D ( v0, v0+n0*0.5f, Vector4DF(1,0,1,0.5) );
				drawLine3D ( v1, v1+n1*0.5f, Vector4DF(1,0,1,0.5) );
				drawLine3D ( v2, v2+n2*0.5f, Vector4DF(1,0,1,0.5) );
			}

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
		
		//uv0.y = 1-uv0.y; uv1.y = 1-uv1.y; uv2.y = 1-uv2.y;   // plot y+ upward

		// draw triangle		
		drawLine ( uv0*sz, uv1*sz, clr ); 
		drawLine ( uv1*sz, uv2*sz, clr );	
		drawLine ( uv2*sz, uv0*sz, clr ); 
	}
	drawLine ( Vector3DF(0, sz.y, 0), Vector3DF(m_curr_pix.x*sz.x/m_displace_img->GetWidth(), m_curr_pix.y*sz.y/m_displace_img->GetHeight(), 0), Vector4DF(0,0,1,1) );
}

void Sample::VisDot ( Vector3DF p, float r, Vector3DF clr )
{
	Vis v;
	v.type = 'p';
	v.a = p;
	v.b.x = r;
	v.clr = Vector4DF(clr, 1);
	m_debugvis.push_back(v);	
}

void Sample::VisLine ( Vector3DF a, Vector3DF b, Vector3DF clr )
{
	Vis v;
	v.type = 'l';
	v.a = a;
	v.b = b;
	v.clr = Vector4DF(clr, 1);
	m_debugvis.push_back(v);	
}


void Sample::DrawVis ()
{
	Vector3DF vb0, vb1, V;
	V = m_cam->getDir();
	V.Normalize();

	// Draw debug visualization
	float vscale = 0.001f;

	setview2D(getWidth(), getHeight());	
	for (int i=0; i < m_debugvis.size(); i++) {
		if ( m_debugvis[i].type == 'p' ) {
			vb0 = m_debugvis[i].a; vb1 = m_debugvis[i].b;
			drawCircle3D ( vb0, vb0+V, vscale * vb1.x, m_debugvis[i].clr );
		}
	}
	for (int i=0; i < m_debugvis.size(); i++) {
		if ( m_debugvis[i].type == 'l' ) {
			vb0 = m_debugvis[i].a; vb1 = m_debugvis[i].b;
			drawLine3D ( vb0, vb1, m_debugvis[i].clr );
		}
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

	// if ( t<0.0 ) return false;

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

	float tpos =  1e10;	
	float tneg = -1e10;
	t = 0;

	// search all tris
	fhit = -1;
	for (int i = 0; i < num_tri; i++)  {
		f = (AttrV3*) m_mesh_src->GetElem (BFACEV3, i );		
		v0 = *m_mesh_src->GetVertPos( f->v1 );		v1 = *m_mesh_src->GetVertPos( f->v2 );		v2 = *m_mesh_src->GetVertPos( f->v3 );			

		//if ( intersectLineBox ( rpos, rdir, s->bmin, s->bmax, t ) ) 

		if ( intersect_tri ( rpos, rdir, v0, v1, v2, t, a, b ) ) {
			
			if ( t > 0 && t < m_limit && t < tpos ) tpos = t;
			if ( t < 0 && t >-m_limit && t > tneg ) tneg = t;
		}
	}
	t = 1e10;
	if ( tneg != -1e10 ) t = tneg;
	if ( tpos !=  1e10 && tpos < fabs(t) ) 
		t = tpos;
	
	return (t != 1e10);
	
}

float sign (Vector3DF p1, Vector3DF p2, Vector3DF p3)
{
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool pointInTriangle3D (Vector3DF pt, Vector3DF v0, Vector3DF v1, Vector3DF v2, float& s, float& t)
{
	v0 -= v2;
	v1 -= v2;
	v2 = pt - v2;

	float e00 = v0.Dot (v0);
	float e01 = v0.Dot (v1);
	float e02 = v0.Dot (v2);
	float e11 = v1.Dot (v1);
	float e12 = v1.Dot (v2);
	float invd = 1.0/( e00 * e11 - e01 * e01);
	float w;
	s = invd * (e11 * e02 - e01 * e12);
	t = invd * (e00 * e12 - e01 * e02);
	//w = (1-w-t);

    if ( s >=0 && t >= 0 && s+t < 1 ) 
		return true;

	return false;
}

bool pointInTriangle2D (Vector2DF& pt, Vector2DF& v1, Vector2DF& v2, Vector2DF& v0, float& s, float& t)
{
	s = v0.y*v2.x - v0.x*v2.y + (v2.y-v0.y)*pt.x + (v0.x-v2.x)*pt.y;
	t = v0.x*v1.y - v0.y*v1.x + (v0.y-v1.y)*pt.x + (v1.x-v0.x)*pt.y;		
	if ( s < 0 || t < 0 ) return false;
	float d = -v1.y*v2.x + v0.y*(-v1.x+v2.x) + v0.x*(v1.y-v2.y) + v1.x*v2.y;
	if (d < .00001) {
		//printf ("ERROR: degenerate triangle <%f,%f> <%f,%f> <%f,%f>\n", v0.x, v0.y, v1.x, v1.y, v2.x, v2.y );
		return false;
	}
	s /= d;
	t /= d; 
	if (s >= 0 && t >= 0 && s+t < 1)
		return true;

    return false;
}

bool Sample::ComputeDisplacement ( Image* img, int x, int y, float& d, Vector3DF& v, Vector3DF& vhit )
{
	bool found = false;
	int fhit;
	float a, b, t;
	Vector2DF uv0, uv1, uv2, uvchk;
	Vector3DF n0, n1, n2, n, na, nb;
	Vector3DF v0, v1, v2;
	Vector3DF p;
	AttrV3* f;
	int num_tri = m_mesh_dest->GetNumElem ( BFACEV3 );

	//--- get UV of current pixel
	// make sure we sample center of pixel 
	Vector2DF uv;
	uv = Vector2DF (float(x+0.5) / img->GetWidth(), float(y+0.5) / img->GetHeight() );

	//--- find triangle containing UV	
	//
	//- attempt current (cache) triangle first for performance	
	f = (AttrV3*) m_mesh_dest->GetElem (BFACEV3, m_curr_tri );		
	uv0 = *m_mesh_dest->GetVertTex( f->v1 );	uv1 = *m_mesh_dest->GetVertTex( f->v2 );	uv2 = *m_mesh_dest->GetVertTex( f->v3 );

	if ( !pointInTriangle2D (uv, uv0, uv1, uv2, a, b ) ) {	

		//- search all triangles
		for (int i = 0; i < num_tri && !found; i++)  {
			f = (AttrV3*) m_mesh_dest->GetElem (BFACEV3, i );		
			uv0 = *m_mesh_dest->GetVertTex( f->v1 );	uv1 = *m_mesh_dest->GetVertTex( f->v2 );	uv2 = *m_mesh_dest->GetVertTex( f->v3 );	
			if ( pointInTriangle2D (uv, uv0, uv1, uv2, a, b ) ) {			
				m_curr_tri = i;
				found = true;
			}
		}
	} else {
		found = true;
	}
	

	if ( found ) {

		//--- compute *smooth* normal	
		f = (AttrV3*) m_mesh_dest->GetElem (BFACEV3, m_curr_tri );
		n0 = *m_mesh_dest->GetVertNorm( f->v1 );	n1 = *m_mesh_dest->GetVertNorm( f->v2 );	 n2 = *m_mesh_dest->GetVertNorm( f->v3 );	
		n = n0 * a +  n1 * b + n2 * (1-a-b);		
		n.Normalize();
	
		v0 = *m_mesh_dest->GetVertPos( f->v1 );		v1 = *m_mesh_dest->GetVertPos( f->v2 );		v2 = *m_mesh_dest->GetVertPos( f->v3 );
		v = v0 * a + v1 * b + v2 * (1-a-b);		

		//--- uv check
		// the uvs from computed barycentric coordinates should match input uvs from pixel
		/*uv0 = *m_mesh_dest->GetVertTex( f->v1 );	uv1 = *m_mesh_dest->GetVertTex( f->v2 );	uv2 = *m_mesh_dest->GetVertTex( f->v3 );
		uvchk= uv0* a + uv1* b + uv2* (1-a-b);
		printf ( "%f,%f -> %f,%f\n", uv.x, uv.y, uvchk.x, uvchk.y ); */

		//--- intersect normal with high-res mesh

		if ( intersect_mesh ( v, n, t, fhit ) ) {

			// mesh hit
			d = t;
			vhit = v + n * t;	// hit point
			return true;
		
		} else {
			// no mesh intersection
			d = NO_MESH;
			v = 0; 
			vhit = 0;
		}
	} else {
		// uv not found in triangle
		d = NO_UVTRI;
	}
	return false;

}

float Sample::getNeighbor (Image* img, int x, int y )
{
	if ( x < 0 || x >= img->GetWidth() ) return NO_UVTRI;
	if ( y < 0 || y >= img->GetHeight()) return NO_UVTRI;
	
	return img->GetPixelF(x,y);
}

void Sample::SaveImage ( Image* img )
{
	float d, dmin=0, dmax=0;
	float n, nbr[4];
	int cnt;
	int w = img->GetWidth();
	int h = img->GetHeight();

	Image* tempimg = new Image;
	tempimg->ResizeImage ( w, h, ImageOp::F32 );
	
	// normalize the displacement map
	for (int y=0; y < h; y++) {
		for (int x=0; x < w; x++) {
			d = img->GetPixelF( x, y );
			if ( d > NO_UVTRI && d < NO_MESH) {
				if ( d < dmin ) dmin = d;
				if ( d > dmax ) dmax = d;
			}
		}
	}

	// expand border pixels 
	for (int iter=0; iter < 4; iter++) {

		for (int y=0; y < h; y++) {
			for (int x=0; x < w; x++) {

				d = img->GetPixelF( x, y );

				// only affect invalid (out-of-bounds) pixels..
				if ( d <= NO_UVTRI) {			
					// evaluate neighbors
					nbr[0] = getNeighbor( img, x-1, y );
					nbr[1] = getNeighbor( img, x+1, y );
					nbr[2] = getNeighbor( img, x, y-1 );
					nbr[3] = getNeighbor( img, x, y+1 );
					// count and sum only valid neighbors
					n = 0; cnt = 0;
					for (int j=0; j<4; j++) {
						if (nbr[j] > NO_UVTRI) {
							n += nbr[j];
							cnt++;	
						}
					}	
					// convert no_uvtri pixel to valid depth
					if (cnt > 0) 
						d = n / cnt;						
				}
				tempimg->SetPixelF( x, y, d );
			}
		}
		// swap image
		memcpy ( img->GetData(), tempimg->GetData(), img->GetSize() );
	}

	// convert any remaining invalid pixels to 0
	for (int y=0; y < h; y++) {
		for (int x=0; x < w; x++) {
			d = img->GetPixelF( x, y );
			if (d <= NO_UVTRI) 
				img->SetPixelF( x, y, 0 );
			else
				bool stop=true;
		}
	}

	// create 16-bit TIF
	Image* outimg = new Image;
	outimg->ResizeImage ( w, h, ImageOp::BW16 );
		
	for (int y=0; y < h; y++) {
		for (int x=0; x < w; x++) {
			d = img->GetPixelF( x, y );
			if (d <= NO_UVTRI) {
				printf ( "ERROR: Invalid pixels remaining.\n");
				exit(-7);
			}
			d = (d - dmin) * 65535 / (dmax - dmin);
			outimg->SetPixel16 ( x, y, uint16_t( d ) );
		}
	}
	// save it 
	outimg->Save ( "out.tif" );	

	printf ( "Saved. dmin=%f, dmax= %f\n", dmin, dmax );

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
			 ComputeDisplacement ( img, x, y, d, v, vhit );
			
			img->SetPixel16 ( x, h-y, d * 65535.0 );
		}
	}
	img->Commit ( DT_CPU | DT_GLTEX );
}


void Sample::display()
{
	Vector3DI c;	
	float d;
	Vector3DF v, vhit;
	clearGL();

	if (m_run) {
		//m_run = false;

		clock_t t1,t2;
		int iw = m_displace_img->GetWidth();
		int ih = m_displace_img->GetHeight();

		// Compute displacement
		if ( ComputeDisplacement ( m_displace_img, m_curr_pix.x, m_curr_pix.y, d, v, vhit ) ) {

			// write displacement value
			m_displace_img->SetPixelF ( m_curr_pix.x, m_curr_pix.y, d );

			// visualization
			c.x = 255 * (d / m_limit); if (c.x<0) c.x = 0;
			m_view_img->SetPixel ( m_curr_pix.x, m_curr_pix.y, c.x );
			
			//VisDot  ( vhit, 0.5, Vector4DF(1,0,1,1 ));
			//VisLine ( v, vhit, Vector4DF(1,1,0,1 ));

		} else {
			// no uv triangle or no hires hit
			if (d <= NO_UVTRI) 
				m_displace_img->SetPixelF ( m_curr_pix.x, m_curr_pix.y, d );	// write no triangle

			// visualization
			c = (d <= NO_UVTRI) ? Vector3DI(0,0,160) : Vector3DI(160,0,0);
			m_view_img->SetPixel ( m_curr_pix.x, m_curr_pix.y, c.x, c.y, c.z );
		}
		m_view_img->Commit ( DT_CPU | DT_GLTEX );

		if (++m_curr_pix.x >= iw ) {
			m_curr_pix.x = 0;
			
			//m_debugvis.clear ();

			if (++m_curr_pix.y >= ih ) {			
				// Done!
				SaveImage ( m_displace_img );
				m_curr_pix.Set(0,0,0);	
			}
		}		
	}

	// Draw grid
	start3D(m_cam);
		setview2D(getWidth(), getHeight());
		
		drawLine3D ( v, vhit, Vector4DF(1,1,0,1) );

		DrawGrid();
	
		// Draw low-res mesh
		DrawMesh( m_mesh_dest, Vector4DF(1,1,1,1), m_view_norms );

		// Draw hi-res mesh
		if (m_view_hires)
			DrawMesh( m_mesh_src, Vector4DF(0,1,1,0.2) );

		// Draw debug vis 
		DrawVis ();
		
	
	end3D();		
	draw3D();

	// Draw displacement map
	start2D();
		setview2D(getWidth(), getHeight());		
		
		int w = 512, h = 512;
		if (m_view_disp) {w = getWidth(); h = getHeight();}			

		drawImg ( m_view_img->getGLID(), 0, 0, w, h, 1,1,1,1 );

		if (m_view_uvs) 
			DrawMeshUV ( m_mesh_dest, w, h, Vector4DF(.5,.5,.5, .5) );
		
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

	case 'm':
		m_view_uvs = !m_view_uvs;
		break;

	case 'n':
		m_view_norms = !m_view_norms;
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




