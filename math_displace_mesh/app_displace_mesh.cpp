//--------------------------------------------------------
// JUST MATH:
// Displace Mesh - raycast displacement mapping on a triangle mesh
//
// Inspired by the papers:
//    Shell Maps, D.Porumbescu et al, 2005
//    Interactive Smooth and Curved Shell Mapping, S. Jeschke et al, 2007
//    Tesselation-Free Displacement Mapping for Raytracing, Thonat et al, 2021
//
// This sample implements tesselation-free displacement mapping on CPU.
//
// A key observation in the above papers is that, while sampling along a 
// ray in world space is linear the projection of those samples down to a base triangle
// is non-linear in barycentric coordinates. Identifying the correct UV samples to 
// evaluate the bump surface for comparison to ray height is an interesting challenge.
// Ray marching over prism geometry gives true silhouettes with deep displacements.
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
#include "geom_helper.h"
#include "mesh.h"

#define BUF_VOL			0

// prism geometry
struct Prism {
	Prism()	{};
	Vector3DF vb0,vb1,vb2;
	Vector3DF ve0,ve1,ve2;			// prism volume, see defines above
	Vector3DF bmin, bmax;			// bounding box of prism
	Vector3DF norm;					// normal of facing triangle
	float	  dmin, dmax;
};

// hit candidate info
struct hitinfo_t {					
	hitinfo_t (int f, float t0, float t1, int e0, int e1) {face=f; tmin=t0; tmax=t1; emin=e0; emax=e1;}
	int	face;	
	float tmin, tmax;		
	int emin, emax;
};

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

	bool		intersect_ray_patch (Vector3DF rpos, Vector3DF rdir, Vector3DF q00, Vector3DF q10, Vector3DF q11, Vector3DF q01, float& tmin, float& tmax, int e, int& emin, int& emax );
	bool		intersect_tri_inout ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& tmin, float& tmax, int e, int& emin, int& emax );
	bool		intersect_ray_prism ( Vector3DF rpos, Vector3DF rdir, Vector3DF vb0, Vector3DF vb1, Vector3DF vb2, Vector3DF ve0, Vector3DF ve1, Vector3DF ve2, 
										float& tmin, float& tmax, int& emin, int& emax);

	Vector3DF	projectPointInPrism ( Vector3DF p, Vector3DF v0, Vector3DF v1, Vector3DF v2,
										     Vector3DF vn0, Vector3DF vn1, Vector3DF vn2, Vector3DF& q );

	void		ConstructPrisms ();
	bool		FindPrismHitCandidates ( Vector3DF rpos, Vector3DF rdir, std::vector<hitinfo_t>& candidates );
	bool		IntersectSurface	( int face, Vector3DF rpos, Vector3DF rdir, float tmin, float tmax, int emin, int emax, float& t, Vector3DF& bc, float dt );
	bool		ShadeSurface		( int face, Vector3DF rpos, Vector3DF rdir, float tmin, float tmax, int emin, int emax, float t, Vector3DF bc, Vector3DF& n, Vector3DF& clr );

	float		getBumpAmt ( Vector3DF bc, Vector3DF uv, float t=0, float tmin=0, float tmax=0, int emin=0, int emax=0);
	void		ComputeBumpAdjust	( int face, Vector3DF& bw, Vector3DF& bcp, Vector3DF rpos, Vector3DF rdir, float dt, float tmin, float tmax, int emin, int emax );

	void		RaytraceMesh ( float bump_depth, Image* img );
	void		RaytraceStart ();
	void		RaytraceDisplacementMesh ( Image* img );	
	void		RaytraceDebug ( Vector3DF from, Vector3DF to, int x, int y, Image* img );	

	void		Resize ();
	void		DrawGrid ();	
	void		DrawMeshUV ( MeshX* m, int w, int h, Vector4DF clr );
	void		UpdateCamera();
	void		MoveCamera ( char t, Vector3DF del );

	void		VisDot ( Vector3DF p, float r, Vector3DF clr );
	void		VisLine ( Vector3DF a, Vector3DF b, Vector3DF clr );
	void		VisTriangleSurface ( int face );

	Camera3D*	m_cam;				// camera
	Vector3DF   m_lgt;
	Image*		m_bump_img;			// bump map
	Image*		m_color_img;		// color map
	Image*		m_out_img;	

	MeshX*		m_mesh;				// mesh geometry

	std::vector<Prism>	m_prisms;	// prism geometry

	float		m_bamt[4];			// interpolation correction

	Mersenne	m_rand;

	Vector4DF   m_displace;	
	float		m_time;
	int			m_samples;
	bool		m_run, m_show_map;
	bool		m_jitter_sample, m_draw_prisms;
	int			mouse_down;

	char		m_venable;			// visualize enabled
	char		m_vcurr;
	std::vector<Vis> m_debugvis;	// debug visualizations	
	Vector3DI	m_vpix;				// pixel to check	
	Vector3DF	m_vfrom, m_vto;		// vis camera specs
	Vector3DF	m_vhit;				// vis hit color
	Vector3DF	m_vscan;			// vis scan tri color
	Vector3DF	m_vsample;			// vis sample color
	Vector3DF	m_vbase;			// vis base color
	Vector3DF	m_vsurf;			// vis surface color

	int			m_res;
	int			m_yline;
	
	

	int			m_currcam;
};
Sample obj;


bool Sample::init()
{
	std::string fpath;		

	addSearchPath(ASSET_PATH);
	addSearchPath( "G:\\Assets_Models" );
	addSearchPath( "G:\\Assets_CC0" );
	init2D( "arial" );
	setText( .02, 1);
	
	m_time = 0;
	m_run = false;	
	m_venable = 0;
	m_show_map = false;
	m_draw_prisms = false;
	m_out_img = 0;

	m_rand.seed ( 123 );


	m_res = 256;
	m_yline = 0;

	// set bump depth
	
	//-- cube-sphere
	//m_displace.Set ( 0.0, 0.816238, 1.5, 0);

	//m_displace.Set (0.0, 0.5, 1.0, 1.0);

	//-- armadillo
	//m_displace.Set( 0.5, 0.6, -0.3);

	//m_displace.Set( 0.0, 0.322081, 0, 1);		// surface

	m_displace.Set( 0.0, 0.2, 0, 1);	

	//m_displace.Set( -0.3, 0.3, -0.5, +0.5 );				// armadillo
	// m_displace.Set( -0.036492, 0.269047, -0.5, +0.5 );	// armadillo 2

	// m_displace.Set( -0.117898, 0.299104, -0.5, +0.5 );		// armadillo 3

	
	//---------------------------------------------------------------- Load mesh	
	//getFileLocation ( "armadillo_lores3.obj", fpath );
	//getFileLocation ( "cube_smooth.obj", fpath );
	//getFileLocation ( "surface.obj", fpath );	
	getFileLocation ( "sphere_uv.obj", fpath );

	//getFileLocation ( "model_dragon_head.obj", fpath );
	//getFileLocation ( "surface.obj", fpath );
	//getFileLocation ( "asteroid.obj", fpath );
	dbgprintf ( "Loading: %s\n", fpath.c_str());
	m_mesh = new MeshX;
	if ( !m_mesh->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-3);
	}

	//----------------------------------------------------------------- Load Displacement map 
	//getFileLocation ( "armadillo_displace1_1k.tif", fpath );
	//getFileLocation ( "armadillo_displace3_4k.tif", fpath );
	//getFileLocation ( "cube_displace_512.tif", fpath );
	getFileLocation ( "cube_test.tif", fpath );	
	//getFileLocation ( "surface_displace_256.tif", fpath );	
	//getFileLocation ( "model_dragon_disp4k.tif", fpath );
	//getFileLocation ( "stones_displace.png", fpath );
	//getFileLocation ( "rock01_disp.tif", fpath );
	//getFileLocation ( "mossy_displace.png", fpath );
	//getFileLocation ( "rock07_displace.png", fpath );
	//getFileLocation ( "gravel02_displace.png", fpath );
	//getFileLocation ( "bump_weave.tif", fpath );

	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_bump_img = new Image;
	if ( !m_bump_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}
	m_bump_img->Commit (  DT_CPU | DT_GLTEX );

	//---------------------------------------------------------
	getFileLocation ( "color_white.png", fpath );
	//getFileLocation ( "stones_color.png", fpath );
	//getFileLocation ( "rock01_color.png", fpath );	
	//getFileLocation ( "rocks02_color.png", fpath );
	//getFileLocation ( "rocks07_color.png", fpath );
	//getFileLocation ( "gravel02_color.png", fpath );
	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_color_img = new Image;
	if ( !m_color_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}

	// Construct Prisms
	ConstructPrisms ();
	
	// Output image
	Resize ();

	// Create a camera
	m_cam = new Camera3D;								
	m_cam->setFov ( 40 );
	m_cam->setAspect ( 1 );
	
	m_cam->setOrbit ( 90, 0, 0, Vector3DF(-0.5, 1.2, 1.2), 3, 1 );

	//m_cam->setOrbit ( -44, 30, 0, Vector3DF(0, 0, 0), 6, 1 );

	m_venable = 1;
	m_vfrom = Vector3DF( 0, 1, 5);
	m_vto = Vector3DF( .5, 1.5, 0 );			// 1.81
	RaytraceDebug (m_vfrom, m_vto, m_out_img->GetWidth()*0.5, m_out_img->GetHeight()*0.5, m_out_img);	


	m_lgt = Vector3DF(-100, 200, 150);
			
	UpdateCamera();

	return true;
}
void Sample::DrawGrid ()
{
	for (int i = -10; i <= 10; i++) {
		drawLine3D(float(i),-0.01f, -10.f, float(i), -0.01f, 10.f, .2f, .2f, .2f, 1.f);
		drawLine3D(-10.f,	-0.01f, float(i), 10.f, -0.01f, float(i), .2f, .2f, .2f, 1.f);
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
		
		// uv0.y = 1-uv0.y; uv1.y = 1-uv1.y; uv2.y = 1-uv2.y;   // plot y+ upward

		// draw triangle		
		drawLine ( uv0*sz, uv1*sz, clr ); 
		drawLine ( uv1*sz, uv2*sz, clr );	
		drawLine ( uv2*sz, uv0*sz, clr ); 
	}
}

void Sample::Resize ()
{
	// create output image
	if ( m_out_img==0x0) 
		m_out_img = new Image;

	// resize
	m_out_img->ResizeImage ( m_res, m_res, ImageOp::RGBA8 );	

	m_out_img->Fill (0);

	m_yline = m_res / 2;

	m_samples = 1;
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

void Sample::RaytraceMesh ( float bump_depth,  Image* img )
{
	float t, tx, diffuse, h;	
	AttrV3* f;
	Vector3DF n;
	Vector3DF v1,v2,v3;
	Vector3DF n1,n2,n3;
	Vector2DF uv1, uv2, uv3, uv;
	Vector3DF a,b,c,p;
	Vector3DF hit, norm;	
	Vector3DF rpos, rdir;
	Vector3DF bc;
	float alpha, beta;
	int best_f;
	float best_t;
	Vector3DF best_hit;
	int raycost;
	int i;
	int xres = img->GetWidth();
	int yres = img->GetHeight();
	int tcnt = 0;
	bool front;

	// maximal shell - expand sphere by bump depth	
	
	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {

			// get camera ray
			rpos = m_cam->getPos();
			rdir = m_cam->inverseRay ( x, y, xres, yres );	
			rdir.Normalize();

			// intersect with each triangle in mesh
			best_t = 1.0e10;
			best_f = -1;
			
			for (f = (AttrV3*) m_mesh->GetStart(BFACEV3); f <= (AttrV3*) m_mesh->GetEnd(BFACEV3); f++ ) {
				v1 = *m_mesh->GetVertPos( f->v1 );	v2 = *m_mesh->GetVertPos( f->v2 );	v3 = *m_mesh->GetVertPos( f->v3 );		
				n1 = *m_mesh->GetVertNorm( f->v1 );	n2 = *m_mesh->GetVertNorm( f->v2 );	n3 = *m_mesh->GetVertNorm( f->v3 );
			
				// intersect with extended triangle 				
				if ( intersectRayTriangle ( rpos, rdir, v1, v2, v3, t, alpha, beta, front ) ) {
					if ( t < best_t ) {
						best_t = t;
						best_f = f - (AttrV3*) m_mesh->GetStart(BFACEV3);
						best_hit = hit;
					}
				}
			}


			// did we hit a triangle?..
			if ( best_f >= 0 ) {

				// shade nearest triangle
				f = (AttrV3*) m_mesh->GetElem (BFACEV3, best_f );
				v1 = *m_mesh->GetVertPos( f->v1 );	v2 = *m_mesh->GetVertPos( f->v2 );	v3 = *m_mesh->GetVertPos( f->v3 );		
				n1 = *m_mesh->GetVertNorm( f->v1 );	n2 = *m_mesh->GetVertNorm( f->v2 );	n3 = *m_mesh->GetVertNorm( f->v3 );
				uv1 = *m_mesh->GetVertTex( f->v1 );	uv2 = *m_mesh->GetVertTex( f->v2 );	uv3 = *m_mesh->GetVertTex( f->v3 );
				//n2 = *m_mesh->GetVertNorm( f->v2 );	n3 = *m_mesh->GetVertNorm( f->v3 );

				// determine barycentric coordinates				
				intersectRayTriangle ( rpos, rdir, v1, v2, v3, t, bc.x, bc.y, front );

				// interpolate uv coords from barycentric
				uv = uv1 * (bc.x) + uv2 * (bc.y) + uv3 * (1-bc.x-bc.y);				
	
				// read texture
				//tx = m_bump_img->GetPixelUV16 ( uv.x, uv.y );
				tx = m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y );

				if (0) {
					//---- shading
					// interpolate normal
					n1 = *m_mesh->GetVertNorm( f->v1 );	n2 = *m_mesh->GetVertNorm( f->v2 );	n3 = *m_mesh->GetVertNorm( f->v3 );
					norm = n1 * float(bc.x) + n2 * float(bc.y) + n3 * float(1-bc.x-bc.y);
					norm.Normalize();

					// diffuse shading
					diffuse = std::max(0.0, norm.Dot( m_lgt ));
					c = m_color_img->GetPixelFilteredUV ( uv.x, uv.y );
					c = c * float(diffuse * 255.0);					
				} else {
					c = tx * 255.0;
				}
				if (c.x > 255 || c.y > 255 || c.z > 255) c = Vector3DF(255,255,255); 
					
				img->SetPixel ( x, y, c.x, c.y, c.z, 255.0 );	
			} 
		}
	}

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_CPU | DT_GLTEX );		
}


void ComputePrismBounds ( Vector3DF vb0, Vector3DF vb1, Vector3DF vb2, Vector3DF ve0, Vector3DF ve1, Vector3DF ve2,
							Vector3DF& bmin, Vector3DF& bmax )
{
	Vector3DF v[6];
	v[0] = vb0;
	v[1] = vb1;
	v[2] = vb2;
	v[3] = ve0;
	v[4] = ve1;
	v[5] = ve2;

	bmin = v[0];
	bmax = v[0];

	for (int j=1; j < 6; j++) {
		if ( v[j].x < bmin.x ) bmin.x = v[j].x;
		if ( v[j].y < bmin.y ) bmin.y = v[j].y;
		if ( v[j].z < bmin.z ) bmin.z = v[j].z;
		if ( v[j].x > bmax.x ) bmax.x = v[j].x;
		if ( v[j].y > bmax.y ) bmax.y = v[j].y;
		if ( v[j].z > bmax.z ) bmax.z = v[j].z;
	}
}

#define fsgn(x)		(x<0 ? -1 : 1)

// ray/bilinear patch intersect
// - the coordinates q should be specified in clockwise order
// 
bool Sample::intersect_ray_patch (Vector3DF rpos, Vector3DF rdir, Vector3DF q00, Vector3DF q10, Vector3DF q11, Vector3DF q01, float& tmin, float& tmax, int e, int& emin, int& emax ) 
{
	float t1, v1, t2, v2;
	Vector3DF pa, pb, n;
	Vector3DF e10 = q10 - q00;
	Vector3DF e11 = q11 - q10;
	Vector3DF e00 = q01 - q00;
	Vector3DF qn = e10.Cross ( (q01-q11) );   // normal of the diagonals
	q00 -= rpos;
	q10 -= rpos;
	float a = q00.Cross ( rdir ).Dot ( e00 );
	float c = qn.Dot ( rdir );
	float b = q10.Cross ( rdir ).Dot ( e11 );
	b -= a + c;
	float det = b*b - 4*a*c;
	if (det < 0 ) return false;
	det = sqrt(det);
	float u1, u2;
	float u=0, v=0;
	float t = 1e10;
	if ( c==0 ) {				// trapezid. only one root
		u1 = -a/b; 
		u2 = -1;
	} else {
		u1 = (-b - det*fsgn(b) )/2;
		u2 = a/u1;
		u1 /= c;
	}
	if (0 <= u1 && u1 <=1 ) {
		pa = q00 + (q10-q00)*u1;
		pb = e00 + (e11-e00)*u1;
		n = rdir.Cross ( pb );
		det = n.Dot ( n );
		n = n.Cross ( pa );
		t1 = n.Dot ( pb );
		v1 = n.Dot ( rdir );
		if ( t1 > 0 && 0 <= v1 && v1 <= det ) {
			t = t1/det; u = u1; v = v1/det;
		}
	}
	if ( 0 <= u2 && u2 <= 1 ) {
		pa = q00 + (q10-q00)*u2;
		pb = e00 + (e11-e00)*u2;
		n = rdir.Cross ( pb );
		det = n.Dot ( n );
		n = n.Cross ( pa );
		t2 = n.Dot ( pb ) / det;
		v2 = n.Dot ( rdir );
		if ( 0 <= v2 && v2 <= det && t > t2 && t2 > 0 ) {
			t = t2; u = u2; v = v2/det;
		}
	}
	if (t==1e10) return false;

	// surface normal
	pb = e00 + (e11-e00) * u;
	pa = e10 + ((q11-q01)-e10) * v;	
	n = pa.Cross ( pb );
	c = n.Dot ( rdir * -1 );

	if ( c >=0 && t > tmax ) {tmax = t; emax = e; return false;}	// backfacing
	if ( c < 0 && t < tmin ) {tmin = t; emin = e; return true;}     // facing the ray	
	
	// compute shading normal given normals at each vertex (vn)
	// pa = vn[0] + (vn[1]-vn[0])*u;
	// pb = vn[3] + (vn[2]-vn[3])*u;
	// n = pa + (pb-pa)* v;
	// ..OR..
	// n = vn[2]*u*v + vn[1]*(1-u)*v + vn[3]*u*(1-v) + vn[0]*(1-u)*(1-v);  // (slower)

	return false;
}

bool Sample::intersect_tri_inout ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& tmin, float& tmax, int e, int& emin, int& emax )
{
	Vector3DF e0 = v1 - v0;
    Vector3DF e1 = v0 - v2;
    Vector3DF n = e1.Cross ( e0 );
	float ndotr = n.Dot( dir );	
	Vector3DF e2 = (v0 - orig) / ndotr;
	float t = n.Dot ( e2 );
	if ( t<0.0 ) return false;

	Vector3DF i = dir.Cross ( e2 );
	float alpha =	i.x*e0.x + i.y*e0.y + i.z*e0.z;	
	float beta =	i.x*e1.x + i.y*e1.y + i.z*e1.z;		
	if ( alpha<0.0 || beta<0.0 || (alpha+beta>1)) return false;  
	if ( ndotr >=0 && t > tmax ) {tmax = t; emax = e; return false;}	  // backfacing
	if ( ndotr < 0 && t < tmin ) {tmin = t; emin = e; return true;}     // facing the ray	
	return false;
}


bool Sample::intersect_ray_prism ( Vector3DF rpos, Vector3DF rdir, Vector3DF vb0, Vector3DF vb1, Vector3DF vb2, Vector3DF ve0, Vector3DF ve1, Vector3DF ve2, 
										float& tmin, float& tmax, int& emin, int& emax)
{
	tmin = 1e10; tmax = 0;
	emin = -1; emax = -1;	
    intersect_tri_inout ( rpos, rdir, ve0, ve1, ve2, tmin, tmax, 0, emin, emax );
	intersect_tri_inout ( rpos, rdir, vb0, vb2, vb1, tmin, tmax, 1, emin, emax );
	
	intersect_ray_patch ( rpos, rdir, vb0, ve0, ve1, vb1, tmin, tmax, 3, emin, emax );
	intersect_ray_patch ( rpos, rdir, vb1, ve1, ve2, vb2, tmin, tmax, 4, emin, emax );
	intersect_ray_patch ( rpos, rdir, vb2, ve2, ve0, vb0, tmin, tmax, 5, emin, emax );

	/*intersect_tri_inout ( rpos, rdir, vb0, ve1, ve0, tmin, tmax, 3, emin, emax );
	intersect_tri_inout ( rpos, rdir, vb0, vb1, ve1, tmin, tmax, 3, emin, emax );
	intersect_tri_inout ( rpos, rdir, vb1, ve2, ve1, tmin, tmax, 4, emin, emax );
	intersect_tri_inout ( rpos, rdir, vb1, vb2, ve2, tmin, tmax, 4, emin, emax );
	intersect_tri_inout ( rpos, rdir, vb2, ve0, ve2, tmin, tmax, 5, emin, emax );
	intersect_tri_inout ( rpos, rdir, vb2, vb0, ve0, tmin, tmax, 5, emin, emax );*/

	if (tmax>0) {
		if (tmin==1e10) {
			tmin = 0;
			emin = 2;	// no front hit, ray is starting inside
		}
		return true;
	}
    return false;
}

void Sample::ConstructPrisms ()
{
	// Precompute prism geometry
	// - gives good acceleration. necessary for CPU version
	// - these are pushed into array that matches the enumeration of faces in the mesh	

	Vector3DF vb0, vb1, vb2;
	Vector3DF vn0, vn1, vn2;	

	Prism s;
	AttrV3* f;

	for (f = (AttrV3*) m_mesh->GetStart(BFACEV3); f <= (AttrV3*) m_mesh->GetEnd(BFACEV3); f++ ) {

		// get vertex normals
		vn0 = *m_mesh->GetVertNorm( f->v1 ); 
		vn1 = *m_mesh->GetVertNorm( f->v2 ); 
		vn2 = *m_mesh->GetVertNorm( f->v3 );				

		s.vb0 = *m_mesh->GetVertPos( f->v1 );	
		s.vb1 = *m_mesh->GetVertPos( f->v2 );
		s.vb2 = *m_mesh->GetVertPos( f->v3 );

		s.norm = (s.vb2 - s.vb1).Cross ( s.vb0 - s.vb2 ); 
		s.norm.Normalize();

		Vector3DF e;
		e.x = m_displace.y / s.norm.Dot ( vn0 );
		e.y = m_displace.y / s.norm.Dot ( vn1 );
		e.z = m_displace.y / s.norm.Dot ( vn2 );
		s.dmax = fmax(e.x, fmax(e.y, e.z));
		if (s.dmax > m_displace.y*2 ) s.dmax = m_displace.y*2;
		
		s.ve0 = s.vb0 + vn0 * s.dmax;
		s.ve1 = s.vb1 + vn1 * s.dmax;
		s.ve2 = s.vb2 + vn2 * s.dmax;

		e.x = m_displace.x / s.norm.Dot ( vn0 );
		e.y = m_displace.x / s.norm.Dot ( vn1 );
		e.z = m_displace.x / s.norm.Dot ( vn2 );
		s.dmin = fmin(e.x, fmin(e.y, e.z));
		//if (s.dmin < m_displace.x ) s.dmin = m_displace.x;
		if (s.dmin < m_displace.x ) s.dmin = m_displace.x;
		//s.dmin = m_displace.x*2;

		s.vb0 = s.vb0 + vn0 * s.dmin;
		s.vb1 = s.vb1 + vn1 * s.dmin;
		s.vb2 = s.vb2 + vn2 * s.dmin;	
		

		// Prism bounding box		
		ComputePrismBounds ( s.vb0, s.vb1, s.vb2, s.ve0, s.ve1, s.ve2, s.bmin, s.bmax );

		m_prisms.push_back ( s );
	}
}


bool Sample::FindPrismHitCandidates ( Vector3DF rpos, Vector3DF rdir, std::vector<hitinfo_t>& candidates )
{
	Prism* s;
	AttrV3* f;
	Vector3DF vb0, vb1, vb2;
	Vector3DF n;
	int face, emin, emax;
	float tmin, tmax;
	
	// clear candidates
	candidates.clear ();

	// search all prisms (faces) for potential hit
	f = (AttrV3*) m_mesh->GetStart(BFACEV3);

	int minf = 0;
	int maxf = m_mesh->GetNumFace3()-1;
	float t;

	for (int i=minf; i <= maxf; f++, i++ ) {

		s = &m_prisms[i];
		
		// fast bounding box check..
		if ( intersectLineBox ( rpos, rdir, s->bmin, s->bmax, t ) ) {
			
			// intersect with triangular prism
			float alpha, beta;
			if ( intersect_ray_prism ( rpos, rdir, s->vb0, s->vb1, s->vb2, s->ve0, s->ve1, s->ve2, tmin, tmax, emin, emax) ) {
				// add to potential hit list
				face = f-(AttrV3*) m_mesh->GetStart(BFACEV3);
				candidates.push_back ( hitinfo_t( face, tmin, tmax, emin, emax ) );
			} 
		}
	}
	return (candidates.size()!=0);
}


void clamp2d ( Vector3DF& v, float vmin, float vmax )
{
    v.x = (v.x < vmin) ? vmin : (v.x > vmax) ? vmax : v.x;
    v.y = (v.y < vmin) ? vmin : (v.y > vmax) ? vmax : v.y;
}

Vector3DF getBarycentricInTriangle ( Vector3DF& hit, Vector3DF& ve0, Vector3DF& ve1, Vector3DF& ve2, float& area )
{
	Vector3DF q0, q1, q2, bc;
	float d00, d01, d11, d20, d21;
	q0 = ve1-ve0; q1 = ve2-ve0; q2 = hit - ve0;
	d00 = q0.Dot( q0);
	d01 = q0.Dot( q1);
	d11 = q1.Dot( q1);
	d20 = q2.Dot( q0);
	d21 = q2.Dot( q1);
	area = (d00 * d11 - d01 * d01);
	bc.y = (d11 * d20 - d01 * d21) / area;
	bc.z = (d00 * d21 - d01 * d20) / area;
	bc.x = 1.0-bc.y-bc.z;
	return bc;
} 

float sign(float x)
{
	return (x>=0 ? 1 : -1);
}

#define EPS     0.1

float pointToLine2D ( Vector3DF p, Vector3DF v0, Vector3DF v1 )
{
	v1 -= v0;	
	return fabs( v1.x*(v0.y-p.y) - v1.y*(v0.x-p.x) ) / sqrt( v1.x*v1.x + v1.y*v1.y );
}

float intersectPlane ( float h, Vector3DF n, Vector3DF dir  )
{
	return h / n.Dot( dir );
}

Vector3DF Sample::projectPointInPrism ( Vector3DF p, Vector3DF v0, Vector3DF v1, Vector3DF v2,
										     Vector3DF vn0, Vector3DF vn1, Vector3DF vn2, Vector3DF& q )
{
	Vector3DF n, q0, q1, q2, bc;
	float de, area;

	// point-to-plane distance	
	n = (v2-v1).Cross ( v0-v2 );	// n.Normalize();
	de = n.Dot( p - v0 );

	// get scan triangle 	
	q0 = v0 + vn0 * intersectPlane( de, n, vn0 );		//  de / n.Dot( vn0 );
    q1 = v1 + vn1 * intersectPlane( de, n, vn1 );
    q2 = v2 + vn2 * intersectPlane( de, n, vn2 );

	bc = getBarycentricInTriangle  ( p, q0, q1, q2, area );

	/*VisLine ( q0,q1, Vector4DF(1,0,1,1));
	VisLine ( q1,q2, Vector4DF(1,0,1,1));
	VisLine ( q2,q0, Vector4DF(1,0,1,1));  */

	q = v0*bc.x + v1*bc.y + v2*bc.z;		// optional
	
	return bc;
}																

#define	T_DEL		0.05

void Sample::ComputeBumpAdjust ( int face, Vector3DF& bw, Vector3DF& bcp, Vector3DF rpos, Vector3DF rdir, float dt, float tmin, float tmax, int emin, int emax )
{
	AttrV3* f;
	Vector3DF v0, v1, v2;
	Vector3DF vn0, vn1, vn2;
	Vector3DF vd0, vd1, vd2;
	Vector3DF vuv0, vuv1, vuv2, uv;
	Vector3DF q0, q1;
	Vector3DF bc;
	float area;

	f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);		
	Prism* s = &m_prisms[ face ];	
	v0   = *m_mesh->GetVertPos( f->v1 );  v1  = *m_mesh->GetVertPos( f->v2 );  v2 = *m_mesh->GetVertPos( f->v3 );	
	vn0  = *m_mesh->GetVertNorm(f->v1 ); vn1  = *m_mesh->GetVertNorm( f->v2 ); vn2 = *m_mesh->GetVertNorm( f->v3 );
	vuv0 = *m_mesh->GetVertTex( f->v1 ); vuv1 = *m_mesh->GetVertTex( f->v2 ); vuv2 = *m_mesh->GetVertTex( f->v3 );	
	
	// compute extrapolated values at edges
	if (tmin != tmax ) {

		// bc.z = 0		edge = 3	 v0 -> v1
		// bc.x = 0		edge = 4	 v1 -> v2
		// bc.y = 0		edge = 5     v2 -> v0	

		float tdel = T_DEL;

		// compute 
		if (emin >= 3 ) {			
			bc = projectPointInPrism ( rpos + rdir * (tmin + tdel), v0, v1, v2, vn0, vn1, vn2, q0 );
			uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;
			m_bamt[0] = m_displace.x + m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y ) * (m_displace.y-m_displace.x);

			bc = projectPointInPrism ( rpos + rdir * (tmin + tdel*2), v0, v1, v2, vn0, vn1, vn2, q1 );
			uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;
			m_bamt[1] = m_displace.x + m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y ) * (m_displace.y-m_displace.x);

			//VisDot ( q0, 0.2, Vector4DF(0,1,1,1));
			//VisDot ( q1, 0.2, Vector4DF(0,1,1,1));
		}

		if (emax >= 3 ) {
			bc = projectPointInPrism ( rpos + rdir * (tmax - tdel), v0, v1, v2, vn0, vn1, vn2, q0 );
			uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;
			m_bamt[2] = m_displace.x + m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y ) * (m_displace.y-m_displace.x);

			bc = projectPointInPrism ( rpos + rdir * (tmax - tdel*2), v0, v1, v2, vn0, vn1, vn2, q1 );
			uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;
			m_bamt[3] = m_displace.x + m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y ) * (m_displace.y-m_displace.x);			

			//VisDot ( q0, 0.2, Vector4DF(0,1,1,1));
			//VisDot ( q1, 0.2, Vector4DF(0,1,1,1));
		}


	}
}


//------------------- OLDER METHOD USING BCs
/*  // compute bc pixel deltas	
	bcp.z = 1.0 / (pointToLine2D ( vuv2, vuv0, vuv1 ) * m_bump_img->GetWidth());
	bcp.x = 1.0 / (pointToLine2D ( vuv0, vuv1, vuv2 ) * m_bump_img->GetWidth());
	bcp.y = 1.0 / (pointToLine2D ( vuv1, vuv2, vuv0 ) * m_bump_img->GetWidth());

	// compute bc world delta lengths
	bc.Set (.5, .5, 0 );
	q0 = v0*bc.x + v1*bc.y + v2*bc.z;
	q1 = v0*bc.x + v1*bc.y + v2*(bc.z + bcp.z);
	bw.z = (q1 - q0).Length();

	bc.Set (0, .5, .5);
	q0 = v0*bc.x + v1*bc.y + v2*bc.z;
	q1 = v0*(bc.x+bcp.x) + v1*bc.y + v2*bc.z;
	bw.x = (q1 - q0).Length();

	bc.Set (.5, 0, .5);
	q0 = v0*bc.x + v1*bc.y + v2*bc.z;
	q1 = v0*bc.x + v1*(bc.y+bcp.y) + v2*bc.z;
	bw.y = (q1 - q0).Length();
*/


// float dh = (dw.x > 0) ? -0.5*dw.x*bw.x : (dw.y > 0) ? -0.5*dw.y*bw.y : (dw.z > 0 ) ? -0.5*dw.z*bw.z : 0;  dh /= cos(45*DEGtoRAD)


float Sample::getBumpAmt ( Vector3DF bc, Vector3DF uv, float t, float tmin, float tmax, int emin, int emax)
{
	int i;
	float dk0, dk1, d, x;
	float tdel = T_DEL;	

	/* if ( emin >= 3 && t <= tmin+tdel ) {
		x = (t - tmin) / tdel;
		dk0 = m_bamt[0];		// closest to edge
		dk1 = m_bamt[1];	 
		return dk0 - (dk1-dk0) * (1-x);		
	} 
	if ( emax >=3 && t >= tmax-tdel ) {
		x = (tmax - t) / tdel;
		dk0 = m_bamt[2];		// closest to edge
		dk1 = m_bamt[3];	 
		return dk0 - (dk1-dk0) * (1-x);
	}*/
		
	return m_displace.x + m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y ) * (m_displace.y-m_displace.x);
}


bool Sample::IntersectSurface ( int face, Vector3DF rpos, Vector3DF rdir, float tmin, float tmax, int emin, int emax, float& t, Vector3DF& bc, float dt )
{
	AttrV3* f;	
	Prism* s;
	Vector3DF vb0, vb1, vb2;		// prism geometry
	Vector3DF ve0, ve1, ve2;
	Vector3DF vd0, vd1, vd2;
	Vector3DF vn0, vn1, vn2;
	Vector3DF v0, v1, v2;
	Vector3DF vuv0, vuv1, vuv2;
	Vector3DF q0, q1, q2;

	Vector3DF n, bcl, hit, uv, p;				
	float alpha, beta, rayhgt, raynh, bmphgt, df, de;
	float area;

	float hx = m_bump_img->GetWidth();
	float hy = m_bump_img->GetHeight();


	f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);	
	s = &m_prisms[ face ];	
	v0   = *m_mesh->GetVertPos( f->v1 );  v1   = *m_mesh->GetVertPos( f->v2 );  v2 = *m_mesh->GetVertPos( f->v3 );
	vn0  = *m_mesh->GetVertNorm( f->v1 ); vn1  = *m_mesh->GetVertNorm( f->v2 ); vn2 = *m_mesh->GetVertNorm( f->v3 );
	vuv0 = *m_mesh->GetVertTex( f->v1 );  vuv1 = *m_mesh->GetVertTex( f->v2 );  vuv2 = *m_mesh->GetVertTex( f->v3 );

	vb0 = s->vb0;
	vb1 = s->vb1;
	vb2 = s->vb2;
	ve0 = s->ve0; 
	ve1 = s->ve1; 
	ve2 = s->ve2;

	// Initial prism hit estimate
	t = tmin;

	if ( m_jitter_sample ) {
		t += m_rand.randF(0,1) * dt;
	}

	hit = rpos + rdir * t;

	//--- debug prism hits
	// bc = getBarycentricInTriangle (hit, ve0, ve1, ve2 );
	// return true;

	// Fractional change in ray height distance to triangle along normal
	// - dr/dy = norm . ray dt   (sample spacing along vertical axis)
	// - df = (dr/dy) / depth    (spacing as a percentage of total depth)
	df = s->norm.Dot ( rdir * -dt );	
	vd0 = vn0 * -df;
	vd1 = vn1 * -df;
	vd2 = vn2 * -df;

	// Project hit point to base triangle
	// - p = proj(hit)   (perpendicular projection of initial hit to base triangle)	
	switch (emin) {	
	case 0:			// top triangle
		raynh = 1.0;
		break;
	case 1:			// bottom triangle
		raynh = 0.0;
		break;
	case 2:			// starting inside prism		        
		q0 = ( ve2-ve1 ).Cross( ve0-ve2 );		// don't normalize q0 here		
		de = q0.Dot( ve0 - hit ) / df;        			
		// set scanning triangle 
		q0 = ve0 + vd0 * de;
        q1 = ve1 + vd1 * de;
        q2 = ve2 + vd2 * de;
        // project
        bc = getBarycentricInTriangle (hit, q0, q1, q2, area );       // using q0,q1,q2
        q0 = vb0 * bc.x + vb1 * bc.y + vb2 * bc.z;	
        q1 = ve0 * bc.x + ve1 * bc.y + ve2 * bc.z;	
        raynh = (hit - q0).Length() / (q1-q0).Length();
		break;
	case 3:			// face of edge e0,e1.. bc.z=0
		q0 = projectPointLine ( hit, vb0, vb1 ); 
		q1 = projectPointLine ( hit, ve0, ve1 );
		raynh = (hit - q0).Length() / (q1-q0).Length();
		break;
	case 4:			// face of edge e1,e2.. bc.x=0
		q0 = projectPointLine ( hit, vb1, vb2 ); 
		q1 = projectPointLine ( hit, ve1, ve2 );
		raynh = (hit - q0).Length() / (q1-q0).Length();
		break;
	case 5:			// face of edge e2,e0.. bc.y=0
		q0 = projectPointLine ( hit, vb2, vb0 ); 
		q1 = projectPointLine ( hit, ve2, ve0 ); 
		raynh = (hit - q0).Length() / (q1-q0).Length();
		break;
	}; 

	// Construct initial scanning triangle	
	ve0 = vb0 + (ve0-vb0) * raynh;
	ve1 = vb1 + (ve1-vb1) * raynh;
	ve2 = vb2 + (ve2-vb2) * raynh;	

	// Get barycentric coordinates (using Cramer's rule)
	bc = getBarycentricInTriangle  ( hit, ve0, ve1, ve2, area );	

	// Ensure valid bcs	
	/*bc.x = (bc.x < 0) ? 0 : (bc.x > 1) ? 1 : bc.x;
	bc.y = (bc.y < 0) ? 0 : (bc.y > 1) ? 1 : bc.y;	
	bc.z = 1-bc.x-bc.y;
	bc.z = (bc.z < 0) ? 0 : (bc.z > 1) ? 1 : bc.z;             	
	bcl = bc; */

	// Get ray height
	q0 = v0*bc.x + v1*bc.y + v2*bc.z;
	q1 = s->ve0*bc.x + s->ve1*bc.y + s->ve2*bc.z;
	//rayhgt = (q1-q0).Length() - (q1-hit).Length();
	rayhgt = (hit-q0).Length() * sign ( s->norm.Dot(hit-q0) );

	

	// [optional] visualization	
	if ( m_vcurr==1 ) {		
		VisLine ( q0, q1, m_vhit );		
		VisDot ( hit, 1, m_vhit );			// initial prism hit				
		VisDot ( p, 1, m_vbase );			// visualize base points (green)
		VisLine ( ve0, ve1, m_vscan );		// visualize scanning triangle
		VisLine ( ve1, ve2, m_vscan );
		VisLine ( ve2, ve0, m_vscan );
	}

	Vector3DF bw, bcp;
	ComputeBumpAdjust ( face, bw, bcp, rpos, rdir, dt, tmin, tmax, emin, emax );	
	
	// Sample displacement texture for initial surface height			
	uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;	
	bmphgt = getBumpAmt ( bc, uv, t, tmin, tmax, emin, emax);
	
	// March ray until it hits or leaves prism
	//
	for (t=tmin; rayhgt > bmphgt && t < tmax; t += dt) {			

		// point-to-plane distance
		de = s->norm.Dot( hit - v0 );
		
		// get scanning triangle 	
		q0 = v0 + vn0 * de / float(s->norm.Dot(vn0));
		q1 = v1 + vn1 * de / float(s->norm.Dot(vn1));
		q2 = v2 + vn2 * de / float(s->norm.Dot(vn2));

		// barycentric coords on triangle
		bcl = bc;
		bc = getBarycentricInTriangle  ( hit, q0, q1, q2, area );	

		// ray height
		// - more accurate measured from the pos extent triangle
		q0 = v0*bc.x + v1*bc.y + v2*bc.z;		
		rayhgt = (hit-q0).Length() * sign ( de );		// can use sign( de ) here

		uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;

		// sample bump map to get displacement height		
		bmphgt = getBumpAmt ( bc, uv, t, tmin, tmax, emin, emax);

		// march along ray
		hit += rdir * dt;	

		// [optional] visualization
		if ( m_vcurr==1 ) {			
			VisDot ( hit, .5, m_vsample );		// sample points 
			
			// VisDot ( q0, .5, m_vbase );			// base points
			
			n = vn0 * bc.x + vn1 * bc.y + vn2 *bc.z; n.Normalize();			
			q1 = q0 + n * bmphgt;
			VisDot ( q1, .1, Vector4DF(1,0,0,1) );			// surface points 

			q1 = q0 + n * getBumpAmt ( bc, uv );
			VisDot ( q1, .1, Vector4DF(0,1,0,1) );			// surface points 
		}

		
	}

	bool tedge = false;

	// Check for miss or t out-of-bounds
	if ( t >= tmax ) {

		tedge = true;

		// one more check at tmax
		/*t = tmax;

		// march along ray
		hit = rpos + rdir * t;

		// point-to-plane distance
		q0 = (ve2-ve1).Cross ( ve0-ve2 ); 
		q0.Normalize();
		de = q0.Dot( ve0 - hit ) / df;
		// advance scanning triangle 
		ve0 += vd0 * de;
		ve1 += vd1 * de;
		ve2 += vd2 * de;
		// barycentric coords on triangle		
		bcl = bc;
		bc = getBarycentricInTriangle  ( hit, ve0, ve1, ve2 );
		// Ensure valid bcs	at edge
		bc.x = (bc.x < 0) ? 0 : (bc.x > 1) ? 1 : bc.x;
		bc.y = (bc.y < 0) ? 0 : (bc.y > 1) ? 1 : bc.y;	
		bc.z = 1-bc.x-bc.y;
		bc.z = (bc.z < 0) ? 0 : (bc.z > 1) ? 1 : bc.z;		
		// ray height
		p = s->ve0*bc.x + s->ve1*bc.y + s->ve2*bc.z;			
		rayhgt = (m_displace.y+m_displace.z) - (hit-p).Length();
		// sample bump map to get displacement height
		uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;
		bmphgt = m_displace.z + m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y ) * (m_displace.x-m_displace.z);

		// [optional] visualization
		if ( m_vcurr==1 ) {	
			Vector3DF q1 = s->ve0 *bc.x + s->ve1 *bc.y + s->ve2 *bc.z;
			Vector3DF a = p + (q1-p).Normalize() * bmphgt;
			VisDot ( hit, 7.0, m_vsample );		// sample points 
			VisDot ( p, 7.0, m_vbase );			// base points 						
			VisDot ( a, 7.0, m_vsurf );			// surface points 			
		}

		if ( rayhgt > bmphgt) 
			return false;

		// Hit surface at edge
		// Final hit t
		t = tmax;*/

		return false;
	
	} else {

		// Hit surface
		// Linear interpolation to find surface
		// we have current sample already - below surface					
		// get previous sample - above surface		
		uv = vuv0 * bcl.x + vuv1 * bcl.y + vuv2 * bcl.z;
		p =  v0   * bcl.x + v1   * bcl.y + v2  * bcl.z;
		hit -= rdir*dt;
		float rayhgt0 = (hit-p).Length() * sign( s->norm.Dot(hit-p) );		
		float bmphgt0 = getBumpAmt ( bc, uv, t, tmin, tmax, emin, emax );
		// interpolate between previous and current sample
		float u = (rayhgt0-bmphgt0) / ((rayhgt0-bmphgt0)+(bmphgt-rayhgt));
		u = (u<0) ? 0 : (u>1) ? 1 : u;									 
					
		// Final hit t
		t = (hit - rpos).Length() - dt + u*dt; 

		hit = rpos + rdir * t;

		// Final update of bcs			
		de = s->norm.Dot( hit - v0 );			
		q0 = v0 + vn0 * de / float(s->norm.Dot(vn0));
		q1 = v1 + vn1 * de / float(s->norm.Dot(vn1));
		q2 = v2 + vn2 * de / float(s->norm.Dot(vn2));
		bcl = bc;
		bc = getBarycentricInTriangle  ( hit, q0, q1, q2, area );	
	}	

	return true;	
}

bool Sample::ShadeSurface ( int face, Vector3DF rpos, Vector3DF rdir, float tmin, float tmax, int emin, int emax, float t, Vector3DF bc, Vector3DF& n, Vector3DF& clr )
{
	AttrV3* f;	
	Prism* s;

	//--- debug hit
	// Vector3DF hit = rpos + rdir * t;
	// clr = Vector3DF(hit.x, hit.y, hit.z) * 255.0f;
	//return true;

	Vector3DF vb0, vb1, vb2;			// prism geometry
	Vector3DF vn0, vn1, vn2;
	Vector3DF v0, v1, v2;
	Vector3DF vuv0, vuv1, vuv2, uv;	
	Vector3DF q0, q1, q2;
	float dpu, dpv, nadj;

	Vector3DF L, R, V, c, n0;			// shading rays
	float diffuse, spec;

	// Get the prism and face
	f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);	
	s = &m_prisms[ face ];		
	v0   = *m_mesh->GetVertPos( f->v1 );  v1   = *m_mesh->GetVertPos( f->v2 );  v2 = *m_mesh->GetVertPos( f->v3 );
	vn0 =  *m_mesh->GetVertNorm( f->v1 ); vn1 = *m_mesh->GetVertNorm( f->v2 ); vn2 = *m_mesh->GetVertNorm( f->v3 );	
	vuv0 = *m_mesh->GetVertTex( f->v1 ); vuv1 = *m_mesh->GetVertTex( f->v2 ); vuv2 = *m_mesh->GetVertTex( f->v3 );	
	vb0 = s->vb0;
	vb1 = s->vb1;
	vb2 = s->vb2;

	// Construct normal from displacement surface
	dpu = 1.0 / (m_bump_img->GetWidth()-1);
	dpv = 1.0 / (m_bump_img->GetHeight()-1);		
	
	float dw, dh, bmphgt;
	
	// Sample displacement texture for initial surface height			
	uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;	
	
    float bDX, bDY;
	bDX = bc.x + dpu;
	bDY = bc.y + dpv;

	// interpolated base normal
	n0 = vn0 * bc.x + vn1 * bc.y + vn2 * bc.z; n0.Normalize();

	// sample along barycentric u
	uv = vuv0* bDX + vuv1* bc.y + vuv2* (1-bDX-bc.y);
    q1 = vb0 * bDX + vb1 * bc.y + vb2 * (1-bDX-bc.y);
	q1 += n0 * getBumpAmt ( Vector3DF(bDX, bc.y, 1-bDX-bc.y), uv, t, tmin, tmax, emin, emax );

    // sample along barycentric v		
    uv = vuv0* bc.x + vuv1* bDY + vuv2* (1-bc.x-bDY);
    q2 = vb0 * bc.x + vb1 * bDY + vb2 * (1-bc.x-bDY);
    q2 += n0 * getBumpAmt ( Vector3DF(bc.x, bDY, 1-bc.x-bDY), uv, t, tmin, tmax, emin, emax );

	// central sample	
    uv = vuv0* bc.x + vuv1* bc.y + vuv2* bc.z;
    q0 = vb0 * bc.x + vb1 * bc.y + vb2 * bc.z;
    q0 += n0 * getBumpAmt ( bc, uv, t, tmin, tmax, emin, emax );	

			
	// color texture
	clr = m_color_img->GetPixelFilteredUV ( uv.x, uv.y );

	/* if (m_vcurr==2) {
		VisDot ( q0, 0.1, m_vhit );
		VisDot ( q1, 0.1, m_vsurf );
		VisLine( q1, q0, m_vsurf);
		VisDot ( q2, 0.1, m_vsurf );
		VisLine( q2, q0, m_vsurf );
	}*/

	// Compute normal as cross product of surface displacements
	q1 -= q0;
	q2 -= q0;
	n = q1.Cross (q2); n.Normalize();

	// combine displace normal with adjustment for smooth normal
	// n = n + n0 - s->norm; n.Normalize();
	
	V = q0 - m_cam->getPos(); V.Normalize();

	//-- debug normal
	//n = s->norm;
	//q0 = rpos + rdir * t;
	// if ( n.Dot ( V ) > 0 ) clr = Vector3DF(1,0,0);
				
	// Diffuse shading	
	L = m_lgt - q0; L.Normalize();					
	diffuse = 0.7 * std::max(0.0, n.Dot( L ));
	R = n * float(2 * n.Dot(L)) - L; R.Normalize();	
	spec = 0.3 * pow ( R.Dot( V ), 20 );

	// colored final surface			
	clr = clr * diffuse + Vector3DF(spec,spec,spec);

	//--- color /w world xyz
	//clr = Vector3DF(q0.x, q0.y, q0.z)*0.3f + 0.5f;

	//--- color /w surface normal
	//clr = n*0.5f + 0.5f;

	//--- color /w barycentric
	// clr = Vector3DF(bc.x, bc.y, 1-bc.x-bc.y);

	//--- color /w interpolated base normal
	//clr = Vector3DF(1,1,1) * 0.5f * float( pow( n0.Dot(L), 3) );

	clr *= 255.0;
	if (clr.x > 255 || clr.y > 255 || clr.z > 255) clr = Vector3DF(255,255,255);

	return true;
}

void Sample::RaytraceStart ()
{
	m_rand.seed ( m_samples );
}



void Sample::RaytraceDisplacementMesh ( Image* img )
{
	Prism* s;
	AttrV3* f;
	Vector3DF vb0, vb1, vb2;		// prism geometry
	Vector3DF vn0, vn1, vn2;	
	Vector3DF rpos, rdir, n, bc;
	Vector3DF clr(1,1,1);
	
	float dt = 0.005;
	
	float doff = 0.0;
	bool used;

	// triangle search setup
	int face, edge, best_f;
	float t, best_t;
	Vector3DF best_bc;
	std::vector<hitinfo_t> candidates;

	// To make this interactive on the CPU we raytrace one scanline at a time.
	// Adjust the resolution using the ',' and '.' keys.
	// Advance y-line (from center outward)
	if ( m_yline <= img->GetHeight()/2 ) m_yline--;
	if ( m_yline >= img->GetHeight()/2 +1 ) m_yline++;
	if ( m_yline <= 0 ) m_yline = img->GetHeight()/2 + 1;

	if ( m_yline==img->GetHeight()-1) { 
		m_yline = img->GetHeight()/2; 
		m_samples++; 
		m_rand.seed ( m_samples );
	}

	//dbgprintf ( "%d ", m_yline );

	// raytrace scan line

	for (int x=0; x < img->GetWidth(); x++) {

		// Get camera ray
		rpos = m_cam->getPos();
		rdir = m_cam->inverseRay ( x, m_yline, img->GetWidth(), img->GetHeight() );	
		rdir.Normalize();

		// Collect candidates for hit
		if (! FindPrismHitCandidates ( rpos, rdir, candidates ) )
			continue;

		// Check candidates for displacement hit			
		best_t = 1.0e10;
		best_f = -1;	
		int best_i;
		t = 0;		
		for (int i=0; i < candidates.size(); i++ ) {
			// Look for intersection using this prism
			face = candidates[i].face;
				
			if ( IntersectSurface ( face, rpos, rdir, candidates[i].tmin, candidates[i].tmax, candidates[i].emin, candidates[i].emax, t, bc, dt ) ) {
				if ( t < best_t ) {
					best_t = t;						
					best_f = face;
					best_i = i;
					best_bc = bc;
				}				
			}
		} 				

		if ( best_f >= 0 ) {								
			// Hit found. Shade surface.
				
			ShadeSurface ( best_f, rpos, rdir, candidates[best_i].tmin, candidates[best_i].tmax, candidates[best_i].emin, candidates[best_i].emax, best_t, best_bc, n, clr );

			if ( m_jitter_sample )
				clr = (img->GetPixel( x, m_yline) * float(m_samples-1) + clr)/m_samples;					

			//-- debug prism sides
			//if (best_s !=-1 ) c += Vector3DF( (best_s==0), (best_s==1), (best_s==2) ) *20.0f;

			// Write pixel
			img->SetPixel ( x, m_yline, clr.x, clr.y, clr.z, 255.0 );

		}

	}	// x pixels	

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_CPU | DT_GLTEX );		
}

	


void Sample::VisTriangleSurface ( int face )
{
	Prism* s;
	AttrV3* f;
	float bmphgt;	
	Vector3DF v0, v1, v2, v;
	Vector3DF n0, n1, n2, n;	
	Vector3DF q0, q1, p;
	Vector2DF uv0,uv1,uv2, uv;
	
	if ( face==5 ) 
		bool stop=true;

	f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);
	s = &m_prisms[ face ];		
	n0  = *m_mesh->GetVertNorm( f->v1 ); n1  = *m_mesh->GetVertNorm( f->v2 ); n2  = *m_mesh->GetVertNorm( f->v3 );		
	v0  = *m_mesh->GetVertPos( f->v1 );  v1  = *m_mesh->GetVertPos( f->v2 );  v2  = *m_mesh->GetVertPos( f->v3 );			
	uv0 = *m_mesh->GetVertTex( f->v1 );  uv1 = *m_mesh->GetVertTex( f->v2 );  uv2 = *m_mesh->GetVertTex( f->v3 );	

	for (float a=0; a < 1.0; a += 0.01) {
		for (float b=0; b < 1.0; b+= 0.01) {

			if (a+b > 1) continue;

			// getsurface point and normal
			v = v0 * a + v1 * b + v2 * (1-a-b);
			n = n0 * a + n1 * b + n2 * (1-a-b);  n.Normalize();
			uv= uv0* a + uv1* b + uv2* (1-a-b);

			bmphgt = m_displace.x + m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y ) * (m_displace.y-m_displace.x);

			p = v + n * bmphgt;

			VisDot ( p, 0.2, m_vsurf );
		}
	}

}



void Sample::RaytraceDebug ( Vector3DF from, Vector3DF to, int x, int y, Image* img )
{
	Prism* s;
	AttrV3* f;
	Vector3DF v0, v1, v2;
	Vector3DF vb0, vb1, vb2;		// prism geometry	
	Vector3DF ve0, ve1, ve2;
	Vector3DF vn0, vn1, vn2;	
	Vector3DF rpos, rdir, n, clr, bc, hit;	
	Vector3DF q0, q1, q2;
	Vector2DF vuv0, vuv1, vuv2, uv;

	// triangle search setup
	int face, edge, best_f, i;
	float t, best_t;
	Vector3DF best_bc;
	std::vector<hitinfo_t> candidates;
	
	// ray setup
	int xres = img->GetWidth();
	int yres = img->GetHeight();	

	m_vfrom = from;
	m_vto = to;
	m_vpix.Set(x, y, 0);

	Camera3D vcam;
	vcam.setPos ( from.x, from.y, from.z );
	vcam.setToPos ( to.x, to.y, to.z );
	vcam.setFov ( 40 );
	vcam.updateMatricies(true);
	rpos = vcam.getPos();
	rdir = vcam.inverseRay ( m_vpix.x, m_vpix.y, xres, yres );	
	rdir.Normalize();

	// surface setup	

	float dt = 0.001;

	float doff = 0.0;
	bool used;

	// Collect candidates for hit
	if (! FindPrismHitCandidates ( rpos, rdir, candidates ) ) {
		m_venable = 0;
		m_vcurr = 0;
		return;
	}

	// print candidates
	printf ("-- FACES\n" );
	for (int n=0; n < candidates.size(); n++)
		printf ( "%d\n", candidates[n].face );

	// Start visualizations
	m_vcurr = m_venable;
	m_debugvis.clear ();

	// Check candidates for displacement hit	
	best_t = 1.0e10;
	best_f = -1;
	int best_i;
	t = 0;	
	
	for (int i=0; i < candidates.size(); i++ ) {

		// Look for intersection using this triangle				
		face = candidates[i].face;
		f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);
		s = &m_prisms[ face ];		
		v0 = *m_mesh->GetVertPos( f->v1 );	v1 = *m_mesh->GetVertPos( f->v2 );	v2 = *m_mesh->GetVertPos( f->v3 );
		ve0 = s->ve0; ve1 = s->ve1; ve2 = s->ve2;
		vb0 = s->vb0; vb1 = s->vb1; vb2 = s->vb2;

		// Visualize candidate prisms
		VisLine ( vb0, vb1, Vector3DF(0,1,0) );	VisLine ( vb1, vb2, Vector3DF(0,1,0) );	VisLine ( vb2, vb0, Vector3DF(0,1,0) );
		VisLine ( v0, v1, Vector3DF(1,1,1) );	VisLine ( v1, v2, Vector3DF(1,1,1) );	VisLine ( v2, v0, Vector3DF(1,1,1) );
		VisLine ( ve0, ve1, Vector3DF(1,.5,0) );VisLine ( ve1, ve2, Vector3DF(1,.5,0) ); VisLine ( ve2, ve0, Vector3DF(1,.5,0) ); 
		
		VisLine ( rpos, rpos+rdir*10, Vector3DF(1,1,1) );		// ray itself
		
		m_vhit.Set( 1, 1, 0);			// yellow
		m_vscan.Set( 1, 1, 0);			// yellow
		m_vsample.Set( 1, .5, 0);		// orange
		m_vbase.Set(0, 1, 0);			// green
		m_vsurf.Set(1, 0, 1);			// purple

		
		VisTriangleSurface ( face );	
		
		if ( IntersectSurface ( face, rpos, rdir, candidates[i].tmin, candidates[i].tmax, candidates[i].emin, candidates[i].emax, t, bc, dt ) ) {

			if ( t < best_t ) {
				best_t = t;				
				best_bc = bc;
				best_i = i;
				best_f = face;				
			}
			// visualize all hits
			if (m_vcurr==1) {
				hit = rpos + rdir * t;			
				VisDot ( hit, .1, m_vhit );
				VisDot ( hit, .6, m_vhit );
				VisDot ( hit, .7, m_vhit );			
			}
		}
	} 	
				
	if ( best_f >= 0 ) {

		hit = rpos + rdir * best_t;

		// highlight hit
		if (m_vcurr==1) {
			s = &m_prisms[ best_f ];	
			f = (AttrV3*) m_mesh->GetElem(BFACEV3, best_f );
			bc = best_bc;

			// surface pnt
			vuv0 = *m_mesh->GetVertTex( f->v1 ); vuv1 = *m_mesh->GetVertTex( f->v2 ); vuv2 = *m_mesh->GetVertTex( f->v3 );
			q0 = s->vb0*bc.x + s->vb1*bc.y + s->vb2*(1-bc.x-bc.y);
			q1 = s->ve0*bc.x + s->ve1*bc.y + s->ve2*(1-bc.x-bc.y);
			uv = vuv0*bc.x + vuv1*bc.y + vuv2*(1-bc.x-bc.y);
			q2 = q0 + (q1-q0) * m_bump_img->GetPixelFilteredUV16 ( uv.x, uv.y );
			VisDot ( q2, 2.0, m_vsurf );
			VisDot ( q2, 2.5, m_vsurf );	

			// hit pnt
			VisDot ( hit, 3.0, Vector3DF(1,0,0) );
			VisDot ( hit, 3.5, Vector3DF(1,0,0) );			
			// hit scanning triangle (white)
			VisLine ( s->ve0, s->ve1, Vector3DF(1,0,0) );		
			VisLine ( s->ve1, s->ve2, Vector3DF(1,0,0) ); 
			VisLine ( s->ve2, s->ve0, Vector3DF(1,0,0) );
		}

		Vector3DF n, clr;
		ShadeSurface ( best_f, rpos, rdir, candidates[best_i].tmin, candidates[best_i].tmax, candidates[best_i].emin, candidates[best_i].emax, best_t, best_bc, n, clr );
	
	} 

	// End visualization
	m_vcurr = 0;
}

#include <time.h>

void Sample::display()
{
	AttrV3* f;
	Vector3DF vb0,vb1,vb2;
	Vector3DF ve0,ve1,ve2;
	Prism* s;
	Vector4DF clr(1,1,1,.5);	
	Vector3DF N, V;

	clearGL();

	// Displacement raycasting
	clock_t t1,t2;
	
	if (!m_show_map) 
		RaytraceDisplacementMesh ( m_out_img );
	else
		RaytraceMesh ( m_displace.x, m_out_img );	
	

	//t1 = clock();
	//t2 = clock();	
	//dbgprintf ( "Time: %f msec\n", float(t2-t1)/float(CLOCKS_PER_SEC/1000.0f) );

	// Draw grid
	start3D(m_cam);
	setLight(S3D, m_lgt.x, m_lgt.y, m_lgt.z);	
		DrawGrid();		
	end3D();
	draw3D();

	// Draw raycast image
	start2D();
	setview2D(getWidth(), getHeight());		
		drawImg ( m_out_img->getGLID(), 0, 0, getWidth(), getHeight(), 1,1,1,1 );		// draw raycast image 	

		drawImg ( m_bump_img->getGLID(), 0, 0, 512, 512, 1,1,1,1 );		// draw raycast image 	

		DrawMeshUV ( m_mesh, 512, 512, Vector4DF(1,1,1,1) );
	end2D();	
	draw2D();										// complete 2D rendering to OpenGL


	V = m_cam->getDir ();
	V.Normalize();
	

	// Draw debug visualization
	if (m_venable ) {
		float vscale = 0.001f;

		start3D( m_cam );
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
		end3D();
	}

	start3D( m_cam );
	setview2D(getWidth(), getHeight());

	// Draw prisms in 3D		
	if ( m_draw_prisms ) {

		Vector3DF v0, v1, v2;
		Vector3DF c;
		char msg[16];

		for (int i = 0; i < m_prisms.size(); i++)  {
			s = &m_prisms[i];
			f = (AttrV3*) m_mesh->GetElem (BFACEV3, i );
			v0 = *m_mesh->GetVertPos( f->v1 );	v1 = *m_mesh->GetVertPos( f->v2 );	v2 = *m_mesh->GetVertPos( f->v3 );
			vb0 = s->vb0;  vb1 = s->vb1;  vb2 = s->vb2;
			ve0 = s->ve0;  ve1 = s->ve1;  ve2 = s->ve2;

			if ( s->norm.Dot ( V ) < 0 ) {

				c = (v1+v2+v2) / 3.0f;
				sprintf ( msg, "%d", i );
				//drawText3D ( m_cam, c.x, c.y, c.z, msg, 1,1,1,1);

				clr = Vector4DF(1,1,1,1);	drawLine3D ( v0, v1, clr ); drawLine3D ( v1, v2, clr );	drawLine3D ( v2, v0, clr);        // original
				clr = Vector4DF(0, 1,0, .5);  drawLine3D ( vb0, vb1, clr ); drawLine3D ( vb1, vb2, clr );	drawLine3D ( vb2, vb0, clr);  // base
				clr = Vector4DF(1,.5,1, .5);  drawLine3D ( ve0, ve1, clr ); drawLine3D ( ve1, ve2, clr ); drawLine3D ( ve2, ve0, clr ); // ext
				clr = Vector4DF(0, 1,0, .25);  drawLine3D ( vb0, v0, clr ); drawLine3D ( vb1, v1, clr );	drawLine3D ( vb2, v2, clr ); 
				clr = Vector4DF(1,.5,0, .25);  drawLine3D ( ve0, v0, clr ); drawLine3D ( ve1, v1, clr );	drawLine3D ( ve2, v2, clr ); 				
			}
		} 		
	}	
	end3D();		
	draw3D();

	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::UpdateCamera()
{
	Vector3DF a, t;
	a = m_cam->getAng();
	t = m_cam->getToPos();
	dbgprintf ( "angs: %3.4f %3.4f %3.4f, to: %3.4f %3.4f %3.4f\n", a.x,a.y,a.z, t.x,t.y,t.z );

	// clear the output image
	memset ( m_out_img->GetData(), 0, m_out_img->GetSize() );	
	
	m_samples = 1;			// reset samples	

	appPostRedisplay();		// update display
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
	float fine = 0.5;
	
	int i = m_currcam;

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
		
		Vector3DF to = m_cam->getToPos();

		dbgprintf ( "cam: angs %f,%f,%f  to %f,%f,%f  dist %f\n", angs.x, angs.y, angs.z, to.x, to.y, to.z, m_cam->getOrbitDist() );
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	if (guiHandler(button, state, x, y)) return;
	
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag

	if ( mouse_down==AppEnum::BUTTON_LEFT) {

		if (m_venable==0) m_venable = 1;
		int xp, yp;
		xp = x * m_out_img->GetWidth() / getWidth();
		yp = y * m_out_img->GetHeight() / getHeight();		
		RaytraceDebug ( m_cam->getPos(), m_cam->getToPos(), xp, yp, m_out_img );

		/* if ( m_debugvis.size() > 0 ) {			
			Vector3DF to = m_debugvis[ m_debugvis.size()-1 ].a;
			float orbit = 2.0;
			m_cam->setOrbit ( m_cam->getAng(), to, orbit, 1 );
			UpdateCamera ();
		} */
	}
}

void Sample::mousewheel(int delta)
{
	// Adjust zoom
	int i = m_currcam;

	float s = 0.0001 * delta;

	MoveCamera('p', Vector3DF( 0, 0,+s));

	UpdateCamera();
}


void Sample::MoveCamera ( char t, Vector3DF del )
{
	switch (t) {
	case 'p': {
		float orbit = m_cam->getOrbitDist() - del.z; 
		if ( orbit < 0.1 ) orbit = 0.1;
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

	float s = (mods==1) ? 1.0 : 0.1;

	switch (keycode) {
	case 'm': 
		m_show_map = !m_show_map; 
		UpdateCamera();		
		break;
	case 'j':
		m_jitter_sample = !m_jitter_sample ;
		UpdateCamera();		
		break;
	case 'p':
		m_draw_prisms = !m_draw_prisms;
		UpdateCamera();		
		break;
	case ' ': m_run = !m_run; break;
		break;
	case '.': 
		m_res *= 2; 
		Resize();
		break;
	case ',': 
		m_res /= 2; 
		if ( m_res < 32 ) m_res = 32;
		Resize();
		break;


	case '`': case 0: m_venable = 0; break;
	case 'r': {
		Vector3DF angs;
		angs = m_cam->getAng();
		angs.x += 45;
		m_cam->setOrbit ( angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly() );
		UpdateCamera();
		} break;


	case 'w': case 'W':		MoveCamera('p', Vector3DF( 0, 0,+s)); break;		// WASD navigation keys
	case 's': case 'S':		MoveCamera('p', Vector3DF( 0, 0,-s)); break;

	case 'a': case 'A':		MoveCamera('t', Vector3DF(-s, 0, 0));	break;	
	case 'd': case 'D':		MoveCamera('t', Vector3DF(+s, 0, 0));	break;
	case 'q': case 'Q':		MoveCamera('t', Vector3DF(0, -s, 0));	break;
	case 'z': case 'Z':		MoveCamera('t', Vector3DF(0, +s, 0));	break;


	// visualization mode
	case '1': 		// geometry vis
		m_venable = 1; 
		RaytraceDebug (m_vfrom, m_vto, m_vpix.x, m_vpix.y, m_out_img);	
		break;
	case '2':		// shading vis
		m_venable = 2; 
		RaytraceDebug (m_vfrom, m_vto, m_vpix.x, m_vpix.y, m_out_img);	
		break;
	case 'c': {
		Vector3DF h(-1,0,0);
		for (int n=0; n < m_debugvis.size(); n++) {
			Vector4DF c = m_debugvis[n].clr;
			if ( m_debugvis[n].type == 'p' && c.x==1 && c.y==0 && c.z==0) 
				h = m_debugvis[n].a;
		}
		if (h.x != -1)
			m_cam->setOrbit ( m_cam->getAng(), h, m_cam->getOrbitDist(), m_cam->getDolly() );
		} break;

	// nudge vis debug pixel
	case KEY_UP:		RaytraceDebug(m_vfrom, m_vto, m_vpix.x, m_vpix.y-1, m_out_img);	break;
	case KEY_DOWN:		RaytraceDebug(m_vfrom, m_vto, m_vpix.x, m_vpix.y+1, m_out_img);	break;
	case KEY_LEFT:		RaytraceDebug(m_vfrom, m_vto, m_vpix.x-1, m_vpix.y, m_out_img);	break;
	case KEY_RIGHT:		RaytraceDebug(m_vfrom, m_vto, m_vpix.x+1, m_vpix.y, m_out_img);	break;
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
	appStart("Displacement Map Sphere", "Displacement Map Sphere", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}




