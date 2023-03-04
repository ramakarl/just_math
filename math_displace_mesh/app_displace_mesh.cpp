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
	Vector3DF ve0,ve1,ve2;			// prism volume, see defines above
	Vector3DF bmin, bmax;			// bounding box of prism
	Vector3DF norm;					// normal of facing triangle
};

// hit candidate info
struct hitinfo_t {					
	hitinfo_t (int f, int s, float t0, float t1) {face=f; side=s; tmin=t0; tmax=t1;}
	int	face;
	int side;
	float tmin, tmax;		
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

	void		ConstructPrisms ( float d );
	bool		FindPrismHitCandidates ( Vector3DF rpos, Vector3DF rdir, std::vector<hitinfo_t>& candidates );
	bool		IntersectSurface ( int face, int edge, Vector3DF rpos, Vector3DF rdir, float displace_depth, float tmin, float tmax, float dt, float& t, Vector3DF& bc );
	bool		ShadeSurface ( int face, Vector3DF bc, float displace_depth, Vector3DF& n, Vector3DF& clr );

	void		RaytraceMesh ( float bump_depth, Image* img );
	void		RaytraceDisplacementMesh ( float bump_depth, Image* img );	
	void		RaytraceDebug ( Vector3DF from, Vector3DF to, int x, int y, Image* img );	

	void		DrawGrid ();	
	void		UpdateCamera();
	void		MoveCamera ( char t, Vector3DF del );

	void		VisDot ( Vector3DF p, float r, Vector3DF clr );
	void		VisLine ( Vector3DF a, Vector3DF b, Vector3DF clr );

	Camera3D*	m_cam;				// camera
	Vector3DF   m_lgt;
	Image*		m_bump_img;			// bump map
	Image*		m_color_img;		// color map
	Image*		m_out_img;	

	MeshX*		m_mesh;				// mesh geometry

	std::vector<Prism>	m_prisms;	// prism geometry

	Mersenne	m_rand;

	float		m_bump_depth;
	float		m_time;
	int			m_samples;
	bool		m_run, m_displace;
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
	


	int			m_currcam;
};
Sample obj;


bool Sample::init()
{
	addSearchPath(ASSET_PATH);
	init2D("arial");
	setText(24,1);
	
	m_time = 0;
	m_run = false;	
	m_venable = 0;
	m_displace = true;
	m_jitter_sample = false;
	m_draw_prisms = false;

	m_rand.seed ( 123 );

	// create a camera
	m_cam = new Camera3D;								
	m_cam->setFov ( 40 );
	m_cam->setAspect ( 1 );
	
	m_cam->setOrbit ( 26, 38, 0, Vector3DF(0,0,0), 6, 1 );

	dbgprintf ( "from: %f %f %f\n", m_cam->getPos().x, m_cam->getPos().y, m_cam->getPos().z );

//	m_cam->setOrbit ( -57, 16, 0, Vector3DF(-0.3, 0.3664, -0.7), 1 , 1 );
	
	m_lgt = Vector3DF(-90, 400, 220);
	

	m_currcam = 0;

	int res = 1024;

	// set bump depth
	m_bump_depth = 0.30;

	// load displacement map 
	std::string fpath;	
	//getFileLocation ( "stones_displace.png", fpath );
	getFileLocation ( "rocks01_displace.png", fpath );
	//getFileLocation ( "rocks02_displace.png", fpath );
	//getFileLocation ( "rocks07_displace.png", fpath );
	//getFileLocation ( "gravel02_displace.png", fpath );
	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_bump_img = new Image;
	if ( !m_bump_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}

	//getFileLocation ( "stones_color.png", fpath );
	getFileLocation ( "rocks01_color.png", fpath );
	//getFileLocation ( "rocks01_displace.png", fpath );
	//getFileLocation ( "rocks02_color.png", fpath );
	//getFileLocation ( "rocks07_color.png", fpath );
	//getFileLocation ( "gravel02_color.png", fpath );
	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_color_img = new Image;
	if ( !m_color_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}

	// load mesh
	getFileLocation ( "s000.obj", fpath );
	//getFileLocation ( "surface.obj", fpath );
	//getFileLocation ( "asteroid.obj", fpath );
	dbgprintf ( "Loading: %s\n", fpath.c_str());
	m_mesh = new MeshX;
	if ( !m_mesh->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-3);
	}

	// construct prisms
	ConstructPrisms ( m_bump_depth );
	
	// create output image
	m_out_img = new Image;
	m_out_img->ResizeImage ( res, res, ImageOp::RGBA8 );	
	
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
	
	m_rand.seed ( m_samples );
	
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
				tx = m_bump_img->GetPixelUV ( uv.x, uv.y ).x;				
	
				// interpolate normal
				n1 = *m_mesh->GetVertNorm( f->v1 );	n2 = *m_mesh->GetVertNorm( f->v2 );	n3 = *m_mesh->GetVertNorm( f->v3 );
				norm = n1 * float(bc.x) + n2 * float(bc.y) + n3 * float(1-bc.x-bc.y);
				norm.Normalize();

				// diffuse shading
				diffuse = std::max(0.0, norm.Dot( m_lgt ));
				c = m_color_img->GetPixelFilteredUV ( uv.x, uv.y );
				c = c * float(diffuse * 255.0);
				if (c.x > 255 || c.y > 255 || c.z > 255) c = Vector3DF(255,255,255);
					
				img->SetPixel ( x, y, c.x, c.y, c.z, 255.0 );	
			} 
		}
	}

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_GLTEX );		
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

bool intersectRayTriangleInOut ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& tmin, float& tmax, float& alpha, float& beta )    
{
	Vector3DF e0 = v2 - v1;
    Vector3DF e1 = v2 - v0;
    Vector3DF n = e1.Cross ( e0 );
	float ndotr = n.Dot( dir );	
	Vector3DF e2 = (v2 - orig) / ndotr;
	float t = n.Dot ( e2 );
	if ( t<0.0 ) return false;

	Vector3DF i = dir.Cross ( e2 );
	alpha =	i.x*e0.x + i.y*e0.y + i.z*e0.z;	
	beta =	i.x*-e1.x + i.y*-e1.y + i.z*-e1.z;		
	if ( alpha<0.0 || beta<0.0 || (alpha+beta>1)) return false;  
	if ( ndotr >=0 && t > tmax ) {tmax = t; return false;}	  // backfacing
	if ( ndotr < 0 && t < tmin ) {tmin = t; return true;}     // facing the ray	
	return false;
}


bool intersectRayPrism ( Vector3DF rpos, Vector3DF rdir, Vector3DF vb0, Vector3DF vb1, Vector3DF vb2, Vector3DF ve0, Vector3DF ve1, Vector3DF ve2, 
                         int& be, float& tmin, float& tmax, float& alpha, float& beta )
{
	tmin = 1e10;
	tmax = 0;
    if ( intersectRayTriangleInOut ( rpos, rdir, ve0, ve1, ve2, tmin, tmax, alpha, beta ) ) be=0;
	if ( intersectRayTriangleInOut ( rpos, rdir, vb0, vb2, vb1, tmin, tmax, alpha, beta ) ) be=1; 
	if ( intersectRayTriangleInOut ( rpos, rdir, vb0, ve1, ve0, tmin, tmax, alpha, beta ) ) be=3;
	if ( intersectRayTriangleInOut ( rpos, rdir, vb0, vb1, ve1, tmin, tmax, alpha, beta ) ) be=3; 
	if ( intersectRayTriangleInOut ( rpos, rdir, vb1, ve2, ve1, tmin, tmax, alpha, beta ) ) be=4; 
	if ( intersectRayTriangleInOut ( rpos, rdir, vb1, vb2, ve2, tmin, tmax, alpha, beta ) ) be=4;
	if ( intersectRayTriangleInOut ( rpos, rdir, vb2, ve0, ve2, tmin, tmax, alpha, beta ) ) be=5;
	if ( intersectRayTriangleInOut ( rpos, rdir, vb2, vb0, ve0, tmin, tmax, alpha, beta ) ) be=5;    
	if (tmax>0) {
		if (tmin==1e10) be = 2;	// no front hit, ray is starting inside
		return true;
	}
    return false;
}

void Sample::ConstructPrisms ( float depth )
{
	// Precompute prism geometry
	// - gives good acceleration. necessary for CPU version
	// - these are pushed into array that matches the enumeration of faces in the mesh	

	Vector3DF vb0, vb1, vb2;		// prism geometry
	Vector3DF vn0, vn1, vn2;	

	Prism s;
	AttrV3* f;
	for (f = (AttrV3*) m_mesh->GetStart(BFACEV3); f <= (AttrV3*) m_mesh->GetEnd(BFACEV3); f++ ) {

		vb0 = *m_mesh->GetVertPos( f->v1 );	vb1 = *m_mesh->GetVertPos( f->v2 );	vb2 = *m_mesh->GetVertPos( f->v3 );
		vn0 = *m_mesh->GetVertNorm( f->v1 ); vn1 = *m_mesh->GetVertNorm( f->v2 ); vn2 = *m_mesh->GetVertNorm( f->v3 );				
		s.norm = (vb2 - vb1).Cross ( vb0 - vb2 ); 
		s.norm.Normalize();
	
		// Extension of face outward along normal
		//--- using unmodified vertex normals

		//s.ve0 = vb0 + vn0 * depth;
		//s.ve1 = vb1 + vn1 * depth;
		//s.ve2 = vb2 + vn2 * depth;

		s.ve0 = vb0 + vn0 * depth;
		s.ve1 = vb1 + vn1 * depth;
		s.ve2 = vb2 + vn2 * depth;
		
		//--- using corrected vertex normals to get uniform height		
		/*s.v[VEXT+0] = s.v[VBASE+0] + s.v[VNORM+0] * float(s.v[VNORM+0].Dot( s.norm ) * d); 
		s.v[VEXT+1] = s.v[VBASE+1] + s.v[VNORM+1] * float(s.v[VNORM+1].Dot( s.norm ) * d); 
		s.v[VEXT+2] = s.v[VBASE+2] + s.v[VNORM+2] * float(s.v[VNORM+2].Dot( s.norm ) * d);	*/

		// Prism bounding box		
		ComputePrismBounds ( vb0, vb1, vb2, s.ve0, s.ve1, s.ve2, s.bmin, s.bmax );

		m_prisms.push_back ( s );
	}
}


bool Sample::FindPrismHitCandidates ( Vector3DF rpos, Vector3DF rdir, std::vector<hitinfo_t>& candidates )
{
	Prism* s;
	AttrV3* f;
	Vector3DF vb0, vb1, vb2;
	Vector3DF n;
	int face, edge;
	float tmin, tmax;
	
	// clear candidates
	candidates.clear ();

	// search all prisms (faces) for potential hit
	f = (AttrV3*) m_mesh->GetStart(BFACEV3);

	int minf = 0;
	int maxf = m_mesh->GetNumFace3()-1;
	float t;

	for (int i=minf; i <= maxf; f++, i++ ) {

		//if (i != 25  ) continue;

		s = &m_prisms[i];
		vb0 = *m_mesh->GetVertPos( f->v1 );	vb1 = *m_mesh->GetVertPos( f->v2 );	vb2 = *m_mesh->GetVertPos( f->v3 );
		
		// fast bounding box check..
		if ( intersectLineBox ( rpos, rdir, s->bmin, s->bmax, t ) ) {
			
			// intersect with triangular prism
			float alpha, beta;
			if ( intersectRayPrism ( rpos, rdir, vb0,vb1,vb2,s->ve0,s->ve1,s->ve2, edge, tmin, tmax, alpha, beta) ) {
				// add to potential hit list
				face = f-(AttrV3*) m_mesh->GetStart(BFACEV3);
				candidates.push_back ( hitinfo_t( face, edge, tmin, tmax ) );
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

Vector3DF getBarycentricInTriangle ( Vector3DF& hit, Vector3DF& ve0, Vector3DF& ve1, Vector3DF& ve2 )
{
	Vector3DF q0, q1, q2, bc;
	float d00, d01, d11, d20, d21, invDenom;
	q0 = ve1-ve0; q1 = ve2-ve0; q2 = hit - ve0;
	d00 = q0.Dot( q0);
	d01 = q0.Dot( q1);
	d11 = q1.Dot( q1);
	d20 = q2.Dot( q0);
	d21 = q2.Dot( q1);
	invDenom = 1.0 / (d00 * d11 - d01 * d01);
	bc.y = (d11 * d20 - d01 * d21) * invDenom;
	bc.z = (d00 * d21 - d01 * d20) * invDenom;
	bc.x = 1.0-bc.y-bc.z;
	return bc;
} 


#define EPS     0.0

bool Sample::IntersectSurface ( int face, int edge, Vector3DF rpos, Vector3DF rdir, float displace_depth, float tmin, float tmax, float dt, float& t, Vector3DF& bc )
{
	AttrV3* f;	
	Prism* s;
	Vector3DF vb0, vb1, vb2;		// prism geometry
	Vector3DF ve0, ve1, ve2;
	Vector3DF vd0, vd1, vd2;
	Vector3DF vn0, vn1, vn2;
	Vector3DF vuv0, vuv1, vuv2;
	Vector3DF q0, q1, q2;

	Vector3DF n, bcl, hit, uv, p;				
	float alpha, beta, rayhgt, raynh, bmphgt, df, de;

	f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);	
	s = &m_prisms[ face ];
	vb0 = *m_mesh->GetVertPos( f->v1 );	vb1 = *m_mesh->GetVertPos( f->v2 );	vb2 = *m_mesh->GetVertPos( f->v3 );
	vn0 = *m_mesh->GetVertNorm( f->v1 ); vn1 = *m_mesh->GetVertNorm( f->v2 ); vn2 = *m_mesh->GetVertNorm( f->v3 );		
	vuv0 = *m_mesh->GetVertTex( f->v1 ); vuv1 = *m_mesh->GetVertTex( f->v2 ); vuv2 = *m_mesh->GetVertTex( f->v3 );
	ve0 = s->ve0; ve1 = s->ve1; ve2 = s->ve2;

	// Initial prism hit estimate
	t = tmin;
	hit = rpos + rdir * tmin;	

	// Fractional change in ray height distance to triangle along normal
	// - dr/dy = norm . ray dt   (sample spacing along vertical axis)
	// - df = (dr/dy) / depth    (spacing as a percentage of total depth)
	df = s->norm.Dot ( rdir * -dt );	
	vd0 = (vb0 - ve0) * df / displace_depth;
	vd1 = (vb1 - ve1) * df / displace_depth;
	vd2 = (vb2 - ve2) * df / displace_depth;

	// Project hit point to base triangle
	// - p = proj(hit)   (perpendicular projection of initial hit to base triangle)	
	switch (edge) {	
	case 0:			// top triangle
		raynh = 1.0;
		break;
	case 1:			// bottom triangle
		raynh = 0.0;
		break;
	case 2:			// starting inside prism
		raynh = 1;
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
	bc = getBarycentricInTriangle  ( hit, ve0, ve1, ve2 );	

	// Ensure valid bcs	
	bc.x = (bc.x < 0) ? 0 : (bc.x > 1) ? 1 : bc.x;
	bc.y = (bc.y < 0) ? 0 : (bc.y > 1) ? 1 : bc.y;	
	bc.z = 1-bc.x-bc.y;
	bc.z = (bc.z < 0) ? 0 : (bc.z > 1) ? 1 : bc.z;             	
	bcl = bc;

	// Get ray height
	p = vb0 * bc.x + vb1 * bc.y + vb2 * bc.z;			
	rayhgt = (hit-p).Length();	

	// Sample displacement texture for initial surface height			
	uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;				
	bmphgt = displace_depth * m_bump_img->GetPixelUV ( uv.x, uv.y ).x;

	// [optional] visualization	
	if ( m_vcurr==1 ) {		
		VisLine ( q0, q1, m_vhit );		
		VisDot ( hit, 1, m_vhit );			// initial prism hit				
		VisDot ( p, 1, m_vbase );			// visualize base points (green)
		VisLine ( ve0, ve1, m_vscan );		// visualize scanning triangle
		VisLine ( ve1, ve2, m_vscan );
		VisLine ( ve2, ve0, m_vscan );			
	}

	// March ray until it hits or leaves prism
	//
	for (t=tmin; rayhgt >= bmphgt && t < tmax; t += dt) {

		// march along ray
		hit += rdir * dt;							

		// point-to-plane distance
		q0 = (ve2-ve1).Cross ( ve0-ve2 );
		de = q0.Dot( ve0 - hit ) / df;

		// advance scanning triangle 
		ve0 += vd0 * de;
		ve1 += vd1 * de;
		ve2 += vd2 * de;

		// barycentric coords on triangle
		bcl = bc;
		bc = getBarycentricInTriangle  ( hit, ve0, ve1, ve2 );	
		bc.z = (edge==3 && bc.z < 0) ? 0 : bc.z;
		bc.x = (edge==4 && bc.x < 0) ? 0 : bc.x;
		bc.y = (edge==5 && bc.y < 0) ? 0 : bc.y;

		// ray height
		p = vb0*bc.x + vb1*bc.y + vb2*bc.z;			
		rayhgt = (hit-p).Length();

		// sample bump map to get displacement height
		uv = vuv0*bc.x + vuv1*bc.y + vuv2*bc.z;
		bmphgt = displace_depth * m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x;

		// [optional] visualization
		if ( m_vcurr==1 ) {						
			VisDot ( hit, .5, m_vsample );		// sample points 
			VisDot ( p, .5, m_vbase );			// base points 			
			Vector3DF a = p + (hit-p).Normalize() * bmphgt;
			VisDot ( a, .5, m_vsurf );			// surface points 
		}
	}

	// Check for miss or t out-of-bounds
	if ( rayhgt > bmphgt || t >= tmax ) 
		return false;
	
	// Hit surface

	// Linear interpolation to find surface
	// we have current sample already - below surface					
	// get previous sample - above surface		
	uv = vuv0 * bcl.x + vuv1 * bcl.y + vuv2 * bcl.z;
	p = vb0*bcl.x + vb1*bcl.y + vb2*bcl.z;					
	float rayhgt0 = ( (hit-rdir*dt) - p).Length();
	float bmphgt0 = displace_depth * m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x;
	// interpolate between previous and current sample
	float u = (rayhgt0-bmphgt0) / ((rayhgt0-bmphgt0)+(bmphgt-rayhgt));
	u = (u<0) ? 0 : (u>1) ? 1 : u;									
					
	// Final hit t
	t = (hit - rpos).Length() - dt + u*dt;
	
	hit = rpos + rdir * t;

	// Final update of bcs	
	q0 = (ve2-ve1).Cross ( ve0-ve2 ); 
	de = q0.Dot( ve0 - hit ) / df;
	ve0 += vd0 * de;
	ve1 += vd1 * de;
	ve2 += vd2 * de;
	bc = getBarycentricInTriangle  ( hit, ve0, ve1, ve2 );	

	return true;	
}

bool Sample::ShadeSurface ( int face, Vector3DF bc, float displace_depth, Vector3DF& n, Vector3DF& clr )
{
	AttrV3* f;	
	Prism* s;

	Vector3DF vb0, vb1, vb2;			// prism geometry
	Vector3DF vn0, vn1, vn2;
	Vector3DF vuv0, vuv1, vuv2, uv;	
	Vector3DF q0, q1, q2;
	float dpu, dpv;

	Vector3DF L, R, V, c, n0;			// shading rays
	float diffuse, spec;

	// Get the prism and face
	f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);	
	s = &m_prisms[ face ];	
	vb0 = *m_mesh->GetVertPos( f->v1 );	vb1 = *m_mesh->GetVertPos( f->v2 );	vb2 = *m_mesh->GetVertPos( f->v3 );
	vn0 = *m_mesh->GetVertNorm( f->v1 ); vn1 = *m_mesh->GetVertNorm( f->v2 ); vn2 = *m_mesh->GetVertNorm( f->v3 );	
	vuv0 = *m_mesh->GetVertTex( f->v1 ); vuv1 = *m_mesh->GetVertTex( f->v2 ); vuv2 = *m_mesh->GetVertTex( f->v3 );	

	// Construct normal from displacement surface
	dpu = 1.0 / (m_bump_img->GetWidth()-1);
	dpv = 1.0 / (m_bump_img->GetHeight()-1);

	// interpolated base normal
    n0 = vn0 * bc.x + vn1 * bc.y + vn2 * bc.z;
    n0.Normalize();

	// central sample
    uv = vuv0*bc.x + vuv1*bc.y + vuv2 * bc.z;
	if (uv.x<0 || uv.y<0 || uv.x>1 || uv.y>1) return false;
    q0 = vb0 * bc.x + vb1 * bc.y + vb2 * bc.z;
    q0 += n0 * (displace_depth *  m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x);

	// color texture
	clr = m_color_img->GetPixelFilteredUV ( uv.x, uv.y );	
	
	// sample along barycentric u
	uv = vuv0*(bc.x+dpu) + vuv1*bc.y + vuv2 * (1-(bc.x+dpu)-bc.y);    
    q1 = vb0 * (bc.x+dpu) + vb1 * bc.y + vb2 * (1-(bc.x+dpu)-bc.y);
    q1 += n0 * (displace_depth *  m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x);	

    // sample along barycentric v	
    uv = vuv0*(bc.x) + vuv1*(bc.y+dpu) + vuv2 * (1-bc.x-(bc.y+dpu));                
    q2 = vb0 * bc.x + vb1 * (bc.y+dpu) + vb2 * (1-bc.x-(bc.y+dpu));
    q2 += n0 * (displace_depth *  m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x );            

	if (m_vcurr==2) {
		VisDot ( q0, 0.1, m_vhit );
		VisDot ( q1, 0.1, m_vsurf );
		VisLine( q1, q0, m_vsurf);
		VisDot ( q2, 0.1, m_vsurf );
		VisLine( q2, q0, m_vsurf );
	}

	// Compute normal as cross product of surface displacements
	q1 -= q0;
	q2 -= q0;
	n = q1.Cross (q2); n.Normalize();

	if (m_vcurr==2) {
		VisLine (q0, q0 + n*dpu, m_vhit );
	}
				
	// Diffuse shading	
	L = m_lgt - q0; L.Normalize();					
	diffuse = 0.7 * std::max(0.0, n.Dot( L ));
	R = n * float(2 * n.Dot(L)) - L; R.Normalize();
	V = q0 - m_cam->getPos(); V.Normalize();
	spec = 0.3 * pow ( R.Dot( V ), 20 );

	// colored final surface			
	clr = clr * diffuse + Vector3DF(spec,spec,spec);

	// world xyz
	// clr = Vector3DF(q0.x, q0.y, q0.z)*0.3f + 0.5f;

	// surface normal
	//clr = n*0.5f + 0.5f;

	// barycentric
	// clr = Vector3DF(bc.x, bc.y, 1-bc.x-bc.y);

	// interpolated base normal
	//clr = Vector3DF(1,1,1) * 0.5f * float( pow( n0.Dot(L), 3) );



	clr *= 255.0;
	if (clr.x > 255 || clr.y > 255 || clr.z > 255) clr = Vector3DF(255,255,255);

	return true;
}

void Sample::RaytraceDisplacementMesh ( float bump_depth, Image* img )
{
	Prism* s;
	AttrV3* f;
	Vector3DF vb0, vb1, vb2;		// prism geometry
	Vector3DF vn0, vn1, vn2;	
	Vector3DF rpos, rdir, n, bc;
	Vector3DF clr(1,1,1);

	// triangle search setup
	int face, edge, best_f;
	float t, best_t;
	Vector3DF best_bc;
	std::vector<hitinfo_t> candidates;

	// ray setup
	int xres = img->GetWidth();
	int yres = img->GetHeight();	
	
	m_rand.seed ( m_samples );

	// surface setup
	float displace_depth = m_bump_depth;

	float dt = 0.01;
	float doff = 0.0;
	bool used;


	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {

			// Get camera ray
			rpos = m_cam->getPos();
			rdir = m_cam->inverseRay ( x, y, xres, yres );	
			rdir.Normalize();

			// Collect candidates for hit
			if (! FindPrismHitCandidates ( rpos, rdir, candidates ) )
				continue;

			// Check candidates for displacement hit			
			best_t = 1.0e10;
			best_f = -1;	
			t = 0;			
			for (int i=0; i < candidates.size(); i++ ) {
				// Look for intersection using this prism
				face = candidates[i].face;
				
				if ( IntersectSurface ( face, candidates[i].side, rpos, rdir, displace_depth, candidates[i].tmin, candidates[i].tmax, dt, t, bc ) ) {
					if ( t < best_t ) {
						best_t = t;						
						best_f = face;
						best_bc = bc;
					}				
				}
			} 				

			if ( best_f >= 0 ) {								
				// Hit found. Shade surface.
				
				ShadeSurface ( best_f, best_bc, displace_depth, n, clr );

				if ( m_jitter_sample )
					clr = (img->GetPixel(x,y) * float(m_samples-1) + clr)/m_samples;					

				//-- debug prism sides
				//if (best_s !=-1 ) c += Vector3DF( (best_s==0), (best_s==1), (best_s==2) ) *20.0f;

				// Write pixel
				img->SetPixel ( x, y, clr.x, clr.y, clr.z, 255.0 );

			}

		}	// x pixels
	}		// y pixels 


	m_samples++;

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_GLTEX );		
}



void Sample::RaytraceDebug ( Vector3DF from, Vector3DF to, int x, int y, Image* img )
{
	Prism* s;
	AttrV3* f;
	Vector3DF vb0, vb1, vb2;		// prism geometry	
	Vector3DF ve0, ve1, ve2;
	Vector3DF vn0, vn1, vn2;	
	Vector3DF rpos, rdir, n, clr, bc, hit;	

	// triangle search setup
	int face, edge, best_f;
	float t, best_t;
	Vector3DF best_bc;
	std::vector<hitinfo_t> candidates;
	
	// ray setup
	int xres = img->GetWidth();
	int yres = img->GetHeight();

	m_rand.seed ( m_samples );

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
	float displace_depth = m_bump_depth;
	float dt = 0.005;
	float doff = 0.0;
	bool used;

	// Collect candidates for hit
	if (! FindPrismHitCandidates ( rpos, rdir, candidates ) )
		return;

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
	t = 0;			
	for (int i=0; i < candidates.size(); i++ ) {

		// Look for intersection using this triangle				
		face = candidates[i].face;
		f = (AttrV3*) m_mesh->GetElem(BFACEV3, face);
		s = &m_prisms[ face ];
		vb0 = *m_mesh->GetVertPos( f->v1 );	vb1 = *m_mesh->GetVertPos( f->v2 );	vb2 = *m_mesh->GetVertPos( f->v3 );
		ve0 = s->ve0; ve1 = s->ve1; ve2 = s->ve2;

		if ( m_vcurr==1 ) {
			// Visualize candidate prisms
			VisLine ( vb0, vb1, Vector3DF(0,.5,0) );
			VisLine ( vb1, vb2, Vector3DF(0,.5,0) );
			VisLine ( vb2, vb0, Vector3DF(0,.5,0) );
			VisLine ( ve0, ve1, Vector3DF(.5,.5,.5) );
			VisLine ( ve1, ve2, Vector3DF(.5,.5,.5) );
			VisLine ( ve2, ve0, Vector3DF(.5,.5,.5) );
			VisLine ( rpos, rpos+rdir*10, Vector3DF(1,1,1) );	
		
		
			if (i % 2 == 0) {			
				// first triangle
				m_vhit.Set( 1, 0, 0);			// red
				m_vscan.Set( 1, 0, 0);			// red
				m_vsample.Set( 1, .5, 0);		// orange
				m_vbase.Set(0, 1, 0);			// green
				m_vsurf.Set(1, 0, 1);			// purple
			} else {
				// other triangles
				m_vhit.Set( 1, 1, 0);			// yellow
				m_vscan.Set( 1, 1, 0);			// yellow
				m_vsample.Set( 0, 1, 1);		// cyan
				m_vbase.Set(0, 0, 1);			// blue
				m_vsurf.Set(1, 0, 1);			// purple
			}		
		}
		
		if ( IntersectSurface ( face, candidates[i].side, rpos, rdir, displace_depth, candidates[i].tmin, candidates[i].tmax, dt, t, bc ) ) {
			if ( t < best_t ) {
				best_t = t;				
				best_bc = bc;
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

		hit = rpos + rdir * t;
		VisDot ( hit, 1.1, Vector3DF(1,1,1) );

		Vector3DF n, clr;
		ShadeSurface ( best_f, best_bc, displace_depth, n, clr );
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
	
	if ( m_samples==1 || m_jitter_sample) {
		t1 = clock();

		if (m_displace) 
			RaytraceDisplacementMesh ( m_bump_depth, m_out_img );
		else
			RaytraceMesh ( m_bump_depth, m_out_img );
		
		t2 = clock();	
		dbgprintf ( "Time: %f msec\n", float(t2-t1)/float(CLOCKS_PER_SEC/1000.0f) );
	}	

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
	end2D();	
	draw2D();										// complete 2D rendering to OpenGL

	V = m_cam->getDir ();
	V.Normalize();
	

	// Draw debug visualization
	start3D( m_cam );
	setview2D(getWidth(), getHeight());	
	for (int i=0; i < m_debugvis.size(); i++) {
		if ( m_debugvis[i].type == 'p' ) {
			vb0 = m_debugvis[i].a; vb1 = m_debugvis[i].b;
			drawCircle3D ( vb0, vb0+V, 0.005 * vb1.x, m_debugvis[i].clr );
		}
	}
	for (int i=0; i < m_debugvis.size(); i++) {
		if ( m_debugvis[i].type == 'l' ) {
			vb0 = m_debugvis[i].a; vb1 = m_debugvis[i].b;
			drawLine3D ( vb0, vb1, m_debugvis[i].clr );
		}
	}
	end3D();


	start3D( m_cam );
	setview2D(getWidth(), getHeight());

	// Draw prisms in 3D		
	if ( m_draw_prisms || m_venable > 0) {

		for (int i = 0; i < m_prisms.size(); i++)  {
			s = &m_prisms[i];
			f = (AttrV3*) m_mesh->GetElem (BFACEV3, i );
			vb0 = *m_mesh->GetVertPos( f->v1 );	vb1 = *m_mesh->GetVertPos( f->v2 );	vb2 = *m_mesh->GetVertPos( f->v3 );
			ve0 = s->ve0;  ve1 = s->ve1;  ve2 = s->ve2;

			if ( s->norm.Dot ( V ) < 0 ) {
				//drawLine3D ( b0, b1, clr ); drawLine3D ( b1, b2, clr );	drawLine3D ( b2, b0, clr );
				drawLine3D ( vb0, ve0, clr ); drawLine3D ( vb1, ve1, clr );	drawLine3D ( vb2, ve2, clr );			
				drawLine3D ( ve0, ve1, clr ); drawLine3D ( ve1, ve2, clr );	drawLine3D ( ve2, ve0, clr );			
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

		appPostRedisplay();	// Update display
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
		m_samples = 0;
		UpdateCamera();
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

		if ( m_debugvis.size()>0) {			
			Vector3DF to = m_debugvis[ m_debugvis.size()-1 ].a;
			float orbit = 2.0;
			m_cam->setOrbit ( m_cam->getAng(), to, orbit, 1 );
			UpdateCamera ();
		}
	}
}

void Sample::mousewheel(int delta)
{
	// Adjust zoom
	int i = m_currcam;

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
	case 'm': 
		m_displace = !m_displace; 
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




