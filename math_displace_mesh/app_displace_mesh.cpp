//--------------------------------------------------------
// JUST MATH:
// Displace Mesh- raycast displacement mapping on a triangle mesh
//
// Inspired by the paper:
//    Tesselation-Free Displacement Mapping for Raytracing, Thonat et al, 2021
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

#define VBASE		0
#define VEXT		3
#define VDEL		6
#define VNORM		9
#define VUV			12
#define VMAX		15

struct Prism {
	Prism()	{};
	Vector3DF v[VMAX];				// prism volume, see defines above
	Vector3DF bmin, bmax;			// bounding box of prism
	Vector3DF norm;					// normal of facing triangle
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

	void		PrecomputePrisms ( float d );

	void		RaycastMesh ( Vector3DF p, float r, float bump_depth, Vector3DF lgt, Image* img );
	void		RaycastDisplacementMesh ( Vector3DF p, float r, float bump_depth, Vector3DF lgt, Image* img );
	void		RaycastDisplacementMeshORIG ( Vector3DF p, float r, float bump_depth, Vector3DF lgt, Image* img );

	void		DrawMesh ( float bump_depth );
	void		DrawGrid ();	

	void		UpdateCamera();

	Camera3D*	m_cam;				// camera
	Image*		m_bump_img;			// bump map
	Image*		m_color_img;		// color map
	Image*		m_out_img;
	Image*		m_hit_img;

	MeshX*		m_mesh;
	std::vector<Prism>	m_prisms;

	Mersenne	m_rand;

	float		m_bump_depth;
	float		m_time;
	int			m_samples;
	bool		m_run, m_displace;
	bool		m_jitter_sample, m_draw_prisms;
	int			mouse_down;

};
Sample obj;


bool Sample::init()
{
	addSearchPath(ASSET_PATH);
	init2D("arial");
	setText(24,1);
	
	m_time = 0;
	m_run = false;	
	m_displace = true;
	m_jitter_sample = true;
	m_draw_prisms = false;
	
	m_bump_depth = 0.10;

	m_rand.seed ( 123 );

	int res = 1024;

	// create a camera
	m_cam = new Camera3D;								
	m_cam->setOrbit ( 30, 40, 0, Vector3DF(0,0,0), 10, 1 );

	// load displacement map 
	std::string fpath;	
	getFileLocation ( "stones_displace.png", fpath );
	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_bump_img = new Image;
	if ( !m_bump_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}

	getFileLocation ( "stones_color.png", fpath );
	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_color_img = new Image;
	if ( !m_color_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}

	// load mesh
	getFileLocation ( "surface.obj", fpath );
	dbgprintf ( "Loading: %s\n", fpath.c_str());
	m_mesh = new MeshX;
	if ( !m_mesh->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-3);
	}

	// precompute prisms
	PrecomputePrisms ( m_bump_depth );
	
	// create output image
	m_out_img = new Image;
	m_out_img->ResizeImage ( res, res, ImageOp::RGBA32 );	
	
	m_hit_img = new Image;
	m_hit_img->ResizeImage ( res, res, ImageOp::F8 );
	
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

void Sample::RaycastMesh ( Vector3DF p, float r, float bump_depth, Vector3DF lgt, Image* img )
{
	float t, tx, diffuse, h;
	AttrV3* f;
	Vector3DF v1,v2,v3;
	Vector3DF n1,n2,n3;
	Vector2DF uv1, uv2, uv3, uv, bc;
	Vector3DF a,b,c;
	Vector3DF hit, norm;	
	Vector3DF rpos, rdir;
	int best_f;
	float best_t;
	Vector3DF best_hit;
	int raycost;
	int i;
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

			// intersect with each triangle in mesh
			best_t = 1.0e10;
			best_f = -1;
			
			for (f = (AttrV3*) m_mesh->GetStart(BFACEV3); f <= (AttrV3*) m_mesh->GetEnd(BFACEV3); f++ ) {
				v1 = *m_mesh->GetVertPos( f->v1 );	v2 = *m_mesh->GetVertPos( f->v2 );	v3 = *m_mesh->GetVertPos( f->v3 );		
				n1 = *m_mesh->GetVertNorm( f->v1 );	n2 = *m_mesh->GetVertNorm( f->v2 );	n3 = *m_mesh->GetVertNorm( f->v3 );
			
				// intersect with extended triangle 
				if ( intersectRayTriangle ( rpos, rdir, v1, v2, v3, t, hit ) ) {
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
				intersectRayTriangleUV ( rpos, rdir, v1, v2, v3, t, hit, bc.x, bc.y );

				// interpolate uv coords from barycentric
				uv = uv1 * (bc.x) + uv2 * (bc.y) + uv3 * (1-bc.x-bc.y);
				
				// read texture
				tx = m_bump_img->GetPixelUV ( uv.x, uv.y ).x;				
	
				// interpolate normal
				n1 = *m_mesh->GetVertNorm( f->v1 );	n2 = *m_mesh->GetVertNorm( f->v2 );	n3 = *m_mesh->GetVertNorm( f->v3 );
				norm = n1 * (bc.x) + n2 * (bc.y) + n3 * (1-bc.x-bc.y);
				norm.Normalize();

				// diffuse shading
				diffuse = std::max(0.0, norm.Dot( lgt ));
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


void ComputePrismBounds ( Vector3DF* v, Vector3DF& bmin, Vector3DF& bmax )
{
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

bool intersectRayPrism ( Vector3DF rpos, Vector3DF rdir, Vector3DF* v, float& t, Vector3DF& hit, int& e )
{
	float ndot, u;
	Vector3DF norm;
	Vector3DF bc0, bc1, bc2;
	Vector3DF p0, p1;

	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+0], v[VEXT+1], v[VEXT+0], t, hit )) {e=0; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+0], v[VBASE+1],v[VEXT+1], t, hit )) {e=0; return true;}

	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+1], v[VEXT+2], v[VEXT+1], t, hit )) {e=1; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+1], v[VBASE+2],v[VEXT+2], t, hit )) {e=1; return true;}

	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+2], v[VEXT+0], v[VEXT+2], t, hit )) {e=2; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+2], v[VBASE+0],v[VEXT+0], t, hit )) {e=2; return true;}
	
	return false;

	//-------- Barycentric method, not quite working, but could be faster..
	/* Vector3DF e0 = v[VBASE+0] - v[VBASE+2];
	Vector3DF e1 = v[VBASE+2] - v[VBASE+1];
	
	norm = e1.Cross ( e0 ); norm.Normalize();
	p0 = intersectLinePlane (rpos, rpos+rdir, v[VBASE+0], norm );
	p1 = intersectLinePlane (rpos, rpos+rdir, v[VEXT+0], norm );

	norm = Vector3DF::CrossFunc( e1, e0 ); ndot = norm.Dot(norm);					
	bc0.x = Vector3DF::CrossFunc( e1, p0 - v[VBASE+1] ).Dot(norm) / ndot;
	bc0.y = Vector3DF::CrossFunc( e0, p0 - v[VBASE+2] ).Dot(norm) / ndot;	
	bc0.z = 1-bc0.x-bc0.y;

	e0 = v[VEXT+0] - v[VEXT+2];
	e1 = v[VEXT+2] - v[VEXT+1];
	norm = Vector3DF::CrossFunc( e1, e0 ); ndot = norm.Dot(norm);					
	bc1.x = Vector3DF::CrossFunc( e1, p1 - v[VEXT+1] ).Dot(norm) / ndot;
	bc1.y = Vector3DF::CrossFunc( e0, p1 - v[VEXT+2] ).Dot(norm) / ndot;	
	bc1.z = 1-bc1.y-bc1.z;

	if ( fsgn(bc0.x) != fsgn(bc1.x) ) {		
		u = -bc0.x / (bc1.x-bc0.x);
		hit = p0 + (p1-p0) * u;
		t = (hit - rpos).Length();
		e = 0;
		return true;
	}
	if ( fsgn(bc0.y) != fsgn(bc1.y) ) {
		u = -bc0.y / (bc1.y-bc0.y);
		hit = p0 + (p1-p0) * u;
		t = (hit - rpos).Length();
		e = 1;
		return true;
	}
	if ( fsgn(bc0.z) != fsgn(bc1.z) ) {
		u = -bc0.z / (bc1.z-bc0.z);
		hit = p0 + (p1-p0) * u;
		t = (hit - rpos).Length();
		e = 2;
		return true;
	}
	return false;*/ 
}


void Sample::PrecomputePrisms ( float d )
{
	// Precompute prism geometry
	// - gives good acceleration. necessary for CPU version
	// - these are pushed into array that matches the enumeration of faces in the mesh

	Prism s;
	AttrV3* f;
	for (f = (AttrV3*) m_mesh->GetStart(BFACEV3); f <= (AttrV3*) m_mesh->GetEnd(BFACEV3); f++ ) {

		s.v[VBASE+0] = *m_mesh->GetVertPos( f->v1 );	s.v[VBASE+1] = *m_mesh->GetVertPos( f->v2 );	s.v[VBASE+2] = *m_mesh->GetVertPos( f->v3 );
		s.v[VNORM+0] = *m_mesh->GetVertNorm( f->v1 );	s.v[VNORM+1] = *m_mesh->GetVertNorm( f->v2 );	s.v[VNORM+2] = *m_mesh->GetVertNorm( f->v3 );		
		s.v[VUV+0]   = *m_mesh->GetVertTex( f->v1 );	s.v[VUV+1]   = *m_mesh->GetVertTex( f->v2 );	s.v[VUV+2]   = *m_mesh->GetVertTex( f->v3 );
		s.norm = (s.v[VBASE+2] - s.v[VBASE+1]).Cross ( s.v[VBASE+0] - s.v[VBASE+2] ); 
		s.norm.Normalize();
	
		// Extension of face outward along normal
		//--- using unmodified vertex normals
		//ve1 = vb1 + n1*d; ve2 = vb2 + n2*d; ve3 = vb3 + n3*d;  	
		//--- using corrected vertex normals to get uniform height		
		s.v[VEXT+0] = s.v[VBASE+0] + s.v[VNORM+0] * float(s.v[VNORM+0].Dot( s.norm ) * d); 
		s.v[VEXT+1] = s.v[VBASE+1] + s.v[VNORM+1] * float(s.v[VNORM+1].Dot( s.norm ) * d); 
		s.v[VEXT+2] = s.v[VBASE+2] + s.v[VNORM+2] * float(s.v[VNORM+2].Dot( s.norm ) * d);	

		// Prism bounding box
		ComputePrismBounds ( s.v, s.bmin, s.bmax );

		m_prisms.push_back ( s );
	}
}

#define EPS		0.0

void Sample::RaycastDisplacementMesh ( Vector3DF p, float r, float bump_depth, Vector3DF lgt, Image* img )
{
	float h, u, t, tx, diffuse;
	float rayhgt, rayhgt0;			// ray height
	float bmphgt, bmphgt0;			// displacement height
	AttrV3* f;	
	
	Prism* s;
	Vector3DF ve0, ve1, ve2;
	Vector3DF vd0, vd1, vd2;
	Vector3DF bmin, bmax;			// prism bounds
	Vector3DF hit, hit2, norm;		// hit points
	Vector2DF bc, bcl, best_bc;		// barycentric coordinates
	Vector3DF q[4], a, c, uv;	
	Vector3DF rpos, rdir, R, V;		// ray directions	
	float dpu, dpv;
	int face;
	int best_f, e, best_e, best_s;
	float best_t, ndot, spec;
	Vector3DF best_hit, lgtdir;		
	int xres = img->GetWidth();
	int yres = img->GetHeight();

	struct hitinfo_t {
		hitinfo_t (int f, int s, float tval, Vector3DF h) {face=f; side=s; t=tval; hit=h;}
		int	face;
		int side;
		float t;
		Vector3DF hit;
	};

	std::vector<hitinfo_t> candidates;

	lgt.Normalize();

	m_rand.seed ( m_samples );

	float d = m_bump_depth;
	float dt = 0.03;
	float doff = 0.0;
	bool used;

	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {

			// get camera ray
			rpos = m_cam->getPos();
			rdir = m_cam->inverseRay ( x, y, xres, yres );	
			rdir.Normalize();

			// intersect with each triangle in mesh
			t = 0;			
			best_t = 1.0e10;
			best_e = -1;
			best_f = -1;
			
			candidates.clear();

			// Collect candidates for hit
			//
			f = (AttrV3*) m_mesh->GetStart(BFACEV3);
			for (int i=0; i < m_mesh->GetNumFace3(); f++, i++ ) {

				s = &m_prisms[i];

				// intersect with facing triangle 
				if ( intersectRayTriangle ( rpos, rdir, s->v[VEXT+0], s->v[VEXT+1], s->v[VEXT+2], t, hit ) ) {				
					face = f-(AttrV3*) m_mesh->GetStart(BFACEV3);
					candidates.push_back ( hitinfo_t( face, -1, t, hit ) );									
				
				} else {					
					
					// intersect with triangular prism
					// with bounding box for fast rejection
					if ( intersectLineBox ( rpos, rdir, s->bmin, s->bmax, t ) ) {
						if ( intersectRayPrism ( rpos, rdir, s->v, t, hit, e) ) {
							face = f-(AttrV3*) m_mesh->GetStart(BFACEV3);
							candidates.push_back ( hitinfo_t( face, e, t, hit ) );					
						}
					}
				} 
			}

			// Check candidates for displacement hit
			//
			best_t = 1.0e10;
			best_f = -1;
			
			for (int i=0; i < candidates.size(); i++ ) {

				// Look for intersection using this triangle		
				face = candidates[i].face;

				//if ( face != 4 && face !=5  && face != 8 && face != 13 ) continue;

				e = candidates[i].side;
				t = candidates[i].t;
				hit = candidates[i].hit;

				// Get the triangular prism
				f = (AttrV3*) m_mesh->GetElem (BFACEV3, face );
				s = &m_prisms[ face ];				

				if ( e==-1 ) {
					// Best hit was the triangle itself
					// get barycentric coordinates - these coords ARE the projection of the hit point down to the base triangle.
					intersectRayTriangleUV ( rpos, rdir, s->v[VEXT+0], s->v[VEXT+1], s->v[VEXT+2], t, hit, bc.x, bc.y );
					
					// height of hit triangle is the total depth
					h = d;

				} else {
					// Best hit was a side face,
					// get barycentric coordinates here
					a = Vector3DF::CrossFunc( s->v[VEXT+2] - s->v[VEXT+1], s->v[VEXT+0] - s->v[VEXT+2] ); // a=norm. don't overwrite 'norm' here
					ndot = a.Dot(a);					
					bc.x = Vector3DF::CrossFunc( s->v[VEXT+2] - s->v[VEXT+1], hit - s->v[VEXT+1] ).Dot(a) / ndot;
					bc.y = Vector3DF::CrossFunc( s->v[VEXT+0] - s->v[VEXT+2], hit - s->v[VEXT+2] ).Dot(a) / ndot;

					// snap to edge
					if (bc.x < 0) bc.x = 0;
					if (bc.y < 0) bc.y = 0;
					if (bc.x > 1) bc.x = 1;
					if (bc.y > 1) bc.y = 1;

					// height of hit, computed as point-to-plane
					h = s->norm.Dot ( hit - s->v[VBASE+0] );

					//--- debugging. mark prism sides as red
					//best_f = -3;
					//continue;
				}

				// [optional] Jitter sampling for thin features
				if ( m_jitter_sample ) {
					t += m_rand.randF() * dt ;					
				}
				hit = rpos + rdir * t;	
				
				// Fractional change in ray height distance to triangle along normal
				// - the height change of the ray is linear with respect to the triangle normal								
				float df = s->norm.Dot ( rdir*-dt ) / d;				
				vd0 = (s->v[VBASE+0] - s->v[VEXT+0]) * df;
				vd1 = (s->v[VBASE+1] - s->v[VEXT+1]) * df;
				vd2 = (s->v[VBASE+2] - s->v[VEXT+2]) * df;	

				// Start ray at initial height
				// when h=d, the adjustment term is 0 giving maximal triangle
				ve0 = s->v[VEXT+0] + vd0*(1-(h/d));
				ve1 = s->v[VEXT+1] + vd1*(1-(h/d));
				ve2 = s->v[VEXT+2] + vd2*(1-(h/d));

				// Start ray at inital height 
				rayhgt = h;  

				// Inital displacement height
				uv = s->v[VUV+0] * (bc.x) + s->v[VUV+1] * (bc.y) + s->v[VUV+2] * (1-bc.x-bc.y);
				bmphgt = doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;				

				// March ray until it hits or leaves prism
				//
				for (; rayhgt >= bmphgt && rayhgt <= d && bc.x >= 0 && bc.x <= 1 && bc.y >= 0 && bc.y <= 1;) {

					// march along ray
					hit += rdir * dt;					
					
					// barycentric coords on triangle at given height
					ve0 += vd0;
					ve1 += vd1;
					ve2 += vd2;
					norm = Vector3DF::CrossFunc( ve2-ve1, ve0-ve2 );	// do not normalize here!
					ndot = norm.Dot(norm);
					bcl = bc;
					bc.x = Vector3DF::CrossFunc( ve2-ve1, hit - ve1 ).Dot(norm) / ndot;
					bc.y = Vector3DF::CrossFunc( ve0-ve2, hit - ve2 ).Dot(norm) / ndot;	

					// point on base triangle
					p = s->v[VBASE+0] * (bc.x) + s->v[VBASE+1] * (bc.y) + s->v[VBASE+2] * (1-bc.x-bc.y);
					
					// current ray point height above base triangle
					a = (hit - p);
					if ( a.Dot(norm) > 0 )
						rayhgt = a.Length();
					else 
						rayhgt = 0;

					// sample bump map to get displacement height
					uv = s->v[VUV+0] * (bc.x) + s->v[VUV+1] * (bc.y) + s->v[VUV+2] * (1-bc.x-bc.y);
					bmphgt = doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;
				}

				// Check for displaced surface hit..
				if ( rayhgt <= bmphgt ) {

					// Linear interpolation to find surface
					// get current sample
					bmphgt = doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;					
					// get previous sample
					p = s->v[VBASE+0] * (bcl.x) + s->v[VBASE+1] * (bcl.y) + s->v[VBASE+2] * (1-bcl.x-bcl.y);								
					rayhgt0 = ( (hit-rdir*dt) - p ).Length();					
					uv = s->v[VUV+0] * (bcl.x) + s->v[VUV+1] * (bcl.y) + s->v[VUV+2] * (1-bcl.x-bcl.y);
					bmphgt0 =  doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;
					// interpolate between previous and current sample
					u = (rayhgt0-bmphgt0) / ((rayhgt0-bmphgt0)+(bmphgt-rayhgt));
					u = (u<0) ? 0 : (u>1) ? 1 : u;
					// adjust barycentric coords and t
					bc = bcl + (bc-bcl) * u; 										
					t = t - dt + u*dt;

					if ( t < best_t ) {
						best_t = t;
						best_hit = rpos + rdir * t;
						best_bc = bc;
						best_f = face;
						best_s = candidates[i].side;
					}									
				} 
				else {
					//best_f = -2;
				}
			} 		

			if ( best_f >= 0 ) {				

				bc = best_bc;

				// Get the best hit prism 
				f = (AttrV3*) m_mesh->GetElem (BFACEV3, best_f );
				s = &m_prisms[ best_f ];	

				// Interpolate normal by neighbor sampling 
				dpu = 1.0 / (m_bump_img->GetWidth()-1);
				//dpv = 1.0 / (m_bump_img->GetHeight()-1);

				// Compute 3D displaced points and gradient points					
				norm = s->v[VNORM+0] * (bc.x) + s->v[VNORM+1] * (bc.y) + s->v[VNORM+2] * (1-bc.x-bc.y);     // interpolated normal
				norm.Normalize();
					
				// central sample
				uv =   s->v[VUV+0]   * (bc.x) + s->v[VUV+1]   * (bc.y) + s->v[VUV+2]   * (1-bc.x-bc.y);			
				q[0] = s->v[VBASE+0] * (bc.x) + s->v[VBASE+1] * (bc.y) + s->v[VBASE+2] * (1-bc.x-bc.y);
				q[0] += norm * (doff + m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x * d); 

				// sample along barycentric u
				uv =   s->v[VUV+0]   * (bc.x+dpu) + s->v[VUV+1]   * (bc.y) + s->v[VUV+2]   * (1-(bc.x+dpu)-bc.y);
				q[1] = s->v[VBASE+0] * (bc.x+dpu) + s->v[VBASE+1] * (bc.y) + s->v[VBASE+2] * (1-(bc.x+dpu)-bc.y);
				q[1] += norm * (doff + m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x * d); 

				// sample along barycentric v					
				uv =   s->v[VUV+0]   * (bc.x) + s->v[VUV+1]   * (bc.y+dpu) + s->v[VUV+2]   * (1-bc.x-(bc.y+dpu));
				q[2] = s->v[VBASE+0] * (bc.x) + s->v[VBASE+1] * (bc.y+dpu) + s->v[VBASE+2] * (1-bc.x-(bc.y+dpu));
				q[2] += norm * (doff + m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x * d); 

				// compute normal as cross product of surface displacements
				q[1] -= q[0];	q[1].Normalize();
				q[2] -= q[0];	q[2].Normalize();
				norm = q[2].Cross (q[1]); norm.Normalize();					

				tx = m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x;
				
				// Diffuse shading
				lgtdir = q[0] - lgt; lgtdir.Normalize();					
				diffuse = 0.2 + 0.8 * std::max(0.0, norm.Dot( lgtdir ));
				R = norm * float(2 * norm.Dot(lgtdir)) - lgtdir; R.Normalize();
				V = q[0] - m_cam->getPos(); V.Normalize();
				spec = 0.3 * pow ( R.Dot( V ), 20 );

				c = m_color_img->GetPixelFilteredUV ( uv.x, uv.y );
				c = c * diffuse + Vector3DF(spec,spec,spec);
				c *= 255.0;
				if (c.x > 255 || c.y > 255 || c.z > 255) c = Vector3DF(255,255,255);
					
				if ( m_jitter_sample )
					c = (img->GetPixel(x,y)*float(m_samples-1) + c)/m_samples;					

				//-- debug prism sides
				//if (best_s !=-1 ) c += Vector3DF( (best_s==0), (best_s==1), (best_s==2) ) *20.0f;

				img->SetPixel ( x, y, c.x, c.y, c.z, 255.0 );

			} else {
				
				if (best_f==-2) img->SetPixel ( x, y, 0, 255, 255, 255.0 );
				if (best_f==-3) img->SetPixel ( x, y, 255, 0, 0, 255.0 );				

			}

		}	// x pixels
	}		// y pixels 


	m_samples++;

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_GLTEX );		
}


void Sample::DrawMesh ( float bump_depth )
{
	float cu, su, cu1, su1;
	float cv, sv, cv1, sv1;
	float pu, pv, dpu, dpv;
	Vector3DF a, b, c, d, n;	
	Vector4DF clr(1,1,1,1);	
	
	/* 
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

			drawFace3D(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x, d.y, d.z, n.x, n.y, n.z, clr.x, clr.y, clr.z, clr.w);
			faces++;
		}
	}
	dbgprintf ( "# faces: %d\n", faces ); */ 
}


void Sample::display()
{
	clearGL();

	Vector3DF lgt (30, 70, 20);

	// Displacement raycasting
	if ( m_samples==1 || m_jitter_sample) {
		if (m_displace)
			RaycastDisplacementMesh ( Vector3DF(0,0,0), 1.0, m_bump_depth, lgt, m_out_img );
		else
			RaycastMesh ( Vector3DF(0,0,0), 1.0, m_bump_depth, lgt, m_out_img );
	}			

	// Draw grid
	start3D(m_cam);
	setLight(S3D, lgt.x, lgt.y, lgt.z);	
		DrawGrid();
	end3D();
	draw3D();

	// Draw raycast image
	start2D();
	setview2D(getWidth(), getHeight());		
	drawImg ( m_out_img->getGLID(), 0, 0, getWidth(), getHeight(), 1,1,1,1 );		// draw raycast image 	
	end2D();	
	draw2D();										// complete 2D rendering to OpenGL

	// Draw prisms in 3D	
	AttrV3* f;
	Vector3DF b0,b1,b2;
	Vector3DF e0,e1,e2;
	Prism* s;
	Vector4DF clr(1,1,1,.5);	
	Vector3DF N, V;
	V = m_cam->getDir ();
	V.Normalize();
	
	if ( m_draw_prisms ) {

		start3D(m_cam);
		for (int i = 0; i < m_prisms.size(); i++)  {
			s = &m_prisms[i];
			b0 = s->v[VBASE+0]; b1 = s->v[VBASE+1]; b2 = s->v[VBASE+2];
			e0 = s->v[VEXT+0];  e1 = s->v[VEXT+1];  e2 = s->v[VEXT+2];			

			if ( s->norm.Dot ( V ) < 0 ) {
				//drawLine3D ( b0, b1, clr ); drawLine3D ( b1, b2, clr );	drawLine3D ( b2, b0, clr );
				drawLine3D ( b0, e0, clr ); drawLine3D ( b1, e1, clr );	drawLine3D ( b2, e2, clr );			
				drawLine3D ( e0, e1, clr ); drawLine3D ( e1, e2, clr );	drawLine3D ( e2, e0, clr );			
			}
		} 
		end3D();		
		draw3D();
	}		

	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::UpdateCamera()
{
	// clear the output image
	memset ( m_out_img->GetData(), 0, m_out_img->GetSize() );
	memset ( m_hit_img->GetData(), 127, m_hit_img->GetSize() );
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
		UpdateCamera();
	} break;

	case AppEnum::BUTTON_RIGHT: {

		// Adjust camera orbit 
		Vector3DF angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
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
	UpdateCamera();
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action==AppEnum::BUTTON_RELEASE) return;

	switch (keycode) {
	case 'd': 
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




