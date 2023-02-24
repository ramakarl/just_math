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

	void		PrecomputePrisms ( float d );

	void		RaycastMesh ( float bump_depth, Vector3DF lgt, Image* img );
	void		RaycastDisplacementMesh ( float bump_depth, Vector3DF lgt, Image* img );	
	void		RaycastDebug ( int x, int y, Image* img );	

	void		DrawGrid ();	
	void		UpdateCamera();

	void		VisDot ( Vector3DF p, float r, Vector3DF clr );
	void		VisLine ( Vector3DF a, Vector3DF b, Vector3DF clr );

	Camera3D*	m_cam[2];			// camera
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

	std::vector<Vis> m_debugvis;	// debug visualizations
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
	m_displace = true;
	m_jitter_sample = false;
	m_draw_prisms = false;

	m_rand.seed ( 123 );

	// create a camera
	m_cam[0] = new Camera3D;								
	m_cam[0]->setOrbit ( 30, 40, 0, Vector3DF(0,0,0), 8, 1 );
	
	m_cam[1] = new Camera3D;								
	m_cam[1]->setOrbit ( 30, 40, 0, Vector3DF(0,0,0), 8, 1 );

	m_currcam = 0;

	int res = 1024;

	// set bump depth
	m_bump_depth = 0.20;

	// load displacement map 
	std::string fpath;	
	//getFileLocation ( "stones_displace.png", fpath );
	getFileLocation ( "rocks01_displace.png", fpath );
	//getFileLocation ( "rocks002_displace.png", fpath );
	//getFileLocation ( "rocks07_displace.png", fpath );
	//getFileLocation ( "gravel02_displace.png", fpath );
	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_bump_img = new Image;
	if ( !m_bump_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}

	//getFileLocation ( "stones_color.png", fpath );
	getFileLocation ( "rocks01_displace.png", fpath );
	//getFileLocation ( "rocks002_color.png", fpath );
	//getFileLocation ( "rocks07_color.png", fpath );
	//getFileLocation ( "gravel02_color.png", fpath );
	dbgprintf ("Loading: %s\n", fpath.c_str() );
	m_color_img = new Image;
	if ( !m_color_img->Load ( fpath ) ) {
		dbgprintf ( "ERROR: Unable to open %s.\n", fpath.c_str() );
		exit(-2);
	}

	// load mesh
	getFileLocation ( "surface.obj", fpath );
	//getFileLocation ( "asteroid.obj", fpath );
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

void Sample::RaycastMesh ( float bump_depth, Vector3DF lgt, Image* img )
{
	float t, tx, diffuse, h;
	AttrV3* f;
	Vector3DF v1,v2,v3;
	Vector3DF n1,n2,n3;
	Vector2DF uv1, uv2, uv3, uv;
	Vector3DF a,b,c,p;
	Vector3DF hit, norm;	
	Vector3DF rpos, rdir;
	Vector4DD bc;
	int best_f;
	float best_t;
	Vector3DF best_hit;
	int raycost;
	int i;
	int xres = img->GetWidth();
	int yres = img->GetHeight();

	// maximal shell - expand sphere by bump depth	
	
	lgt.Normalize();

	m_rand.seed ( m_samples );
	
	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {

			// get camera ray
			rpos = m_cam[0]->getPos();
			rdir = m_cam[0]->inverseRay ( x, y, xres, yres );	
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
				norm = n1 * float(bc.x) + n2 * float(bc.y) + n3 * float(1-bc.x-bc.y);
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
	if ( intersectRayTriangle ( rpos, rdir, v[VEXT+0],  v[VEXT+1],  v[VEXT+2], t, hit )) {e=-1; return true; }
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+0], v[VEXT+1],  v[VEXT+0], t, hit )) {e=0; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+0], v[VBASE+1], v[VEXT+1], t, hit )) {e=0; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+1], v[VEXT+2],  v[VEXT+1], t, hit )) {e=1; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+1], v[VBASE+2], v[VEXT+2], t, hit )) {e=1; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+2], v[VEXT+0],  v[VEXT+2], t, hit )) {e=2; return true;}
	if ( intersectRayTriangle ( rpos, rdir, v[VBASE+2], v[VBASE+0], v[VEXT+0], t, hit )) {e=2; return true;}	
	return false;
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
		s.v[VEXT+0] = s.v[VBASE+0] + s.v[VNORM+0] * d;
		s.v[VEXT+1] = s.v[VBASE+1] + s.v[VNORM+1] * d;
		s.v[VEXT+2] = s.v[VBASE+2] + s.v[VNORM+2] * d;
		
		//--- using corrected vertex normals to get uniform height		
		/*s.v[VEXT+0] = s.v[VBASE+0] + s.v[VNORM+0] * float(s.v[VNORM+0].Dot( s.norm ) * d); 
		s.v[VEXT+1] = s.v[VBASE+1] + s.v[VNORM+1] * float(s.v[VNORM+1].Dot( s.norm ) * d); 
		s.v[VEXT+2] = s.v[VBASE+2] + s.v[VNORM+2] * float(s.v[VNORM+2].Dot( s.norm ) * d);	*/

		// Prism bounding box
		ComputePrismBounds ( s.v, s.bmin, s.bmax );

		m_prisms.push_back ( s );
	}
}


void Sample::RaycastDisplacementMesh ( float bump_depth, Vector3DF lgt, Image* img )
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
	Vector3DF q[4], a, c, uv, p;
	Vector3DF rpos, rdir, R, V;		// ray directions	
	Vector4DD bc, bcl, best_bc;		// barycentric coordinates
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
			rpos = m_cam[0]->getPos();
			rdir = m_cam[0]->inverseRay ( x, y, xres, yres );	
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

				// fast bounding box check..
				if ( intersectLineBox ( rpos, rdir, s->bmin, s->bmax, t ) ) {

					// intersect with triangular prism
					if ( intersectRayPrism ( rpos, rdir, s->v, t, hit, e) ) {
						face = f-(AttrV3*) m_mesh->GetStart(BFACEV3);
						candidates.push_back ( hitinfo_t( face, e, t, hit ) );					
					} 
				}
			}

			if ( candidates.size() == 0 )
				continue;

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
					
				} else {
					// Best hit was a side face..				
					// Get barycentric coordinates
					a = Vector3DF::CrossFunc( s->v[VEXT+2] - s->v[VEXT+1], s->v[VEXT+0] - s->v[VEXT+2] ); // a=norm. don't overwrite 'norm' here
					ndot = a.Dot(a);					
					bc.x = Vector3DF::CrossFunc( s->v[VEXT+2] - s->v[VEXT+1], hit - s->v[VEXT+1] ).Dot(a) / ndot;
					bc.y = Vector3DF::CrossFunc( s->v[VEXT+0] - s->v[VEXT+2], hit - s->v[VEXT+2] ).Dot(a) / ndot;
					// snap to edge
					bc.x = (bc.x < 0) ? 0 : bc.x;
					bc.y = (bc.y < 0) ? 0 : bc.y;			
				}
				bc.z = 1-bc.x-bc.y;
				bc.z = (bc.z < 0) ? 0 : bc.z;

				// [optional] Jitter sampling for thin features
				if ( m_jitter_sample ) {							
					hit += rdir * m_rand.randF() * dt; 		
				}

				// Inital ray height
				p = s->v[VBASE+0] * float(bc.x) + s->v[VBASE+1] * float(bc.y) + s->v[VBASE+2] * float(bc.z);		
				rayhgt = (hit - p).Length();
				
				// Fractional change in ray height distance to triangle along normal
				// - the height change of the ray is linear with respect to the triangle normal
				float df = s->norm.Dot ( rdir*-dt ) / d;				
				vd0 = (s->v[VBASE+0] - s->v[VEXT+0]) * df;
				vd1 = (s->v[VBASE+1] - s->v[VEXT+1]) * df;
				vd2 = (s->v[VBASE+2] - s->v[VEXT+2]) * df;	

				// Compute inital scanning triangle				
				df = (1.0 - (rayhgt/d)) / df;
				ve0 = s->v[VEXT+0] + vd0 * df;
				ve1 = s->v[VEXT+1] + vd1 * df;
				ve2 = s->v[VEXT+2] + vd2 * df;
		
				// Project hit point to edge
				/*if (e != -1) {
					switch (e) {
					case 0: hit = projectPointLine( hit, ve0, ve1 ); bc.z = 0; bc.x = 1-bc.y-bc.z;	break;
					case 1: hit = projectPointLine( hit, ve1, ve2 ); bc.x = 0; bc.z = 1-bc.x-bc.y;	break;
					case 2: hit = projectPointLine( hit, ve2, ve0 ); bc.y = 0; bc.z = 1-bc.x-bc.y;	break;
					}
				}*/			

				// Inital displacement height
				uv = s->v[VUV+0] * float(bc.x) + s->v[VUV+1] * float(bc.y) + s->v[VUV+2] * float(1-bc.x-bc.y);
				bmphgt = doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;				

				// March ray until it hits or leaves prism
				//
				for (; rayhgt >= bmphgt && rayhgt <= d && bc.x >= 0 && bc.x <= 1 && bc.y >= 0 && bc.y <= 1 && bc.z >= 0 && bc.z <= 1;) {

					// march along ray
					hit += rdir * dt;					
					
					// advance scanning triangle 
					ve0 += vd0;
					ve1 += vd1;
					ve2 += vd2;

					// barycentric coords on triangle
					a = Vector3DF::CrossFunc( ve2-ve1, ve0-ve2 );	// do not normalize here!
					ndot = a.Dot(a);
					bcl = bc;
					bc.x = Vector3DF::CrossFunc( ve2-ve1, hit - ve1 ).Dot(a) / ndot;
					bc.y = Vector3DF::CrossFunc( ve0-ve2, hit - ve2 ).Dot(a) / ndot;	 
					bc.z = 1 - bc.x - bc.y;

					// point on base triangle
					p = s->v[VBASE+0] * float(bc.x) + s->v[VBASE+1] * float(bc.y) + s->v[VBASE+2] * float(bc.z);	

					// current ray point height above base triangle					
					a = hit - p;
					if ( a.Dot( s->norm ) > 0 )
						rayhgt = a.Length();
					else 
						rayhgt = 0;

					// sample bump map to get displacement height
					uv = s->v[VUV+0] * float(bc.x) + s->v[VUV+1] * float(bc.y) + s->v[VUV+2] * float(bc.z);
					bmphgt = doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;
				}

				// Check for displaced surface hit..
				if ( rayhgt <= bmphgt ) {

					// Linear interpolation to find surface
					// we have current sample already - below surface					
					// get previous sample - above surface
					p = s->v[VBASE+0] * float(bcl.x) + s->v[VBASE+1] * float(bcl.y) + s->v[VBASE+2] * float(1-bcl.x-bcl.y);								
					rayhgt0 = ( (hit-rdir*dt) - p ).Length();					
					uv = s->v[VUV+0] * float(bcl.x) + s->v[VUV+1] * float(bcl.y) + s->v[VUV+2] * float(1-bcl.x-bcl.y);
					bmphgt0 =  doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;
					// interpolate between previous and current sample
					u = (rayhgt0-bmphgt0) / ((rayhgt0-bmphgt0)+(bmphgt-rayhgt));
					u = (u<0) ? 0 : (u>1) ? 1 : u;
					// adjust barycentric coords and t
					bc = bcl + (bc-bcl) * u; 										
					
					// set t (not really used, the bc is used below)
					t = (hit - rpos).Length() - dt + u*dt;

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
			
			bc = best_bc;

			if ( best_f >= 0 ) {				

				// Get the best hit prism 
				f = (AttrV3*) m_mesh->GetElem (BFACEV3, best_f );
				s = &m_prisms[ best_f ];	

				// Interpolate normal by neighbor sampling 
				dpu = 1.0 / (m_bump_img->GetWidth()-1);
				//dpv = 1.0 / (m_bump_img->GetHeight()-1);

				// Compute 3D displaced points and gradient points					
				bool reject = false;
				norm = s->v[VNORM+0] * float(bc.x) + s->v[VNORM+1] * float(bc.y) + s->v[VNORM+2] * float(1-bc.x-bc.y);     // interpolated normal
				norm.Normalize();
					
				
				// sample along barycentric u
				uv =   s->v[VUV+0]   * float(bc.x+dpu) + s->v[VUV+1] * float(bc.y) + s->v[VUV+2]   * float(1-(bc.x+dpu)-bc.y);
				if ( uv.x >= 0 && uv.x <=1 && uv.y >=0 && uv.y <=1 ) {
					q[1] = s->v[VBASE+0] * float(bc.x+dpu) + s->v[VBASE+1] * float(bc.y) + s->v[VBASE+2] * float(1-(bc.x+dpu)-bc.y);
					q[1] += norm * (doff + m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x * d); 
				} else {
					reject = true;
				}

				// sample along barycentric v					
				uv =   s->v[VUV+0]  * float(bc.x) + s->v[VUV+1]  * float(bc.y+dpu) + s->v[VUV+2]  * float(1-bc.x-(bc.y+dpu));
				if ( uv.x >= 0 && uv.x <=1 && uv.y >=0 && uv.y <=1 ) {
					q[2] = s->v[VBASE+0] * float(bc.x) + s->v[VBASE+1] * float(bc.y+dpu) + s->v[VBASE+2] * float(1-bc.x-(bc.y+dpu));
					q[2] += norm * (doff + m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x * d); 
				} else {
					reject = true;
				}

				// central sample
				uv =   s->v[VUV+0]  * float(bc.x) + s->v[VUV+1]  * float(bc.y) + s->v[VUV+2] * float(1-bc.x-bc.y);			
				if ( uv.x >= 0 && uv.x <=1 && uv.y >=0 && uv.y <=1 ) {
					q[0] = s->v[VBASE+0] * float(bc.x) + s->v[VBASE+1] * float(bc.y) + s->v[VBASE+2] * float(1-bc.x-bc.y);
					q[0] += norm * (doff + m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x * d); 					
					// get color texture sample here
					c = m_color_img->GetPixelFilteredUV ( uv.x, uv.y );
				} else {
					reject = true;
				}

				// check if rejected.. 
				if ( !reject ) {
					// Compute normal as cross product of surface displacements
					q[1] -= q[0];	q[1].Normalize();
					q[2] -= q[0];	q[2].Normalize();
					norm = q[2].Cross (q[1]); norm.Normalize();					
				
					// Diffuse shading
					lgtdir = lgt - q[0]; lgtdir.Normalize();					
					diffuse = 0.2 + 0.8 * std::max(0.0, norm.Dot( lgtdir*-1.f ));
					R = norm * float(2 * norm.Dot(lgtdir*-1.f)) + lgtdir; R.Normalize();
					V = q[0] - m_cam[0]->getPos(); V.Normalize();
					spec = 0.3 * pow ( R.Dot( V ), 20 );

					c = c * diffuse + Vector3DF(spec,spec,spec);
					c *= 255.0;
					if (c.x > 255 || c.y > 255 || c.z > 255) c = Vector3DF(255,255,255);

				} else {
					c = 0;
				}

				if ( m_jitter_sample )
					c = (img->GetPixel(x,y)*float(m_samples-1) + c)/m_samples;					

				//-- debug prism sides
				//if (best_s !=-1 ) c += Vector3DF( (best_s==0), (best_s==1), (best_s==2) ) *20.0f;

				// Write pixel
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



void Sample::RaycastDebug ( int x, int y, Image* img )
{
	m_debugvis.clear ();

	float h, u, t, tx, diffuse;
	float rayhgt, rayhgt0;			// ray height
	float bmphgt, bmphgt0;			// displacement height
	AttrV3* f;	
	
	Prism* s;
	Vector3DF ve0, ve1, ve2;
	Vector3DF vd0, vd1, vd2;
	Vector3DF bmin, bmax;			// prism bounds
	Vector3DF hit, hit2, norm;		// hit points	
	Vector3DF q[4], a, c, uv, p;
	Vector3DF rpos, rdir, R, V;		// ray directions	
	Vector4DD bc, bcl, best_bc;		// barycentric coordinates
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

	m_rand.seed ( m_samples );

	float d = m_bump_depth;
	float dt = 0.01;
	float doff = 0.0;
	bool used;

	// get camera ray at x,y

	int xp, yp;
	xp = x * xres / getWidth();
	yp = y * yres / getHeight();

	rpos = m_cam[0]->getPos();
	rdir = m_cam[0]->inverseRay ( xp, yp, xres, yres );	
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

		// fast bounding box check..
		if ( intersectLineBox ( rpos, rdir, s->bmin, s->bmax, t ) ) {

			// intersect with triangular prism
			if ( intersectRayPrism ( rpos, rdir, s->v, t, hit, e) ) {
				face = f-(AttrV3*) m_mesh->GetStart(BFACEV3);
				candidates.push_back ( hitinfo_t( face, e, t, hit ) );					
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
					
		} else {
			// Best hit was a side face..
			// Project hit point to edge
			/*switch (e) {
			case 0: hit = projectPointLine( hit, s->v[VEXT+0], s->v[VEXT+1] ); break;
			case 1: hit = projectPointLine( hit, s->v[VEXT+1], s->v[VEXT+2] ); break;
			case 2: hit = projectPointLine( hit, s->v[VEXT+2], s->v[VEXT+0] ); break;
			}*/					
			// Get barycentric coordinates here
			a = Vector3DF::CrossFunc( s->v[VEXT+2] - s->v[VEXT+1], s->v[VEXT+0] - s->v[VEXT+2] ); // a=norm. don't overwrite 'norm' here
			ndot = a.Dot(a);					
			bc.x = Vector3DF::CrossFunc( s->v[VEXT+2] - s->v[VEXT+1], hit - s->v[VEXT+1] ).Dot(a) / ndot;
			bc.y = Vector3DF::CrossFunc( s->v[VEXT+0] - s->v[VEXT+2], hit - s->v[VEXT+2] ).Dot(a) / ndot;

			// snap to edge - eliminates edge artifacts
			bc.x = (bc.x < 0) ? 0 : bc.x;
			bc.y = (bc.y < 0) ? 0 : bc.y;			
		}
		bc.z = 1-bc.x-bc.y;
		bc.z = (bc.z < 0) ? 0 : bc.z;

		// inital ray height
		p = s->v[VBASE+0] * float(bc.x) + s->v[VBASE+1] * float(bc.y) + s->v[VBASE+2] * float(bc.z);
		rayhgt = (hit - p).Length();
		dbgprintf ( "%f (%f%%)\n", rayhgt, rayhgt/d );

		// Fractional change in ray height distance to triangle along `al
		// - the height change of the ray is linear with respect to the triangle normal								
		float df = s->norm.Dot ( rdir*-dt ) / d;				
		vd0 = (s->v[VBASE+0] - s->v[VEXT+0]) * df;
		vd1 = (s->v[VBASE+1] - s->v[VEXT+1]) * df;
		vd2 = (s->v[VBASE+2] - s->v[VEXT+2]) * df;	

		// Start ray at initial height
		// when h=d, the adjustment term is 0 giving maximal triangle
		df = (1.0 - (rayhgt/d)) / df;
		ve0 = s->v[VEXT+0] + vd0 * df;
		ve1 = s->v[VEXT+1] + vd1 * df;
		ve2 = s->v[VEXT+2] + vd2 * df;

		VisDot ( hit, 1.1, Vector3DF(1,1,1) );
		VisDot ( hit, 1.2, Vector3DF(1,1,1) );

		// Project hit point to edge
		switch (e) {
		case 0: hit = projectPointLine( hit, ve0, ve1 ); break;
		case 1: hit = projectPointLine( hit, ve1, ve2 ); break;
		case 2: hit = projectPointLine( hit, ve2, ve0 ); break;
		}		
		
		// [optional] Jitter sampling for thin features
		if ( m_jitter_sample ) {
			t += m_rand.randF() * dt ;					
			hit += rdir * dt; 		
		}		

		// Inital displacement height
		uv = s->v[VUV+0] * float(bc.x) + s->v[VUV+1] * float(bc.y) + s->v[VUV+2] * float(1-bc.x-bc.y);
		bmphgt = doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;				

		VisLine ( s->v[VBASE+0], s->v[VBASE+1], Vector3DF(0,.5,0) );
		VisLine ( s->v[VBASE+1], s->v[VBASE+2], Vector3DF(0,.5,0) );
		VisLine ( s->v[VBASE+2], s->v[VBASE+0], Vector3DF(0,.5,0) );
		VisLine ( s->v[VEXT+0], s->v[VEXT+1], Vector3DF(.5,.5,.5) );
		VisLine ( s->v[VEXT+1], s->v[VEXT+2], Vector3DF(.5,.5,.5) );
		VisLine ( s->v[VEXT+2], s->v[VEXT+0], Vector3DF(.5,.5,.5) );
		VisLine ( rpos, rpos+rdir*10, Vector3DF(1,1,1) );			

		if (i==0) {			
			VisDot ( hit, 1, Vector3DF(1,0,0) );
			VisLine ( ve0, ve1, Vector3DF(1,0,0) );
			VisLine ( ve1, ve2, Vector3DF(1,0,0) );
			VisLine ( ve2, ve0, Vector3DF(1,0,0) );
		} else {			
			VisDot ( hit, 1, Vector3DF(1,0,1) );
			VisLine ( ve0, ve1, Vector3DF(1,0,1) );
			VisLine ( ve1, ve2, Vector3DF(1,0,1) );
			VisLine ( ve2, ve0, Vector3DF(1,0,1) );
		}

		// March ray until it hits or leaves prism
		//
		for (; rayhgt >= bmphgt && rayhgt <= d && bc.x >= 0 && bc.x <= 1 && bc.y >= 0 && bc.y <= 1 && bc.z >= 0 && bc.z <= 1;) {

			// march along ray
			hit += rdir * dt;					
					
			// advance scanning triangle 
			ve0 += vd0;
			ve1 += vd1;
			ve2 += vd2;

			// barycentric coords on triangle
			a = Vector3DF::CrossFunc( ve2-ve1, ve0-ve2 );	// do not normalize here!
			ndot = a.Dot(a);
			bcl = bc;
			bc.x = Vector3DF::CrossFunc( ve2-ve1, hit - ve1 ).Dot(a) / ndot;
			bc.y = Vector3DF::CrossFunc( ve0-ve2, hit - ve2 ).Dot(a) / ndot;
			bc.z = 1 - bc.x - bc.y;

			// point on base triangle
			p = s->v[VBASE+0] * float(bc.x) + s->v[VBASE+1] * float(bc.y) + s->v[VBASE+2] * float(bc.z);
					
			// current ray point height above base triangle	
			a = hit - p;
			if ( a.Dot( s->norm ) > 0 )
				rayhgt = a.Length();
			else 
				rayhgt = 0;

			// sample bump map to get displacement height
			uv = s->v[VUV+0] * float(bc.x) + s->v[VUV+1] * float(bc.y) + s->v[VUV+2] * float(bc.z);
			bmphgt = doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;

			//---------- visualization debugging
			if (i==0) {			
				VisDot ( hit, .8, Vector3DF(1,.5,0) );		// visualize sample points (orange)			
				VisDot ( p, .8, Vector3DF(0,1,0) );			// visualize base points (green)
				a.Normalize();
				a = p + a * bmphgt;
				VisDot ( a, .8, Vector3DF(1,1,0) );			// visualize displace points (yellow) 
			} else {
				VisDot ( hit, .8, Vector3DF(1,.5,1) );			
				VisDot ( p, .8, Vector3DF(0,0,1) );		
				a.Normalize();
				a = p + a * bmphgt;
				VisDot ( a, .8, Vector3DF(0,1,1) );			
			}

		}

		// Check for displaced surface hit..
		if ( rayhgt <= bmphgt ) {

			// Linear interpolation to find surface
			// we have current sample already - below surface					
			// get previous sample - above surface
			p = s->v[VBASE+0] * float(bcl.x) + s->v[VBASE+1] * float(bcl.y) + s->v[VBASE+2] * float(1-bcl.x-bcl.y);								
			rayhgt0 = ( (hit-rdir*dt) - p ).Length();					
			uv = s->v[VUV+0] * float(bcl.x) + s->v[VUV+1] * float(bcl.y) + s->v[VUV+2] * float(1-bcl.x-bcl.y);
			bmphgt0 =  doff + m_bump_img->GetPixelUV ( uv.x, uv.y ).x * d;
			// interpolate between previous and current sample
			u = (rayhgt0-bmphgt0) / ((rayhgt0-bmphgt0)+(bmphgt-rayhgt));
			u = (u<0) ? 0 : (u>1) ? 1 : u;
			// adjust barycentric coords and t
			bc = bcl + (bc-bcl) * u; 										
					
			// set t (not really used, the bc is used below)
			t = (hit - rpos).Length() - dt + u*dt;
			hit = rpos + rdir * t;

			if ( t < best_t ) {
				best_t = t;
				best_hit = hit;
				best_bc = bc;
				best_f = face;
				best_s = candidates[i].side;

				if (i==0) {			
					VisDot ( hit, .4, Vector3DF(1,0,0) );
					VisDot ( hit, .6, Vector3DF(1,0,0) );
				} else {			
					VisDot ( hit, .4, Vector3DF(1,0,1) );
					VisDot ( hit, .6, Vector3DF(1,0,1) );
				}
			}									
		} 
		else {
			//best_f = -2;
		}
	} 	
			
	bc = best_bc;

	if ( best_f >= 0 ) {				

		// Get the best hit prism 
		f = (AttrV3*) m_mesh->GetElem (BFACEV3, best_f );
		s = &m_prisms[ best_f ];	

		// central sample
		uv =   s->v[VUV+0]   * float(bc.x) + s->v[VUV+1]   * float(bc.y) + s->v[VUV+2]   * float(1-bc.x-bc.y);			
		if ( uv.x >= 0 && uv.x <=1 && uv.y >=0 && uv.y <=1 ) {
			q[0] = s->v[VBASE+0] * float(bc.x) + s->v[VBASE+1] * float(bc.y) + s->v[VBASE+2] * float(1-bc.x-bc.y);
			q[0] += norm * (doff + m_bump_img->GetPixelFilteredUV ( uv.x, uv.y ).x * d); 					
		} 

	} 
}

#include <time.h>

void Sample::display()
{
	AttrV3* f;
	Vector3DF b0,b1,b2;
	Vector3DF e0,e1,e2;
	Prism* s;
	Vector4DF clr(1,1,1,.5);	
	Vector3DF N, V;

	clearGL();

	Vector3DF lgt (30, 70, 20);

	// Displacement raycasting
	clock_t t1,t2;
	t1 = clock();
	if ( m_samples==1 || m_jitter_sample) {
		if (m_displace) 
			RaycastDisplacementMesh ( m_bump_depth, lgt, m_out_img );
		else
			RaycastMesh ( m_bump_depth, lgt, m_out_img );
	}			
	t2 = clock();

	//dbgprintf ( "Time: %f msec\n", float(t2-t1)/float(CLOCKS_PER_SEC/1000.0f) );

	// Draw grid
	start3D(m_cam[0]);
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

	V = m_cam[0]->getDir ();
	V.Normalize();
	

	// Draw debug visualization
	start3D(m_cam[1]);
	setview2D(getWidth()/2, getHeight()/2);
	DrawGrid ();
	for (int i=0; i < m_debugvis.size(); i++) {
		if ( m_debugvis[i].type == 'p' ) {
			b0 = m_debugvis[i].a; b1 = m_debugvis[i].b;
			drawCircle3D ( b0, b0+V, 0.005 * b1.x, m_debugvis[i].clr );
		}
	}
	for (int i=0; i < m_debugvis.size(); i++) {
		if ( m_debugvis[i].type == 'l' ) {
			b0 = m_debugvis[i].a; b1 = m_debugvis[i].b;
			drawLine3D ( b0, b1, m_debugvis[i].clr );
		}
	}
	end3D();


	start3D(m_cam[0]);
	setview2D(getWidth(), getHeight());

	// Draw prisms in 3D		
	if ( m_draw_prisms ) {


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
	}	
	end3D();		
	draw3D();

	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::UpdateCamera()
{
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

		RaycastDebug ( x, y, m_out_img );

		appPostRedisplay();	// Update display
	} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam[i]->moveRelative(float(dx) * fine * m_cam[i]->getOrbitDist() / 1000, float(-dy) * fine * m_cam[i]->getOrbitDist() / 1000, 0);
		UpdateCamera();
	} break;

	case AppEnum::BUTTON_RIGHT: {

		// Adjust camera orbit 
		Vector3DF angs = m_cam[i]->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam[i]->setOrbit(angs, m_cam[i]->getToPos(), m_cam[i]->getOrbitDist(), m_cam[i]->getDolly());		
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
	int i = m_currcam;

	float zoomamt = 1.0;
	float dist = m_cam[i]->getOrbitDist();
	float dolly = m_cam[i]->getDolly();
	float zoom = (dist - dolly) * 0.001f;
	dist -= delta * zoom * zoomamt;

	m_cam[i]->setOrbit(m_cam[i]->getAng(), m_cam[i]->getToPos(), dist, dolly);

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

	case 'c':
		m_currcam = 1 - m_currcam;
		break;
	}
}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

	m_cam[0]->setSize( w, h );
	m_cam[0]->setAspect(float(w) / float(h));
	m_cam[0]->setOrbit(m_cam[0]->getAng(), m_cam[0]->getToPos(), m_cam[0]->getOrbitDist(), m_cam[0]->getDolly());
	m_cam[0]->updateMatricies();

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




