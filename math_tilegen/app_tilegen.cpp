//--------------------------------------------------------
// JUST MATH:
// Tilegen, R.Hoetzlein
//
//--------------------------------------------------------------------------------
// Copyright 2019-2023 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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
#include "quaternion.h"
#include "string_helper.h"

#include "mesh.h"

// shader slots (must match glsl layout of shader being used)
#define slotPos		0
#define slotClr		1
#define slotUVs		2
#define slotNorm	3

// VBO buffer ids
#define VBO_NULL	255
#define VBO_POS		0
#define VBO_NORM	1
#define VBO_UVS		2
#define VBO_CLR		3
#define VBO_FACES	4
#define VBO_MAX		5

#define MAX_GRP		10

// material
// describes a material according to the .mtl
// Material Template Library spec
struct RMtl {
	std::string name;
	Vector3DF Ka;		// ambient
	Vector3DF Kd;		// diffuse
	Vector3DF Ks;		// specular	
	Vector3DF Tf;		// transparent clr
	float d;			// transparency (dissolve)
	float Ni;			// index of refraction
	float Ns;			// specular power	
};

// face group
struct RGroup { 
	std::string		mtl_name;
	int				mtl_id;
	int64_t			face_start;
	int64_t			face_end;
};

// renderable mesh
struct RMesh {

	RMesh()	{ 
		  for (int v=0;v <VBO_MAX;v++)
			mVBO[v] = VBO_NULL;
	}

	MeshX*		mesh;				// mesh geometry (cpu)
	int			vert_cnt;			// total vert count

	GLint		mVBO[VBO_MAX];		// gpu storage (opengl VBOs)

	std::vector<RGroup>  mFaceGrps;	// face groups

};


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

	void		drawGrid (Vector4DF clr);

	void		LoadMaterials ( std::string name );
	void		ResolveMaterials ();
	int			FindMaterial ( std::string name ) ;
	

	void		AddMesh ( std::string name );
	void		RenderMesh ( RMesh& rm );
	void		DrawMesh ( RMesh& rm ) ;
		
	std::vector<RMesh>		m_meshes;

	std::vector<RMtl>		m_materials;

	Camera3D*	m_cam;
	int			mouse_down;
};
Sample obj;


bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width &f height

	addSearchPath ( ASSET_PATH );

	init2D ( "arial" );
	setview2D ( w, h );	
	setText ( 16, 1 );		

	// Load meshes
	// - must have correct 'face orientation'
	// - only triangle meshes (no quads)

	AddMesh ( "01_rail_bend.obj" );
	AddMesh ( "02_rail_straight.obj" );

	// Load library file (if specified)
	std::string mtl_lib = m_meshes[0].mesh->getMtlLibrary();
	if ( mtl_lib.size() > 0 ) {
		LoadMaterials ( mtl_lib );
	}

	// Resolve materials
	ResolveMaterials ();

	// Create camera
	m_cam = new Camera3D;
	m_cam->setFov ( 120 );
	m_cam->setNearFar ( 0.01, 100 );
	m_cam->SetOrbit ( Vector3DF(240, 20,0), Vector3DF(0.5, 0.5, 0.5), 2.5, 0.01 );

	return true;
}

void Sample::LoadMaterials ( std::string name )
{
	// Load materials
	std::string fpath;
	if ( !getFileLocation (name, fpath) ) {
		printf ( "ERROR: Unable to find %s\n", name.c_str() );
		exit(-2);
	}	
	char buf[4096];
	strncpy (buf, fpath.c_str(), 2048);
	FILE* fp = fopen ( buf, "rt" );	
	if ( fp == 0 ) {
		dbgprintf ( "ERROR: Cannot open file %s\n", buf );
		exit(-3);
	}
	std::string word, strline;
	
	// read lines	
	RMtl* mtl;
	int curr_mtl = -1;
	
	while ( feof( fp ) == 0 ) {
		fgets ( buf, 4096, fp );
		strline = buf;
		word = strSplitLeft ( strline, " " );
		strline = strTrim(strline);

		if (word.compare("newmtl")==0) {
			// allocate new mtl
			RMtl newmtl;
			m_materials.push_back ( newmtl );
			curr_mtl++;
			// get mtl					
			mtl = &m_materials[ curr_mtl ];
			mtl->name = strline;
		}
		if (word.compare("Ns")==0) {
			mtl->Ns = strToF ( strline );
		}
		if (word.compare("Ni")==0) {
			 mtl->Ni = strToF ( strline );
		}
		if (word.compare("d")==0) {
			 mtl->d = strToF ( strline );
		}
		if (word.compare("Ka")==0) {
			 strToVec3 ( strline, '*', ' ', '*', &mtl->Ka.x );	
		}
		if (word.compare("Kd")==0) {
			 strToVec3 ( strline, '*', ' ', '*', &mtl->Kd.x );	
		}
		if (word.compare("Ks")==0) {
			 strToVec3 ( strline, '*', ' ', '*', &mtl->Ks.x );	
		}
	}

	fclose(fp);
}


void Sample::ResolveMaterials ()
{
	int mid;

	// for all meshes
	for (int n=0; n < m_meshes.size(); n++) {
		RMesh* m = &m_meshes[n];

		// for each face group in a mesh..
		for (int g =0; g < m->mFaceGrps.size(); g++) {
			
			// find material by name and assign to face group
			mid = FindMaterial ( m->mFaceGrps[g].mtl_name );			
			m->mFaceGrps[g].mtl_id = mid;

			// during render we can directly access the face group 
			// to get material ID, which gives material params (Ks,Kd,..)
		}
	}
}

int Sample::FindMaterial ( std::string name ) 
{
	for (int n=0; n < m_materials.size(); n++) {
		if ( m_materials[n].name.compare(name)==0 )
			return n;
	}
	return -1;
}

void Sample::AddMesh ( std::string name )
{
	RMesh rm;

	// Allocate mesh object
	rm.mesh = new MeshX;

	// Load geometry from disk		
	std::string fpath;
	if ( !getFileLocation (name, fpath) ) {
		printf ( "ERROR: Unable to find %s\n", name.c_str() );
		exit(-2);
	}
	if ( !rm.mesh->Load ( fpath ) ) {
		printf ( "ERROR: Unable to load %s\n", name.c_str() );
		exit(-3);
	}

	// Retrieve material groups
	if ( rm.mesh->isActive ( BMTL ) ) {

		RGroup grp;
		AttrV3 info;
		for (int g=0; g < rm.mesh->GetNumElem (BMTL); g++) {
			info = *(AttrV3*) rm.mesh->GetElem (BMTL, g);
			grp.mtl_name = rm.mesh->getGrpMtl ( g );
			grp.face_start = info.v1;
			grp.face_end = info.v2;
			rm.mFaceGrps.push_back (grp);
		}
	}
	
	// Allocate VBO buffers
	int grp = 0;
	rm.vert_cnt = 3 * rm.mesh->GetNumElem ( BFACEV3 );
	
	if ( rm.mesh->isActive ( BVERTPOS ) ) {
		glGenBuffers ( 1, (GLuint*) &rm.mVBO[ VBO_POS ] );
		glBindBufferARB ( GL_ARRAY_BUFFER_ARB, rm.mVBO[ VBO_POS ] );		
		glBufferDataARB ( GL_ARRAY_BUFFER_ARB, rm.mesh->GetBufSize(BVERTPOS), rm.mesh->GetBufData(BVERTPOS), GL_DYNAMIC_DRAW);
	}
	if ( rm.mesh->isActive ( BVERTCLR ) ) {
		glGenBuffers ( 1, (GLuint*) &rm.mVBO[ VBO_CLR ] );
		glBindBufferARB ( GL_ARRAY_BUFFER_ARB, rm.mVBO[ VBO_CLR ] );

		// MeshX packs colors into ints (RGBA), so we 
		// need to repack 4-byte colors (32-bits) into 16-byte Vec4F (128-bits).
		//int ndx_cnt = rm.vert_cnt[grp];
		int ndx_cnt = rm.mesh->GetNumElem ( BVERTCLR );
		int repack_sz =  ndx_cnt * sizeof(Vector4DF);
		Vector4DF* repack_buf = (Vector4DF*) malloc ( repack_sz );
		uint32_t*  src_buf = (uint32_t*) rm.mesh->GetBufData( BVERTCLR );
		Vector4DF* dst_buf = repack_buf;
		for (int n=0; n < ndx_cnt; n++) {
			*dst_buf++ = CLRVEC( *src_buf );		// cast uint clr to Vec4F clr
			src_buf++;
		}

		glBufferDataARB ( GL_ARRAY_BUFFER_ARB, repack_sz, repack_buf, GL_DYNAMIC_DRAW);

		free (repack_buf);
	}

	if ( rm.mesh->isActive ( BVERTNORM ) ) {				
		glGenBuffers ( 1, (GLuint*) &rm.mVBO[ VBO_NORM ] );
		glBindBufferARB ( GL_ARRAY_BUFFER_ARB, rm.mVBO[ VBO_NORM ] );
		glBufferDataARB ( GL_ARRAY_BUFFER_ARB, rm.mesh->GetBufSize(BVERTNORM), rm.mesh->GetBufData(BVERTNORM), GL_DYNAMIC_DRAW);
	}
	if ( rm.mesh->isActive ( BVERTTEX ) ) {		
		glGenBuffers ( 1, (GLuint*) &rm.mVBO[ VBO_UVS ] );
		glBindBufferARB ( GL_ARRAY_BUFFER_ARB, rm.mVBO[ VBO_UVS ] );
		glBufferDataARB ( GL_ARRAY_BUFFER_ARB, rm.mesh->GetBufSize(BVERTTEX), rm.mesh->GetBufData(BVERTTEX), GL_DYNAMIC_DRAW);		
	}
	if ( rm.mesh->isActive ( BFACEV3 ) ) {		
		glGenBuffers ( 1, (GLuint*) &rm.mVBO[ VBO_FACES ] );
		glBindBufferARB ( GL_ELEMENT_ARRAY_BUFFER_ARB, rm.mVBO[ VBO_FACES ] );

		// MeshX supports very large meshes natively, with int64_t so we 
		// need to repack 64-bit vertex indices into 32-bits for OpenGL.
		int ndx_cnt = rm.mesh->GetBufSize( BFACEV3 ) / sizeof(int64_t);
		int repack_sz =  ndx_cnt * sizeof(int32_t);
		int32_t* repack_buf = (int32_t*) malloc ( repack_sz );
		int64_t* src_buf = (int64_t*) rm.mesh->GetBufData( BFACEV3 );
		int32_t* dst_buf = repack_buf;
		for (int n=0; n < ndx_cnt; n++) {
			*dst_buf++ = (int32_t) *src_buf++;		// cast to 32-bit int
		}
		glBufferDataARB ( GL_ELEMENT_ARRAY_BUFFER_ARB, repack_sz, repack_buf, GL_DYNAMIC_DRAW );

		free ( repack_buf);
	}
	checkGL ("mesh bindings");

	// Add to master list
	m_meshes.push_back ( rm );
}

void Sample::RenderMesh ( RMesh& rm )
{
	// Bind mesh geometry to shader slots
	int grp = 0;	


	// bind pos
	glEnableVertexAttribArray ( slotPos );
	glBindBuffer(GL_ARRAY_BUFFER, rm.mVBO[ VBO_POS ]);
	glVertexAttribPointer( slotPos, 3, GL_FLOAT, GL_FALSE, 0x0, 0);		// Bind vertices				

	// bind normals
	if (rm.mVBO[VBO_NORM] != VBO_NULL) {
		glEnableVertexAttribArray ( slotNorm );
		glBindBuffer(GL_ARRAY_BUFFER, rm.mVBO[ VBO_NORM ]);
		glVertexAttribPointer( slotNorm, 3, GL_FLOAT, GL_FALSE, 0x0, 0);		// Bind normals
	} else {
		glDisableVertexAttribArray ( slotNorm );
		glVertexAttrib3f( slotNorm, 1.0, 1.0, 1.0 );		// value when not bound
	}

	// bind texture coords
	if (rm.mVBO[VBO_UVS] != VBO_NULL) {			
		glEnableVertexAttribArray ( slotUVs );
		glBindBuffer(GL_ARRAY_BUFFER, rm.mVBO[ VBO_UVS ]);
		glVertexAttribPointer( slotUVs, 2, GL_FLOAT, GL_FALSE, 0x0, 0);		
	} else {
		glDisableVertexAttribArray ( slotUVs );
		glVertexAttrib2f( slotUVs, 1.0, 1.0 );		// value when not bound
	}

	// bind clr
	if (rm.mVBO[VBO_CLR] != VBO_NULL) {
		glEnableVertexAttribArray ( slotClr );
		glBindBuffer(GL_ARRAY_BUFFER, rm.mVBO[VBO_CLR]);
		glVertexAttribPointer( slotClr, 4, GL_FLOAT, GL_FALSE, 0x0, 0);		// Bind color (float4)
	} else {
		glDisableVertexAttribArray ( slotClr );
		glVertexAttrib4f( slotClr, 1.0, 1.0, 1.0, 1.0 );	// value when not bound
	}
	// bind face indices		
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rm.mVBO[VBO_FACES] );				// Bind face indices		
	
	// Draw elements	

	if (rm.mFaceGrps.size() > 0 ) {
		
		// use face groups
		int mtl_id;
		int vstart, vend, vcnt;
		int num_grp = rm.mFaceGrps.size();		
		for (int i=0; i < num_grp; i++) {
			vstart = rm.mFaceGrps[i].face_start * 3;
			vend = rm.mFaceGrps[i].face_end * 3;
			vcnt = vend - vstart + 3;

			mtl_id = rm.mFaceGrps[i].mtl_id;
			if (  mtl_id >= 0 ) {
				// assign color to slot
				Vector3DF Kd = m_materials [ mtl_id].Kd;
				glDisableVertexAttribArray ( slotClr );
				glVertexAttrib4f( slotClr, Kd.x, Kd.y, Kd.z, 1.0 );

				// set material on shader
				setMaterial ( S3D, m_materials [ mtl_id].Ks, Kd, m_materials [ mtl_id].Ns, 1.0 );
			}

			glDrawElements ( GL_TRIANGLES, vcnt, GL_UNSIGNED_INT, (void*) (vstart * sizeof(uint)) );	
		}
	} else {

		// entire mesh. no groups.
		glDrawElements ( GL_TRIANGLES, rm.vert_cnt, GL_UNSIGNED_INT, (void*) 0 );	
	}	
}


void Sample::DrawMesh ( RMesh& rm ) 
{
	// Debugging only:
	// draw mesh face-by-face (slow)

	Vector3DF n, V;
	Vector3DF v0, v1, v2;
	Vector3DF n0, n1, n2;
	Vector3DF uv0, uv1, uv2;
	CLRVAL c0,c1,c2;
	AttrV3* f;		// contains 64-bit vertex indices
	
	Vector4DF lclr(1,1,1, 0.3);		// line color
	Vector4DF fclr(1,1,1, 1);		// face color

	int num_tri = rm.mesh->GetNumElem ( BFACEV3 );	

	int lines = 1;
	float normals = 0.0f;		// 0.01f 

	for (int i = 0; i < num_tri; i++)  {
		f = (AttrV3*) rm.mesh->GetElem (BFACEV3, i );		
		// get face vertices & normals
		v0 = *rm.mesh->GetVertPos( f->v1 );		v1 = *rm.mesh->GetVertPos( f->v2 );		v2 = *rm.mesh->GetVertPos( f->v3 );		
		n0 = *rm.mesh->GetVertNorm( f->v1 );	n1 = *rm.mesh->GetVertNorm( f->v2 );	n2 = *rm.mesh->GetVertNorm( f->v3 );			
		c0 = *rm.mesh->GetVertClr( f->v1 );	    c1 = *rm.mesh->GetVertClr( f->v2 );	    c2 = *rm.mesh->GetVertClr( f->v3 );			
		V = m_cam->getPos() - v0;
		if ( n.Dot ( V ) >= 0 ) {
			// draw triangle
			Vector4DF clr;
			clr = CLRVEC( c0 );

			//drawTri3D ( v0.x,v0.y,v0.z, v1.x,v1.y,v1.z, v2.x,v2.y,v2.z, n0.x,n0.y,n0.z, clr.x,clr.y,clr.z,1);

			if ( lines > 0 ) {
				drawLine3D ( v0, v1, lclr ); 
				drawLine3D ( v1, v2, lclr );	
				drawLine3D ( v2, v0, lclr ); 
			}
			if ( normals > 0 ) {
				drawLine3D ( v0, v0 + n0 * normals, Vector4DF(0,1,1,0.5) );
				drawLine3D ( v1, v1 + n1 * normals, Vector4DF(0,1,1,0.5) );
				drawLine3D ( v2, v2 + n2 * normals, Vector4DF(0,1,1,0.5) );
			}
		}
	}
}


void Sample::drawGrid( Vector4DF clr )
{
	float o = -0.01;
	for (int n=-10; n <= 10; n++ ) {
		drawLine3D ( Vector3DF(n, o, -10), Vector3DF(n, o, 10), clr );
		drawLine3D ( Vector3DF(-10, o, n), Vector3DF(10, o, n), clr );
	}
}




void Sample::display ()
{	
	Vector3DF pnt;
	Vector4DF clr;

	int w = getWidth();
	int h = getHeight();

	clearGL();	

	// 2D rendering (overlay)
    setview2D(getWidth(), getHeight());
	start2D();
	end2D();

	// 3D rendering (widgets)
	start3D(m_cam);
		// Draw ground
		drawGrid( Vector4DF(.1, .1, .1, 1) );

		drawBox3D ( Vector3DF(0,0,0), Vector3DF(1,1,1), 1,1,1,1 );

		// draw outlines and normals
		for (int i=0; i < m_meshes.size(); i++ )
			DrawMesh ( m_meshes[i] );		

	end3D();


	// Direct rendering (non-gui)	
	//
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_TWO_SIDE);

	selfDraw3D ( m_cam, S3D );
	setLight ( S3D, 3, 6, 0, 5,5,5);

		selfSetTexture();	// no texturing
		
		// render mesh fast
		for (int i=0; i < m_meshes.size(); i++ )
			RenderMesh ( m_meshes[i] );

	selfEndDraw3D();

	draw2D (); 	
	draw3D ();
	
	appPostRedisplay();								// Post redisplay since simulation is continuous
}


void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	int w = getWidth(), h = getHeight();				// window width & height

	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;

	if (mouse_down == AppEnum::BUTTON_LEFT) {
		
	}
}


void Sample::motion (AppEnum button, int x, int y, int dx, int dy) 
{
	// Get camera for scene
	bool shift = (getMods() & KMOD_SHIFT);		// Shift-key to modify light
	float fine = 0.5f;
	Vector3DF dang; 

	switch ( mouse_down ) {	
	case AppEnum::BUTTON_LEFT:  {	
	
		} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		float zoom = (m_cam->getOrbitDist() - m_cam->getDolly()) * 0.003f;
		m_cam->moveRelative ( float(dx) * zoom, float(-dy) * zoom, 0 );	
		} break; 

	case AppEnum::BUTTON_RIGHT: {
		// Adjust orbit angles
		Vector3DF angs = m_cam->getAng();
		angs.x += dx*0.2f;
		angs.y -= dy*0.2f;				
		m_cam->SetOrbit ( angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly() );
		} break;	

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
}




void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	switch ( keycode ) {
	};
}

void Sample::reshape (int w, int h)
{
	glViewport ( 0, 0, w, h );
	setview2D ( w, h );

	m_cam->setAspect(float(w) / float(h));
	m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());	
		
	appPostRedisplay();	
}

void Sample::startup ()
{
	int w = 1900, h = 1000;
	appStart ( "Tilegen", "Tilegen", w, h, 4, 2, 16, false );
}

void Sample::shutdown()
{
}


