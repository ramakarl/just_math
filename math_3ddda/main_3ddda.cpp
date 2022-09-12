//--------------------------------------------------------
// JUST MATH:
// 3DDDA - 3D differential analyzer. Identify voxels in a volume along a line. 
// 
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
#include <GL/glew.h>
#include <algorithm>

#define V_3DDDA		0


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

	void		allocVoxels(int id, Vector3DI res);
	void		clearVoxels(int id);
	void		setVoxel(int id, int x, int y, int z, char val);
	void		drawVoxels(int id);
	void		Run3DDDA();
	
	void		Reset ();

	int			mouse_down;
	int			mouse_action;
	int			m_adjust, m_voxshow;

	Camera3D*	m_cam;
	Vector3DF	m_goal;
	
	Vector3DF	m_vmin, m_vmax;
	Vector3DI	m_vres;

	Vector3DF	line[2];
	Vector3DF	m_pnt;		// test point

	char*		vox[5];

	int			m_voxel_visit, m_voxel_set, m_voxel_count;
};
Sample sample_obj;

void Sample::Reset ()
{
}

bool Sample::init ()
{	
	int w = getWidth(), h = getHeight();			// window width & height
	for (int n = 0; n < 5; n++) vox[n] = 0x0;
	
	addSearchPath(ASSET_PATH);
	init2D ( "arial" );
	setText(18,1);
	setview2D ( w, h );	

	m_cam = new Camera3D;
	m_cam->setNearFar ( 1, 2000 );
	m_cam->setOrbit ( Vector3DF(140,30,0), Vector3DF(32,16,32), 400, 70 );
	reshape( w, h );
	m_adjust = -1;

	// create triangle
	line[0].Set(30, 5, 20);
	line[1].Set(50, 10, 50);

	// create volume
	m_vmin.Set(0, 0, 0);
	m_vmax.Set(64, 64, 64);
	m_vres.Set(32, 32, 32);
	
	allocVoxels(V_3DDDA, m_vres);
	
	// voxelize triangle into volume
	m_voxshow = V_3DDDA;
	Run3DDDA();	

	return true;
}

void Sample::allocVoxels(int id, Vector3DI res)
{
	if (vox[id] != 0x0) free(vox[id]);
	vox[id] = (char*) malloc(res.x*res.y*res.z * sizeof(char));
	m_vres = res;
	clearVoxels(id);
}

void Sample::clearVoxels(int id)
{
	memset(vox[id], 0, m_vres.x*m_vres.y*m_vres.z * sizeof(char));
}

void Sample::setVoxel(int id, int x, int y, int z, char val)
{	
	if (x < 0 || y < 0 || z < 0 || x >= m_vres.x || y >= m_vres.y || z >= m_vres.z) return;
	m_voxel_set++;
	char* v = vox[id];
	*(v + (z*m_vres.y + y)*m_vres.x + x) = val;
}
void Sample::drawVoxels(int id)
{
	Vector3DF p, d;
	Vector3DF o(0.01, 0.01, 0.01f);
	d = (m_vmax - m_vmin) / m_vres;
	
	m_voxel_count = 0;
	int i = 0;
	char* v = vox[id];
	for (int z = 0; z < m_vres.z; z++)
		for (int y = 0; y < m_vres.y; y++)
			for (int x = 0; x < m_vres.x; x++) {
				i = (z*m_vres.y + y)*m_vres.x + x;
				p.Set(m_vmin.x + x*(m_vmax.x - m_vmin.x) / m_vres.x, m_vmin.y + y*(m_vmax.y - m_vmin.y) / m_vres.y, m_vmin.z + z*(m_vmax.z - m_vmin.z) / m_vres.z);
				if (v[i] != 0) {
					m_voxel_count++;
					drawCube3D(p + o, p + d - o, 2 * float(x) / m_vres.x, 2 * float(y) / m_vres.y, 2 * float(z) / m_vres.z, 1.0);
				}
			}
}


Vector3DF fabs3(Vector3DF f)
{
	return Vector3DF(abs(f.x), abs(f.y), abs(f.z));
}
Vector3DF floor3(Vector3DF f)
{
	return Vector3DF(int(f.x), int(f.y), int(f.z));
}

void Sample::Run3DDDA ()
{	
	float t, tend;
	Vector3DF d, dstep, mask, s, c, v[2];

	clearVoxels ( V_3DDDA );

	// transform to unit volume
	v[0] = (line[0] - m_vmin) * m_vres / (m_vmax - m_vmin);
	v[1] = (line[1] - m_vmin) * m_vres / (m_vmax - m_vmin);

	// prepare 3DDA	
	c = v[0];				// starting point
	d = v[1] - v[0];		// direction vector
	t = 0;
	tend = d.Length(); d.Normalize();		
	dstep.Set((d.x > 0) ? 1 : -1, (d.y > 0) ? 1 : -1, (d.z > 0) ? 1 : -1);	// signed direction	
	d = fabs3(d);
	if ( d.x < 0.001) d.x = .001;
	if ( d.y < 0.001) d.y = .001;
	if ( d.z < 0.001) d.z = .001;
	s.Set ( ((int(c.x) - c.x + 0.5f)*dstep.x + 0.5) / d.x + t,
			((int(c.y) - c.y + 0.5f)*dstep.y + 0.5) / d.y + t,
			((int(c.z) - c.z + 0.5f)*dstep.z + 0.5) / d.z + t);
	
	// 3DDA 
	m_voxel_set = 0;

	while (t < tend) {
		setVoxel(V_3DDDA, c.x, c.y, c.z, 1);
		mask = (s.x < s.y) ? ((s.x < s.z) ? Vector3DF(1, 0, 0) : Vector3DF(0, 0, 1)) : 	 // choose next voxel (branchless)
							 ((s.y < s.z) ? Vector3DF(0, 1, 0) : Vector3DF(0, 0, 1));
		t = mask.x ? s.x : (mask.y ? s.y : s.z);		// advance t
		s += mask / d;						// advance x/y/z intercepts by the inverse normal (not obvious)
		c += mask * dstep;					// advance to next voxel
	}
	c = v[1];
	setVoxel(V_3DDDA, c.x, c.y, c.z, 1);
}


void Sample::display()
{
	Vector3DF a,b,c;
	Vector3DF p, q, d;

	clearGL();

	// draw text
	start2D();
		setview2D(getWidth(), getHeight());
		drawText(10, 20, "Input:", 1,1,1,1);
		drawText(10, 35, "  Orbit view      Right-mouse drag", 1,1,1,1);
		drawText(10, 50, "  Move end X/Y    Left-click+move circles", 1,1,1,1);		
		drawText(10, 65, "  Move end Z      Right-click+move circles", 1,1,1,1);		
		char buf[1024];
		drawText(10, 80, "Voxels:", 1,1,1,1);
		sprintf(buf, "  Total: %d", m_vres.x*m_vres.y*m_vres.z); drawText(10, 95, buf, 1, 1, 1, 1);
		sprintf(buf, "  Set:   %d", m_voxel_set);	drawText(10, 110, buf, 1, 1, 1, 1);
	end();
	
	// draw in 3D
	start3D(m_cam);
		setLight(S3D, 20, 100, 20);

		// sketch a grid
		for (int z = 0; z < m_vres.z; z++)
			for (int x = 0; x < m_vres.x; x++) {
				p.Set(m_vmin.x + x*(m_vmax.x - m_vmin.x) / m_vres.x, 0, m_vmin.z + z*(m_vmax.z - m_vmin.z) / m_vres.z);
				d = (m_vmax - m_vmin) / m_vres; d.y = 0;
				drawBox3D(p, p + d, 1, 1, 1, 0.05);
			}

		// world axes
		a = Vector3DF(5, 0, 0); b = Vector3DF(0, 5, 0); c = Vector3DF(0, 0, 5);
		p = Vector3DF(0.f, 0.1f, 0.f);
		drawLine3D(p.x, p.y, p.z, p.x + a.x, p.y + a.y, p.z + a.z, 1, 0, 0, 1);
		drawLine3D(p.x, p.y, p.z, p.x + b.x, p.y + b.y, p.z + b.z, 0, 1, 0, 1);
		drawLine3D(p.x, p.y, p.z, p.x + c.x, p.y + c.y, p.z + c.z, 0, 0, 1, 1);
	
		// draw line to be voxelized
		drawLine3D(line[0].x, line[0].y, line[0].z, line[1].x, line[1].y, line[1].z, 1, 1, 0, 1);		// edges

		// draw voxels
		drawVoxels(m_voxshow);

	end3D();

	// overlay - new draw group to turn off depth testing
	start3D(m_cam);
		setdepth3D(false);
		setLight(S3D, 100, 100, 100);
		Vector4DF clr = (m_adjust==0) ? Vector4DF(.5,.5,.5,1) : Vector4DF(1,1,0,1);
		drawCircle3D(line[0], line[0]+Vector3DF(0,1,0), 2.0, clr);	// corners of triangle (interactive)
		drawCircle3D(line[1], line[1]+Vector3DF(0,1,0), 2.0, clr);

		drawLine3D(line[0].x, line[0].y, line[0].z, line[0].x, 0, line[0].z, 0.5, 0.5, 0.5, 1);
		drawLine3D(line[1].x, line[1].y, line[1].z, line[1].x, 0, line[1].z, 0.5, 0.5, 0.5, 1);
		drawBox3D ( m_vmin, m_vmax, 0.1, 0.1, 0.1, 1 );
	end();

	// Use NvGui to draw in 2D/3D
	draw3D ();										// Render the 3D drawing groups
	drawGui (0);									// Render the GUI
	draw2D (); 

	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
	float fine = 0.5;

	switch ( mouse_down ) {	
	case AppEnum::BUTTON_LEFT: {
		if ( m_adjust == -1 ) {			
			// Adjust dist
			float dist = m_cam->getOrbitDist();
			dist -= dy*fine;
			m_cam->setOrbit ( m_cam->getAng(), m_cam->getToPos(), dist, m_cam->getDolly() );		
		} else {
			line[m_adjust] = moveHit3D(m_cam, x, y, line[m_adjust], Vector3DF(0, 1, 0)); // xz plane			
			Run3DDDA();
		}		
		appPostRedisplay();		// Update display
		} break;
	
	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam->moveRelative ( float(dx) * fine*m_cam->getOrbitDist()/1000, float(-dy) * fine*m_cam->getOrbitDist()/1000, 0 );	
		appPostRedisplay();		// Update display
		} break;
	
	case AppEnum::BUTTON_RIGHT: {	
	
		if ( m_adjust == -1 ) {
			// Adjust camera orbit 
			Vector3DF angs = m_cam->getAng();
			angs.x += dx*0.2f*fine;
			angs.y -= dy*0.2f*fine;				
			m_cam->setOrbit ( angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly() );				
		} else {
			line[m_adjust] = moveHit3D(m_cam, x, y, line[m_adjust], Vector3DF(0, 0, 1));   // xy plane			
			Run3DDDA();
		}


		appPostRedisplay();	// Update display
		} break;
	}
}


void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	if (guiHandler(button, state, x, y)) return;
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag
	m_adjust = -1;

	if (state==AppEnum::BUTTON_PRESS) {		
		// check for interaction with corners of triangle
		for (int i=0; i < 2; i++) {
			if ( checkHit3D (m_cam, x, y, line[i], 2.0) ) m_adjust = i;
		}		
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
	m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), dist, dolly);
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action==AppEnum::BUTTON_RELEASE) return;

	switch ( keycode ) {
	case 'c': case 'C':	m_adjust = -1;	break;	
	case '1':		 	m_adjust = 0;	break;
	case '2':		 	m_adjust = 1;	break;	
	};
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
	int w=1900, h=1000;	

	appStart ( "3DDDA", "3DDDA", w, h, 4, 2, 16, false );
}



