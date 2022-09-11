//--------------------------------------------------
// Just Math:
// Basis - Change of orthonormal basis
//
// An orthonormal basis is defined by three perpendical vectors in 3D space.
//
// This demo shows how an orthonormal basis can be used to transform a coordinate space.
// Several graphics are drawn when running this demo:
//  Graphic 1 (center). The input is an angular distribution of points across two spatial angles, in world space.
//  Graphic 2 (above left). The output space is an oriented orthonormal basis. (drawn top left of center)
//  Graphic 3 (above right). Re-normalization of the basis so that Y is up.
//  Graphic 4 (far right). Concatenated basis showing basis applied multiple times. 
//
// The code demonstrates the following functions:
//   RemakeBasis -			build a basis from 3x column vectors
//   RemakeBasisFromAngles - build a basis from angles
//   RemakeBasisUpward -    build a basis from a vector, renormalized upward
//   RemakeBasisFromLines - build a basis from a line
//   RemakeBasisCopy -		build a basis by copying another
//
//--------------------------------------------------------------------------------
// Copyright 2019-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
// 
// * Derivative works may append the above copyright notice but should not remove or modify earlier notices.
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

struct Basis {
	Vector3DF fwd;
	Vector3DF up;
	Vector3DF side;	

	Vector3DF ctr;
};

#define CONSTR		0
#define UI			1
#define TRUNK		2
#define TRUNK2		3
#define BRANCH0		4
#define BRANCH1		5
#define BRANCH2		6
#define UI_DEFAULT	7

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

	void		drawGrid();
	void		drawBasis(Basis& b);
	void		RebuildLines(Vector3DF angs);
	void		RemakeBasis ( Basis& dest, Vector3DF fwd, Vector3DF up, Vector3DF side, Vector3DF ctr );
	void		RemakeBasisFromAngles (Basis& dest, Basis src, Vector3DF angs );
	void		RemakeBasisUpward (Basis& dest, Basis src, Vector3DF pos );
	void		RemakeBasisFromLines ( Basis& dest, Basis src, Basis constr, int i);
	void		RemakeBasisCopy (Basis& dest, Basis src, Vector3DF pos );
	
	Basis		basis[10];

	std::vector<Vector3DF>	m_lines;

	Vector3DF	angs_distrib, angs_ui;
	int			mouse_down;
	int			mouse_mod;
	Camera3D*	m_cam;
};
Sample obj;


bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width & height

	addSearchPath ( ASSET_PATH );
	init2D ( "arial" );
	setview2D ( w, h );	
	setText ( 16, 1 );		
	
	mouse_mod = 1;

	m_cam = new Camera3D;
	m_cam->setOrbit ( Vector3DF(-40, 30,0), Vector3DF(0,0,0), 100, 1 );

	// construction basis - fwd=X, up=Y 
	RemakeBasis( basis[CONSTR], Vector3DF(1, 0, 0), Vector3DF(0, 1, 0), Vector3DF(0, 0, 1), Vector3DF(0, 0, 0) );
	
	// interface basis - fwd=Y, up=X
	angs_ui = Vector3DF(0,-70,0);
	RemakeBasis (basis[UI_DEFAULT], Vector3DF(0, 1, 0), Vector3DF(-1, 0, 0), Vector3DF(0, 0, 1), Vector3DF(0, 0, -10));
	RemakeBasisFromAngles (basis[UI], basis[UI_DEFAULT], angs_ui );
	
	// distributed lines
	angs_distrib = Vector3DF(20,0,0);
	RebuildLines ( angs_distrib );
	RemakeBasisUpward(basis[TRUNK], basis[UI], Vector3DF(10, 0, 0));
	
	// concatenated bases
	RemakeBasisCopy (basis[TRUNK2], basis[TRUNK], Vector3DF(20, 0, 0));
	RemakeBasisFromLines ( basis[BRANCH0], basis[TRUNK2], basis[CONSTR], 0 );
	RemakeBasisFromLines ( basis[BRANCH1], basis[TRUNK2], basis[CONSTR], 1 );
	RemakeBasisFromLines ( basis[BRANCH2], basis[TRUNK2], basis[CONSTR], 2 );


	return true;
}

void Sample::drawGrid()
{
	float o	 = -0.05;		// offset
	for (int n=-100; n <= 100; n+=10 ) {
		drawLine3D ( n, o,-100, n, o,100, .5,.5,.5, 1);
		drawLine3D (-100, o, n, 100, o, n, .5, .5, .5, 1);
	}
}

void Sample::RebuildLines ( Vector3DF angs ) 
{
	Quaternion q1, q2;
	Vector3DF p, r;

	m_lines.clear();
	
	// first six lines define a frustum
	for (int y=0; y <= 1; y++) {
		for (int x=-1; x<=1; x++) {
			p.Set(1, 0, 0);
			q1.fromAngleAxis( y* angs.y * DEGtoRAD, Vector3DF(0, 0, 1)); q1.normalize(); p *= q1;
			q2.fromAngleAxis( x * angs.x * DEGtoRAD, Vector3DF(0, 1, 0)); q2.normalize(); p *= q2;
			p.Normalize(); p *= 5.0; m_lines.push_back(p);
		}
	}
	// random distribution lines within frustum
	for (int n = 0; n < 100; n++) {
		r.Random(-1, 1, 0, 1, 0, 1);	
		p.Set(1, 0, 0);
		q1.fromAngleAxis ( r.y*angs.y*DEGtoRAD, Vector3DF(0,0,1) ); q1.normalize(); p *= q1;
		q2.fromAngleAxis ( r.x*angs.x*DEGtoRAD, Vector3DF(0, 1, 0)); q2.normalize(); p *= q2;
		p.Normalize (); p *= 5.0;
		m_lines.push_back( p );
	}
}

void Sample::RemakeBasis ( Basis& dest, Vector3DF fwd, Vector3DF up, Vector3DF side, Vector3DF ctr )
{
	dest.fwd = fwd;
	dest.up = up;
	dest.side = side;
	dest.ctr = ctr;
}
void Sample::RemakeBasisCopy(Basis& dest, Basis src, Vector3DF pos)
{
	dest = src;
	dest.ctr = pos;
}

void Sample::RemakeBasisFromAngles (Basis& dest, Basis src, Vector3DF angs)
{
	// construct a basis with two angles defining the foward direction
	// this is just to adjust the UI basis for the demo
	Quaternion q1; q1.fromAngleAxis ( angs.y*DEGtoRAD, src.side );
	Quaternion q2; q2.fromAngleAxis ( angs.x*DEGtoRAD, src.fwd );
	Vector3DF fwd;
	fwd = src.fwd;
	fwd *= q1;
	fwd *= q2;
	fwd.Normalize();
	dest.fwd = fwd;		dest.fwd.Normalize();
	dest.side.Cross ( dest.fwd, Vector3DF(-1,0,0));		dest.side.Normalize();
	dest.up.Cross ( dest.side, dest.fwd );				dest.up.Normalize();
	dest.ctr = src.ctr;
}
void Sample::RemakeBasisUpward (Basis& dest, Basis src, Vector3DF pos)
{
	Vector3DF up (0,1,0);
	dest.fwd = src.fwd;
	dest.side.Cross( dest.fwd, up );		dest.side.Normalize();
	dest.up.Cross( dest.side, dest.fwd );	dest.up.Normalize();
	dest.ctr = pos;
}

void Sample::RemakeBasisFromLines(Basis& dest, Basis src, Basis constr, int i)
{
	// find the trunk line in the src basis
	Matrix4F m;
	m.SRT(src.fwd, src.up, src.side, Vector3DF(0, 0, 0), 1 );		// which trunk line
	dest.fwd = m_lines[ i ] * m; dest.fwd.Normalize();
	dest.ctr = src.ctr + dest.fwd * 5.0f;									// center of new branch basis

	RemakeBasisUpward ( dest, dest, dest.ctr );						// remake the basis normalized upward
}

void Sample::drawBasis (Basis& s) 
{
	Vector3DF a,b,c;
	drawLine3D (s.ctr, s.ctr + s.fwd, Vector4DF(1,0,0,1) );
	drawLine3D (s.ctr, s.ctr + s.up, Vector4DF(0,1, 0,1));
	drawLine3D (s.ctr, s.ctr + s.side, Vector4DF(0,0,1,1) );
	drawBox3D ( s.ctr+s.fwd-Vector3DF(.1,.1,.1), s.ctr + s.fwd +Vector3DF(.1,.1,.1), 1,0,0,1 );
}


void Sample::display ()
{	
	Vector3DF pnt;
	Vector4DF clr;

	int w = getWidth();
	int h = getHeight();

	start2D();
		setview2D(getWidth(), getHeight());
		drawText(10, 20, "Input:", 1,1,1,1);
		drawText(10, 35, "  Orbit view       Right-mouse drag", 1,1,1,1);		
		drawText(10, 50, "  1 key            Select source object. Change the angular distribution, world coordinates", 1,1,1,1);		
		drawText(10, 65, "  2 key            Select target object. Rotate the target basis", 1,1,1,1);			
		drawText(10, 80, "  Modify selected  Left-click + drag", 1,1,1,1);		
	end();


	clearGL();

	Vector3DF ctr;

	start3D(m_cam);
		drawGrid();
				
		RemakeBasisUpward ( basis[TRUNK], basis[UI], Vector3DF(10,0,0) );

		RemakeBasisCopy( basis[TRUNK2], basis[TRUNK], Vector3DF(20, 0, 0));
		RemakeBasisFromLines (basis[BRANCH0], basis[TRUNK2], basis[CONSTR], 0);
		RemakeBasisFromLines (basis[BRANCH1], basis[TRUNK2], basis[CONSTR], 1);
		RemakeBasisFromLines (basis[BRANCH2], basis[TRUNK2], basis[CONSTR], 2);


		// draw all basis
		for (int b=0; b <= 6; b++ ) {
			drawBasis( basis[b] );
	
			for (int n=0; n < m_lines.size(); n++) {				
				// transform a point to an new basis (from the construction basis) 
				//Matrix4F m;
				//m.toBasis (basis[b].fwd, basis[b].up, basis[b].side );
				Quaternion q;
				q.toBasis (basis[b].fwd, basis[b].up, basis[b].side);

				pnt = m_lines[n] * q;
				ctr = basis[b].ctr;
				clr = (n < 6 ? Vector4DF(1,1,1,1) : Vector4DF(1,1,1,.15 ) );
				drawLine3D( ctr, ctr + pnt, clr );
			}
		}

		if ( mouse_down == AppEnum::BUTTON_LEFT ) {
			Vector3DF h;
			// draw a cyan ring to show how the UI basis works
			h = Vector3DF(0, 5.0*basis[UI].fwd.y, 0 ) + basis[UI].ctr;
			float r = 5.0 * sqrt(basis[UI].fwd.x* basis[UI].fwd.x + basis[UI].fwd.z* basis[UI].fwd.z);
			drawCyl3D ( h, h+Vector3DF(0,.01,0), r, r+.05, Vector4DF(0,1,1,1));
		}
	end3D();

	draw3D ();
	draw2D (); 	
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
	
		dang = Vector3DF(dx*0.1, dy*0.1, 0.0);
		
		switch ( mouse_mod ) {
		case 1:	{
			angs_distrib += dang;	
			if (angs_distrib.x< 0) angs_distrib.x = 0;
			RebuildLines ( angs_distrib );	
		} break;
		case 2:	
			angs_ui += dang;	
			RemakeBasisFromAngles (basis[UI], basis[UI_DEFAULT], angs_ui );	
			break;
		};

		} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		float zoom = (m_cam->getOrbitDist() - m_cam->getDolly()) * 0.0003f;
		m_cam->moveRelative ( float(dx) * zoom, float(-dy) * zoom, 0 );	
		#ifdef HIT_PLANE
			Vector3DF hit = intersectLinePlane ( m_cam->getPos(), m_cam->to_pos, Vector3DF(0,0,0), Vector3DF(0,1,0) );
			m_cam->setOrbit ( m_cam->getAng(), hit, m_cam->getOrbitDist(), m_cam->getDolly() );		
		#endif
		} break; 

	case AppEnum::BUTTON_RIGHT: {
		// Adjust orbit angles
		Vector3DF angs = m_cam->getAng();
		angs.x += dx*0.2f*fine;
		angs.y -= dy*0.2f*fine;				
		m_cam->setOrbit ( angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly() );
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

	m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), dist, dolly);		
}




void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action == AppEnum::BUTTON_RELEASE) return;

	switch ( keycode ) {
	case '1':	mouse_mod = 1;	break;
	case '2':	mouse_mod = 2;	break;
	};
}

void Sample::reshape (int w, int h)
{
	glViewport ( 0, 0, w, h );
	setview2D ( w, h );

	m_cam->setAspect(float(w) / float(h));
	m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
	m_cam->updateMatricies();
		
	appPostRedisplay();	
}

void Sample::startup ()
{
	int w = 1900, h = 1000;
	appStart ( "Orthonormal Basis", "Orthonormal Basis", w, h, 4, 2, 16, false );
}

void Sample::shutdown()
{
}

