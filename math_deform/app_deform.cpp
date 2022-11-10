//--------------------------------------------------------
// JUST MATH:
// Deform - spatial deformations in 3D, such as folding, twisting and bending
// 
// This demo shows how to compute these 3D spatial deformations.
// The source volume is defined by a box with points in 3D.
// The target volume is a folded, twisted and bended box.
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

#include <time.h>
#include "main.h"			// window system 
#include "nv_gui.h"			// gui system
#include "quaternion.h"

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
	void		drawPnts();
	void		drawResult();
	
	void		CreatePnts();
	void		StartOutput ();		
	void		DeformBend();
	void		DeformTwist();
	void		DeformFold();

	Vector3DI	m_pnt_min, m_pnt_max, m_pnt_res;

	Vector3DF	m_bend_size;		
	Vector3DF	m_bend_radius;
	Vector3DF	m_bend_ctr;
	Vector3DF	m_bend_amt;
	
	Vector3DF	m_twist_amt;
	Vector3DF	m_twist_size;
	Vector3DF	m_twist_ctr;

	Vector3DF	m_fold_ctr;
	Vector3DF	m_fold_amt;

	std::vector<Vector3DF>	m_input_pos;
	std::vector<Quaternion>	m_input_rot;

	std::vector<Vector3DF>	m_pos;
	std::vector<Quaternion>	m_rot;

	int			m_mode;
	int			mouse_down;
	Camera3D*	m_cam;
};
Sample obj;

void Sample::CreatePnts()
{
	// Make points
	Vector3DF p, s;
	Quaternion q;
	
	m_pnt_min.Set(0,-2,-2);
	m_pnt_max.Set(10,2,2);
	m_pnt_res = m_pnt_max - m_pnt_min + Vector3DI(1,1,1);

	for (p.x = m_pnt_min.x; p.x <= m_pnt_max.x; p.x++) {
		for (p.y = m_pnt_min.y; p.y <= m_pnt_max.y; p.y++) {
			for (p.z = m_pnt_min.z; p.z <= m_pnt_max.z; p.z++) {
				s = p;			
				m_input_pos.push_back(s);
				m_input_rot.push_back(q);
				m_pos.push_back(s);
				m_rot.push_back(q);
			}
		}
	}

}

bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width & height

	m_mode = 2;

	addSearchPath ( ASSET_PATH );
	init2D ( "arial" );
	setview2D ( w, h );	
	setText ( 16, 1 );		
	
	m_cam = new Camera3D;
	m_cam->setOrbit ( Vector3DF(-40, 30,0), Vector3DF(0,0,0), 100, 1 );

	// Create pnts
	CreatePnts();

	// Setup bend
	m_bend_ctr.Set(5, 0, 0);		// center of bend
	m_bend_size.Set(5, 1, 0);		// region to bend. domain = ctr +/- size = <0,0,-2> to <10,0,2>	
	m_bend_radius.Set(5, 0, 0);
	m_bend_amt.Set(-30, 0, 0);

	// Setup twist	
	m_twist_ctr.Set(0, 0, 0);		// center of twist
	m_twist_size.Set(10, 1, 2);		// region to twist
	m_twist_amt.Set (40, 0, 0);		// twist

	// Setup fold
	m_fold_ctr.Set(0, 0, 0);		// center of fold
	m_fold_amt.Set(10, 0, 0);

	return true;
}

void Sample::drawGrid()
{
	float o	 = -0.05;		// offset
	for (int n=-100; n <= 100; n+=10 ) {
		drawLine3D ( n, o,-100, n, o,100, .5,.5,.5, .3);
		drawLine3D (-100, o, n, 100, o, n, .5, .5, .5, .3);
	}
	Vector3DF p(-10,0,0);
	drawLine3D ( p, p+Vector3DF(1,0,0), Vector4DF(1,0,0,1) );
	drawLine3D ( p, p+Vector3DF(0,1,0), Vector4DF(0,1,0,1) );
	drawLine3D ( p, p+Vector3DF(0,0,1), Vector4DF(0,0,1,1));
}
void Sample::drawPnts()
{
	Vector3DI i;
	Vector3DF y, p, pl;
	Vector3DF offs(0, 0, -5);
	int n, nl;

	for (i.x = m_pnt_min.x; i.x <= m_pnt_max.x; i.x++) {
		for (i.y = m_pnt_min.y; i.y <= m_pnt_max.y; i.y++) {
			for (i.z = m_pnt_min.z; i.z <= m_pnt_max.z; i.z++) {
				// draw point
				n = ((i.x-m_pnt_min.x) * m_pnt_res.y + (i.y-m_pnt_min.y)) * m_pnt_res.z + (i.z-m_pnt_min.z);
				y.Set(0,.1,0); y *= m_input_rot[n];				// get orientation				
				p = m_input_pos[n] + offs;
				drawCircle3D( p, p+y, 0.05, Vector4DF(1,1,1,1) );
				drawLine3D( p, p+y, Vector4DF(0,1,0,1) );
				
				// draw rails (lines)
				if ( i.x > m_pnt_min.x && (i.z==m_pnt_min.z || i.z==m_pnt_max.z) && (i.y==m_pnt_min.y || i.y==m_pnt_max.y)) {
					nl = n - (m_pnt_res.y * m_pnt_res.z);		// previous x pnt
					pl = m_input_pos[nl] + offs;
					drawLine3D ( p, pl, Vector4DF(1,1,1,0.7) );
				}
			}
		}
	}
}

void Sample::drawResult()
{
	Vector3DI i;
	Vector3DF y, p, pl;
	Vector3DF offs(0, 0, 5);
	int n, nl;

	for (i.x = m_pnt_min.x; i.x <= m_pnt_max.x; i.x++) {
		for (i.y = m_pnt_min.y; i.y <= m_pnt_max.y; i.y++) {
			for (i.z = m_pnt_min.z; i.z <= m_pnt_max.z; i.z++) {
				// draw point
				n = ((i.x - m_pnt_min.x) * m_pnt_res.y + (i.y - m_pnt_min.y)) * m_pnt_res.z + (i.z - m_pnt_min.z);
				y.Set(0, .1, 0); y *= m_rot[n];					// get orientation				
				p = m_pos[n] + offs;
				drawCircle3D(p, p + y, 0.05, Vector4DF(1, 1, 0, 1));
				drawLine3D(p, p + y, Vector4DF(0, 1, 0, 1));

				// draw rails (lines)
				if (i.x > m_pnt_min.x && (i.z == m_pnt_min.z || i.z == m_pnt_max.z) && (i.y == m_pnt_min.y || i.y == m_pnt_max.y)) {
					nl = n - (m_pnt_res.y * m_pnt_res.z);		// previous x pnt
					pl = m_pos[nl] + offs;
					drawLine3D(p, pl, Vector4DF(1, 1, 0, 0.7));
				}
			}
		}
	}
}

void Sample::StartOutput ()
{
	// copy input points to output so that 
	// subsequent deforms can operate in-place on output data
	for (int n = 0; n < m_pos.size(); n++) {
		m_pos[n] = m_input_pos[n];
		m_rot[n] = m_input_rot[n];
	}
}

void Sample::DeformBend ( )
{
	Vector3DF p, s, c;
	Vector3DF dp;
	Quaternion q,r1,r2;
	Vector3DF d;
	float v, a0, a1, r, bsgn;
	
	// Bending moves the original geometry along a displaced circle:
	//
	// For bending along X-axis with Y-up:
	//    bend_ctr.x = given axial coordinate for bending
	//    bend_amt.x = given total amount of bending (degrees)
	//   bend_size.x = given range over which to apply bend
	// 
	//             C                  C = bend center
	//            / |--              a0 = base angle of bend = bend_amt * x/s
	//           /a0|a1 --           a1 = final angle of bend = bend_amt * x/s   
	//          /   |      --
	//       r /    |Cy       --
	//        /     |           --
	//       /      |              --      x = bend axial ctr 
	// (0,0) ---x---|----------------- s   s = bend size
	//       .      |            .....          
	//         ..   v    ........          . = circle centered at C
	//            .......    
	// Solution: construct a right triangle with bend center at: <x, x/tan(a0), 0>,   tan(a0) = x / Cy
	//
	float ang = m_bend_amt.x;
	float sz = m_bend_size.x * 2;
	float ctr = m_bend_ctr.x;

	bsgn = (ang > 0) ? 1 : -1;
	a0 = -fabs(ang) * ctr / sz;			// a9 = base angle = fractional of bend angle
	a1 = fabs(ang) + a0;
	if ( fabs(a0) > a1 ) {
		c = Vector3DF(ctr, bsgn * ctr / tan(-a0 * DEGtoRAD), 0);		// bend center (better numerically with a0)
	} else {
		c = Vector3DF(ctr, bsgn * (sz-ctr) / tan( a1 * DEGtoRAD), 0);	// bend center (better numerically with a1)
	}
	r = c.Length();

	for (int n = 0; n < m_pos.size(); n++) {		
		p = m_pos[n];
		v = p.x / sz;								// bend fraction (0 < v < 1)
		s = Vector3DF(0, -r * bsgn, 0);
		
		p.x = 0;									// move cross-section to 0
		r1.fromAngleAxis(  (a0 + v*(a1-a0)) * bsgn * DEGtoRAD, Vector3DF(0,0,1) );		// quaternion rotation (Z-axis)
		p *= r1;									// rotate cross-section		
		p += c + (s * r1);							// bend - shift section onto the circle

		m_pos[n] = p;
		m_rot[n] *= r1;
	}
}

void Sample::DeformTwist()
{
	Vector3DF p;
	Vector3DF dp;
	Vector3DF u;
	Quaternion q,q1,q2,q3;
 
	// Twist is simply a rotation along the twist axis
	//      v             v = fraction of rotation = x / s
	//      |
	// --x--|----------s  --> twist axis, where s = twist_size.x 
	//      |
	
	for (int n = 0; n < m_pos.size(); n++) {
	
		p = m_pos[n]-m_twist_ctr;

		q1.fromAngleAxis ( p.x*m_twist_amt.x / m_twist_size.x * DEGtoRAD, Vector3DF(1, 0, 0));	
		q2.fromAngleAxis ( p.y*m_twist_amt.y / m_twist_size.y * DEGtoRAD, Vector3DF(0, 1, 0));	
		q3.fromAngleAxis ( p.z*m_twist_amt.z / m_twist_size.z * DEGtoRAD, Vector3DF(0, 0, 1));	
		q1 = q3*q2*q1;

		p *= q1;
		q = m_rot[n] * q1;

		m_pos[n] = p + m_twist_ctr;
		m_rot[n] = q;
	}
}

void Sample::DeformFold ()
{
	Vector3DF p;
	Vector3DF dp;
	Vector3DF u;
	Quaternion q, q1, q2, q3, qx;

	// Folding is a constant rotation that is neg/pos on either side of the axis,
	// where the angle is same on both sides
	//    \    /
	//     \  /       looking straight down x-axis
	//     a\/a
	//  <---0----> z-axis     0 = origin
    //
	q1.fromAngleAxis( m_fold_amt.x * DEGtoRAD, Vector3DF(1, 0, 0));	
	q2.fromAngleAxis( m_fold_amt.y * DEGtoRAD, Vector3DF(0, 1, 0));	
	q3.fromAngleAxis( m_fold_amt.z * DEGtoRAD, Vector3DF(0, 0, 1));	

	for (int n = 0; n < m_pos.size(); n++) {

		p = m_pos[n] - m_fold_ctr;
		
		qx =  (p.z==0) ? q1.identity() : (p.z < 0) ? q1 : q1.conjugate();		// quaternion conjugate = negative rotation
		qx *= (p.x==0) ? q2.identity() : (p.x > 0) ? q2 : q2.conjugate();
		qx *= (p.x==0) ? q3.identity() : (p.x > 0) ? q3 : q3.conjugate();
		p *= qx;
		q = m_rot[n] * qx;

		m_pos[n] = p + m_fold_ctr;
		m_rot[n] = q;
	}
}

void Sample::display ()
{	
	Vector3DF pnt;
	Vector4DF clr;

	int w = getWidth();
	int h = getHeight();

	clearGL();

	start2D();
		setview2D(getWidth(), getHeight());
		drawText(10, 20, "Input:", 1,1,1,1);
		drawText(10, 35, "  Orbit cam  Right-mouse drag", 1,1,1,1);
		drawText(10, 50, "  Deform     Left-mouse drag", 1,1,1,1);
		drawText(10, 65, "  0          Fold", 1,1,1,1);				
		drawText(10, 80, "  1          Twist", 1,1,1,1);				
		drawText(10, 95, "  2          Bend", 1,1,1,1);
	end2D();


	start3D(m_cam);

		drawGrid();
				
		drawPnts();					// draw input

		StartOutput ();				// start deform
		DeformFold();				// fold
		DeformTwist();				// twist
		DeformBend();				// bend

		drawResult();				// draw output

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
		
		switch ( m_mode) {
		case 0:	m_fold_amt.x += dx*0.1f;		break;
		case 1:	m_twist_amt.x -= dx*0.1f;		break;
		case 2: m_bend_amt.x += dx*0.1f;		break;
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
	case '1':	m_mode = 0; break;
	case '2':	m_mode = 1; break;
	case '3':	m_mode = 2; break;
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
	appStart ( "Deformations", "Deformations", w, h, 4, 2, 16, false );
}

void Sample::shutdown()
{
}

