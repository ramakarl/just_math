//--------------------------------------------------------
// JUST MATH:
// Quat Squad - Spherical Cubic Spline Quadrangle
// 
// SQUAD, spherical cubic spline quadrangle, is a method for interpolating an orientation.
// Interpolation of orientations is often used for computing smooth trajectories,
// such as lofting curves, flight paths, etc.
// The typical SLERP, spherical linear interpolation, smoothly interpolates between any 
// two orientations, but does so linearly such that *changes* in orientation are abrupt (discontinuous).
// SQUAD overcomes this with C1 continunity for the interpolation through multiple orientations.
// Based on paper: eqn 15, 2018, Harrbach et.al, see Figure 4
//    "Survey of Higher Order Rigid Body Motion Interpolation Methods for Keyframe Animation and Continuous-Time Trajectory Estimation"
// The definition of SQUAD is given using three SLERPs:
//     def. SQUAD(q,s,q+1,s+1) = SLERP( SLERP(q,q+1,u), SLERP(s,s+1,u), 2u(1-i) )
//
// *BIG NOTE* In order for this to work, your Quaternion class *must* have the 
// correct implementations of slerp, log, exp and inverse. No normalization is needed except the last one.
//  (The originals in Irrlicht Engine by N.Gebhardt were incorrect, and fixed here in libmin/quaternions.cpp)
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
#include "vec.h"
#include "quaternion.h"
#include "mersenne.h"

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

	void		GenerateKeys(int k);

	void		drawGrid();
	void		drawOrientedBox(Quaternion& x, Vector3DF p);

	void		drawKeys(float t);
	void		drawSlerp(float t);
	void		drawSquad (float t);

	Quaternion	Squad (float t, Quaternion* keys, int num_keys, int stride=0);

	int			mNumKeys;
	Quaternion	mKeyRot[10];
	Vector3DF	mKeyPnt[10];
	Mersenne	mt;
	float		mTime;

	Camera3D*	m_cam;
	bool		m_run;
	int			mouse_down;
};

Sample obj;


bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width & height
	m_run = true;

	addSearchPath ( ASSET_PATH );
	init2D ( "arial_256" );
	setview2D ( w, h );	
	setText ( 16, 1 );			

	m_cam = new Camera3D;
	m_cam->setFov ( 70 );
	m_cam->setOrbit ( Vector3DF(20,30,0), Vector3DF(0,0,0), 200, 1 );

	mt.seed(124);

	GenerateKeys(5);

	mTime = 0;

	return true;
}

void Sample::GenerateKeys (int k)
{
	Vector3DF p;
	Quaternion q;
	float a;

	// Generate keys
	mNumKeys = k;

	for (int n = 0; n < mNumKeys; n++) {
		// random orientation
		//q = mt.randQ(); q.normalize();
		
		p = mt.randV3(-1, 1); p.y = 0.5; p.Normalize();
		a = mt.randF();
		q.fromAngleAxis( a, p ); q.normalize();

 		p = Vector3DF(0, n*4, 0);

		mKeyRot[n] = q;
		mKeyPnt[n] = p;
	}
}


void Sample::drawGrid()
{
	int w = getWidth(), h = getHeight();
	for (int n = -50; n <= 50; n += 10 ) {
		drawLine3D ( n, 0, -50, n, 0, 50, 1, 1, 1, .7);
		drawLine3D( -50, 0, n, 50, 0, n, 1, 1, 1, .7);
	}
}

void Sample::drawKeys(float t)
{
	Vector3DF p,q,r,s,v;
	for (int n=0; n < mNumKeys;n++) {
		p = mKeyPnt[n] - Vector3DF(20,0,0);
		q = Vector3DF(1,0,0); q *= mKeyRot[n]; q += p;
		r = Vector3DF(0,1, 0); r *= mKeyRot[n]; r += p;
		s = Vector3DF(0, 0,1); s *= mKeyRot[n]; s += p;
		drawLine3D ( p.x,p.y,p.z, q.x,q.y,q.z, 1,0,0,1);
		drawLine3D ( p.x, p.y, p.z, r.x, r.y, r.z, 0, 1, 0, 1);
		drawLine3D ( p.x, p.y, p.z, s.x, s.y, s.z, 0, 0, 1, 1);
		drawCircle3D ( p, r, 1.0, Vector4DF(1,1,0,1));
		if (n != mNumKeys-1) {
			v = mKeyPnt[n + 1] - Vector3DF(20,0,0);
			drawLine3D (p.x,p.y,p.z, v.x, v.y, v.z, 1,1,0, 0.5);
		}
	}

	int n;
	float m;

	n = int(t); m = t - n;
	p = mKeyPnt[n] + (mKeyPnt[n + 1] - mKeyPnt[n]) * m;	// linear interpolation of translation
	p += Vector3DF(-20,0,0);
	drawCircle3D(p, p + Vector3DF(0, 1, 0), 0.2, Vector4DF(1, 1, 1, 1));

}

void Sample::drawOrientedBox ( Quaternion& x, Vector3DF p )
{
	Vector3DF q,r,s;
	q = Vector3DF(5, 0, 0); q *= x; q += p;
	r = Vector3DF(0, 5, 0); r *= x; r += p;
	s = Vector3DF(0, 0, 5); s *= x; s += p;
	drawLine3D(p.x, p.y, p.z, q.x, q.y, q.z, 1, 0, 0, 1);
	drawLine3D(p.x, p.y, p.z, r.x, r.y, r.z, 0, 1, 0, 1);
	drawLine3D(p.x, p.y, p.z, s.x, s.y, s.z, 0, 0, 1, 1);
	drawCircle3D(p, r, 4.0, Vector4DF(1, 1, 0, 1));

	Matrix4F xform;
	xform.Translate( p.x, p.y, p.z);
	xform *= x.getMatrix();
	drawBox3DXform(Vector3DF(-1, -1, -1), Vector3DF(1, 1, 1), Vector4DF(1, 1, 1, 1), xform);
}

void Sample::drawSlerp( float t)
{
	Quaternion a, b, q, ql;
	Vector3DF p, r, g, h;
	int n;
	float m;

	// draw the current slerp orientation
	n = int(t); m = t - n;
	q = q.slerp( mKeyRot[n], mKeyRot[n+1], m);			// slerp at t	
	drawOrientedBox ( q, Vector3DF(0,15,0) );

	h = q.getAxis();  h.Normalize();  h *= 5.0;
	drawCircle3D( h, h+Vector3DF(0,1,0), 0.2, Vector4DF(1,1,1,1));

	// visualize slerp on the unit sphere
	for (float u=0; u < mNumKeys-1; u += 0.05 ) {
		
		// slerp
		n = int(u);			// integer key
		m = u - n;			// fractional parameter
		q = q.slerp ( mKeyRot[n], mKeyRot[n+1], m );	

		// draw on unit sphere
		if (u == 0) ql = q;
		g = ql.getAxis(); g.Normalize();  g *= 5.0;	
		h = q.getAxis();  h.Normalize();  h *= 5.0;	
		drawLine3D(g.x, g.y, g.z, h.x, h.y, h.z, 1, 1, 0, 1);

		ql = q;
	}
	// draw sphere itself
	drawSphere3D( Vector3DF(0,0,0), 4.8, Vector4DF(0.5,0.5,0.5,1));
}

void Sample::drawSquad (float t)
{	
	Quaternion q, ql;
	Vector3DF p, r, g, h;
	int n;
	float m, u;
		
	// draw the current squad orientation
	q = Squad(t, mKeyRot, mNumKeys);
	drawOrientedBox(q, Vector3DF(20, 15, 0));
		
	h = q.getAxis();  h.Normalize();  h *= 5.0;	h += Vector3DF(20, 0, 0);
	drawCircle3D(h, h + Vector3DF(0, 1, 0), 0.2, Vector4DF(1, 1, 1, 1));
	
	// visualize squad on the unit sphere
	for (float u = 0; u < mNumKeys - 1; u += 0.05) {
		
		// squad
		q = Squad( u, mKeyRot, mNumKeys );
		
		// draw on unit sphere
		if ( t==0 ) ql = q;
		g = ql.getAxis(); g.Normalize();	g *= 5.0;	g += Vector3DF(20,0,0);
		h = q.getAxis();  h.Normalize();  h *= 5.0;	h += Vector3DF(20, 0, 0);
		drawLine3D( g.x, g.y, g.z, h.x, h.y, h.z, 1,1,0,1 );

		ql = q;
	}
	// draw sphere itself
	drawSphere3D(Vector3DF(20, 0, 0), 4.8, Vector4DF(0.5, 0.5, 0.5, 1));
}


Quaternion Sample::Squad (float t, Quaternion* keys, int num_keys, int stride )
{
	Quaternion q, qi, qi1, qi2, qim1;
	Quaternion si, si1;
	Quaternion t0, t1;

	// stride is optional if keys are part of a user-defined struct, e.g. sizeof(MyStruct)
	// otherwise we assume Quaternions keys are tightly packed
	if (stride==0) stride = sizeof(Quaternion);

	// SQUAD
	// Implemented by R.C.Hoetzlein (c) 2021
	// Based on paper: eqn 15, 2018, Harrbach et.al, Survey of Higher Order Rigid Body Motion Interpolation Methods for Keyframe Animation and Continuous-Time Trajectory Estimation	
	// *NOTE* In order for this to work, your Quaternion class must have the 
	// correct implementations of slerp, log, exp and inverse. No normalization is needed except the last one.
	//
	// def. SQUAD(q,s,q+1,s+1) = SLERP( SLERP(q,q+1,u), SLERP(s,s+1,u), 2u(1-i) )
	//      si = qi * exp( -(log(qi^-1 qi+1) + log(qi^-1 qi-1))/4 )

	int n = int(t);			// integer key
	float u = t - n;		// fractional parameter

	qi = *(Quaternion*) ((char*) keys + n*stride);
	qi1 = *(Quaternion*) ((char*) keys + (n+1) * stride);
	qi2 = (n == mNumKeys - 2) ? q.identity() : *(Quaternion*)((char*)keys + (n + 2) * stride);
	qim1 = (n == 0) ? q.identity() : *(Quaternion*)((char*)keys + (n - 1) * stride);

	// compute si
	t0 = qi.inverse() * qi1;	t0 = t0.log();
	t1 = qi.inverse() * qim1;	t1 = t1.log();
	si = qi * ((t0 + t1) * -0.25f).exp();

	// compute si+1
	t0 = qi1.inverse() * qi2;	t0 = t0.log();
	t1 = qi1.inverse() * qi;	t1 = t1.log();
	si1 = qi1 * ((t0 + t1) * -0.25f).exp();

	t0 = q.slerp(qi, qi1, u);				// first inner slerp
	t1 = si.slerp(si, si1, u);				// second inner slerp
	q = q.slerp(t0, t1, 2 * u * (1 - u));	// final slerp		
	q.normalize();

	return q;
}


void Sample::display ()
{	
	int w = getWidth(); int h = getHeight();

	if (m_run) {
		mTime += 0.001; 
		if ( mTime >= mNumKeys-1 ) mTime = 0;
	}
	
	setLight(S3D, 100, 100, 100);

	clearGL();

	start3D(m_cam);

		drawGrid();

		drawKeys(mTime);
		
		drawSlerp(mTime);

		drawSquad(mTime);

	end3D();

	draw3D ();
	draw2D (); 	
	appPostRedisplay();								// Post redisplay since simulation is continuous
}



void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	int w = getWidth(), h = getHeight();				// window width & height

	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;
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



void Sample::motion(AppEnum button, int x, int y, int dx, int dy)
{
	// Get camera for scene
	bool shift = (getMods() & KMOD_SHIFT);		// Shift-key to modify light
	float fine = 0.5f;

	switch (mouse_down) {
	case AppEnum::BUTTON_LEFT: {
	
	} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		float zoom = (m_cam->getOrbitDist() - m_cam->getDolly()) * 0.0003f;
		m_cam->moveRelative(float(dx) * zoom, float(-dy) * zoom, 0);
	} break;
	case AppEnum::BUTTON_RIGHT: {
		// Adjust orbit angles
		Vector3DF angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam->setOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
	} break;

	}
}
void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action == AppEnum::BUTTON_RELEASE) return;

	switch (keycode) {
	case 'g': case 'G':	GenerateKeys(5); break;
	case ' ':	m_run = !m_run; break;
	};
}

void Sample::reshape (int w, int h)
{
	glViewport ( 0, 0, w, h );
	setview2D ( w, h );
		
	appPostRedisplay();	
}

void Sample::startup ()
{
	int w = 1800, h = 1000;
	appStart ( "Quaternion SQUAD", "Quaternion SQUAD", w, h, 4, 2, 16, false );
}

void Sample::shutdown()
{
}

