//--------------------------------------------------------
// JUST MATH:
// Quaternion Trajectories 
// 
// A trajectory is defined here as a path through space that changes in both position and orientation,
// with the example here of ring that translates through space while also rotating. 
// The path left by the ring defines a 'rope', a lofted surface.
// Quaternion trajectory is a variety of techniques to create smooth orientated trajectories.
//
// Position may be interpolated differently that orientation. This gives a variety of combinations 
// for creating trajectories. Those implemented here are:
//   LinearSlerp -  Linear interpolated position (LERP) with, Spherical-linear or Squad interpolated orientation 
//   BSplineSlerp - B-Spline interpolated position (BSp.) with, Spherical-linear or Squad interpolated orientation 
//   CatmullRomSlerp - Catmull-Rom interpolated pos (CatRom) with, Spherical-linear or Squad interpolated orientation 
//   BezierSlerp -  Bezier interpolate position (Bez.) with, Spherical-linear or Squad interpolated orientation 
// 
// Generally, the interpolator used for position is much more noticable than that used for orientation.
// The orientation can be interpolated with either SLERP (sph.linear) or SQUAD (sph.cubic), by pressing 's' key.
// In this demo a lofted rope is created from an underlying set of randomized keys.
// Other demo options are describe here:
//   'g' - Regenerates with a different set of keys, oriented mostly upward.
//   'r' - Regenerate keys with fully randomized orientations.
//   'a' - Aligns each keys toward the next, which reduces 'kinks' in the rope.
//   's' - Switch between SLERP or SQUAD interp for orientation
//   't' - Optional tapering reduces the size of the ring as it moves along the trajectory.
//  <,>  - Change the number of underlying keys.
//  0..6 - Change the degree of the B-Spline
//  -,+  - Adjust the tension of the Catmull-Rom
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
//-------------------------------------------------

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
	void		Regenerate(bool bNewKeys);
	void		AlignKeys();
	int			FindKey( float t, float& u);

	void		drawGrid();
	void		drawOrientedCircle(Quaternion& x, Vector3DF p, float r, float m);

	void		drawKeys();
	void		drawLinearSlerp(float dt);
	void		drawBSplineSlerp(float dt);	
	void		drawCatmullRomSlerp(float dt);
	void		drawBezierSlerp(float dt);

	void		BSpline_prepare(Vector3DF* keys, int num_keys, int*& knots, Vector3DF*& d, int degree);
	Vector3DF	BSpline (int k, float u, Vector3DF* keys, int num_keys, int* knots, Vector3DF* d, int degree);
	
	void		CatmullRom_prepare(Vector3DF* keys, int num_keys, Vector3DF*& mids, float tension);
	Vector3DF	CatmullRom (int k, float u, Vector3DF* keys, int num_keys, Vector3DF* mids );
	
	void		Bezier_prepare (Vector3DF* keys, int num_keys, Vector3DF*& tans);
	Vector3DF	Bezier(int k, float u, Vector3DF* keys, int num_keys, Vector3DF* tans );

	Quaternion	Squad (int k, float u, Quaternion* keys, int num_keys, int stride=0);

	int			mNumKeys;
	float		mKeyTime[100];		// input keys
	Quaternion	mKeyRot[100];			
	Vector3DF	mKeyPnt[100];
	Mersenne	mt;

	bool		m_taper;
	int			m_degree;
	float		m_tension;
	bool		m_veryrandom;
	bool		m_squad;
	
	int*		m_knots;
	Vector3DF*	m_ktmp;
	Vector3DF*	m_mids;
	Vector3DF*  m_tans;

	Camera3D*	m_cam;
	bool		m_run;
	int			mouse_down;
};

Sample obj;


bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width & height
	
	addSearchPath ( ASSET_PATH );
	init2D ( "arial" );
	setview2D ( w, h );	
	setText ( 16, 1 );			

	m_cam = new Camera3D;
	m_cam->setFov ( 70 );
	m_cam->setOrbit ( Vector3DF(20,30,0), Vector3DF(0,20,0), 150, 1 );

	mt.seed(124);

	m_knots = 0;			// temp buffers
	m_ktmp = 0;
	m_mids = 0;
	m_tans = 0;
	m_run = true;
	m_taper = false;
	m_squad = false;
	m_veryrandom = false;
	
	mNumKeys = 10;			// default # keys
	m_degree = 3;			// default B-Spline degree
	m_tension = 0.5;		// default Catmull-Rom tension

	Regenerate (true);


	return true;
}

void Sample::Regenerate(bool bNewKeys)
{
	if ( bNewKeys ) GenerateKeys( mNumKeys);

	BSpline_prepare(mKeyPnt, mNumKeys, m_knots, m_ktmp, m_degree);

	CatmullRom_prepare(mKeyPnt, mNumKeys, m_mids, m_tension);

	Bezier_prepare(mKeyPnt, mNumKeys, m_tans);
}


void Sample::AlignKeys ()
{
	Quaternion q;
	Vector3DF tan, dir, a, b;
	float dot;
	for (int n = 0; n < mNumKeys-1; n++) {
			
		// -- different method for aligning keys. aim somewhat before next key.
		/*a = mKeyPnt[n + 2] - mKeyPnt[n + 1];	a.Normalize();
		b = mKeyPnt[n + 1] - mKeyPnt[n];		b.Normalize();
		dot = a.Dot( b );
		tan = mKeyPnt[n+1] - (mKeyPnt[n+2] - mKeyPnt[n])*(1-fabs(dot))*0.2f;
		dir = tan - mKeyPnt[n]; dir.Normalize(); */
		
		// simplest method. aim at next key
		dir = mKeyPnt[n+1] - mKeyPnt[n]; dir.Normalize();			

		q.rotationFromTo ( Vector3DF(0,1,0), dir ); q.normalize();

		mKeyRot[n] = q;
	}

	Regenerate(false);

}

void Sample::GenerateKeys (int k)
{
	Vector3DF p;
	Quaternion q;
	float a;

	// Generate keys
	mNumKeys = k;

	float dk = 1.0 / (mNumKeys-1);

	for (int n = 0; n < mNumKeys; n++) {

		// random orientation
		if ( m_veryrandom ) {
			q = mt.randQ(); q.normalize();						// complete random orientation
		} else {
			p = mt.randV3(-1, 1); p.y = 0.5; p.Normalize();		// semi-random (mostly upward)
			a = mt.randF();
			q.fromAngleAxis( a, p ); q.normalize();
		}
		
		// random positions
 		p = mt.randV3(-4, 4); 
		p.y = n * 50.0f * dk;		

		mKeyTime[n] = (n + mt.randF())*dk * 2;			// random variation in timing
		mKeyRot[n] = q;
		mKeyPnt[n] = p;
	}
}

// Uniform B-Spline
// smooth interpolation with arbitrary degree polynomial
// 
// Example: Cubic B-Spline of degree 3
// b3(t) = { 1/6t^3,									0 <= t <=1		Bmtx = 1/6 [0  0  0  1  
//           1/6(-3(t-1)^3 + 3(t-1)^2 + 3(t-1) + 1),	1 <= t <= 2                 1  3  3 -3  
//           1/6( 3(t-2)^3 - 6(t-2)^2 + 4),				2 <= t <= 3                 4  0 -6  3  
//			 1/6(-1(t-3)^3 + 3(t-3)^2 - 3(t-3) + 1),    3 <= t <= 4 }               1 -3  3 -1] 
// T = [1 t t^2 t^3]
//
// p(t) = <Pj, Pj+1, Pj+2, Pj+3> Bmtx T u     where u = t - int(t),  P = input points, Bmtx = B-Spline Basis Mtx, T = powers of t
//

void Sample::BSpline_prepare(Vector3DF* keys, int num_keys, int*& knots, Vector3DF*& d, int degree)
{
	int p = degree;

	if ( knots != 0x0) free (knots);
	if ( d != 0x0) free (d);
	knots = (int*) malloc( (num_keys + p*2) * sizeof(int));
	d = (Vector3DF*) malloc( (p + 1) * sizeof(Vector3DF));

	// knot vector
	for (int n = 0; n < num_keys; n++) {
		knots[n + p] = n;
	}
	for (int n = 0; n < p; n++) {
		knots[n] = 0;							// left padding
		knots[n + p+num_keys] = num_keys-1;		// right padding
	}
}

Vector3DF Sample::BSpline(int k, float u, Vector3DF* keys, int num_keys, int* knots, Vector3DF* d, int degree)
{
	float a;
	int p = degree;
	int ki = k + p;		// key offset by knot index by p

	// De Boor's algorithm, 1972, On Calculating with B-Splines, also see Wikipedia pseudo-code
	int i;
	for (int j = 0; j <= p; j++) {
		i = j + ki - p - 1; i = (i<0) ? 0 : ((i >= num_keys) ? num_keys-1 : i);
		d[j] = keys[i];
	}
	for (int r = 1; r <= p; r++) {
		for (int j = p; j >= r; j--) {
			a = ( (k+u) - knots[j+ki-p]) / (knots[j+1+ki-r] - knots[j+ki-p]);
			d[j] = d[j - 1] * (1 - a) + d[j] * a;
		}
	}
	return d[p];
}


// Catmull-Rom Spline
// faster piecewise cubic spline
// 
// BZ(t) = { 2t^3 - 3t^2 + 1,				0 <= t <=1		Bmtx =     [1  0  -3  2
//            t^3 - 2t^2 + t,								            0  1  -2  1  
//          -2t^3 + 3t^2,										        0 -2   3  0  
//			  t^3 -  t^2 }									            0  0  -1  1] 
// T = [1 t t^2 t^3]
//
// p(t) = <Pj, Pj+1, Pj+2, Pj+3> Bmtx T u     where u = t - int(t),  P = input points, Bmtx = Basis Mtx, T = powers of t
//
void Sample::CatmullRom_prepare (Vector3DF* keys, int num_keys, Vector3DF*& mids, float tension )
{
	if (mids != 0x0) free(mids);
	mids = (Vector3DF*) malloc ( num_keys * sizeof(Vector3DF) );

	// construct tangents
	mids[0] = (keys[2] - keys[0]) * tension;
	for (int n = 1; n < num_keys-1; n++) {
		mids[n] = (keys[n+1] - keys[n-1]) * tension;		// central difference
	}
	mids[num_keys-1] = (keys[num_keys-1] - keys[num_keys-3]) * tension;
}

Vector3DF Sample::CatmullRom (int k, float u, Vector3DF* keys, int num_keys, Vector3DF* mids)
{
	float u2 = u*u; float u3 = u2*u;
	
	float h00 = 2*u3 - 3*u2 +1;		// Hermite basis funcs
	float h10 = u3 - 2*u2 + u;
	float h01 = -2*u3 + 3*u2;
	float h11 = u3 - u2;
	Vector3DF kn1 = (k+1 >= num_keys) ? keys[ num_keys-1 ] : keys[k+1];

	Vector3DF p;
	p = keys[k]*h00 + mids[k]*h10 + kn1*h01 + mids[k+1]*h11;

	return p;
}

// Cubic Bezier
// 
// BZ(t) = { (1-t)^3,						0 <= t <=1		Bmtx = 1/6 [1  0  0  0
//           +3(1-t)^2*u,										       -3  3  0  0  
//           +3(1-t)*u^2,										        3 -6  3  0  
//			 t^3}											           -1  3 -3  1] 
// T = [1 t t^2 t^3]
//
// p(t) = <Pj, Pj+1, Pj+2, Pj+3> Bmtx T u     where u = t - int(t),  P = input points, Bmtx = Basis Mtx, T = powers of t
//
void Sample::Bezier_prepare(Vector3DF* keys, int num_keys, Vector3DF*& tans )
{
	if (tans != 0x0) free(tans);
	tans = (Vector3DF*) malloc(num_keys * sizeof(Vector3DF));

	// Construct tangents	
	// There are many ways to construct Bezier tangents.
	// The most natural is user-controlled, direct manipulation of tangents.
	// One automatic way is to central difference the control pnts. This ignores orientation of keys.
	// Another way is to use the normal of the key orientation. This ignores the curve shape.
	// Here we blend these two with varying amounts. 
	float normal_amt = 0.4f;
	float control_amt = 0.2f;

	Vector3DF v(0, 1.0, 0);  // normal axis of keys
	
	tans[0] = (keys[2] - keys[0]) * control_amt;
	for (int n = 1; n < num_keys - 1; n++) {
		tans[n] = v * mKeyRot[n] * normal_amt;						// normal of orientation, blended with..
		tans[n] += (keys[n + 1] - keys[n - 1]) * control_amt;		// central difference of control pnts
	}
	tans[num_keys - 1] = (keys[num_keys - 1] - keys[num_keys - 3]) * control_amt;
}

Vector3DF Sample::Bezier(int k, float u, Vector3DF* keys, int num_keys, Vector3DF* tans)
{
	float um = 1 - u;

	float b0 = um*um*um;
	float b1 = 3*u*um*um;
	float b2 = 3*u*u*um;
	Vector3DF kn1 = (k + 1 >= num_keys) ? keys[num_keys - 1] : keys[k + 1];

	Vector3DF p;
	p = keys[k] * b0 + (keys[k]+tans[k])* b1 + (kn1-tans[k])* b2 + kn1 * u*u*u;

	return p;
}



void Sample::drawGrid()
{
	int w = getWidth(), h = getHeight();
	for (int n = -50; n <= 50; n += 10 ) {
		drawLine3D ( n, 0, -50, n, 0, 50, 1, 1, 1, .7);
		drawLine3D( -50, 0, n, 50, 0, n, 1, 1, 1, .7);
	}
}

void Sample::drawKeys()
{
	Vector3DF p,q,r,s,v;
	Vector3DF drawoffs (-40,0,0);

	for (int n=0; n < mNumKeys;n++) {
		p = mKeyPnt[n] + drawoffs;
		q = Vector3DF(1,0,0); q *= mKeyRot[n]; q += p;
		r = Vector3DF(0,1, 0); r *= mKeyRot[n]; r += p;
		s = Vector3DF(0, 0,1); s *= mKeyRot[n]; s += p;
		drawLine3D ( p.x,p.y,p.z, q.x,q.y,q.z, 1,0,0,1);
		drawLine3D ( p.x, p.y, p.z, r.x, r.y, r.z, 0, 1, 0, 1);
		drawLine3D ( p.x, p.y, p.z, s.x, s.y, s.z, 0, 0, 1, 1);
		drawCircle3D ( p, r, 1.0, Vector4DF(1,1,0,1));
		if (n != mNumKeys-1) {
			v = mKeyPnt[n + 1] + drawoffs;
			drawLine3D (p.x,p.y,p.z, v.x, v.y, v.z, 1,1,0, 0.5);
		}
	}
	drawText3D(m_cam, drawoffs.x, drawoffs.y, drawoffs.z + 5, "Keys", 1, 1, 1, 1);
}

void Sample::drawOrientedCircle ( Quaternion& x, Vector3DF p, float radius, float m )
{
	Vector3DF q,r,s;
	x.normalize();
	r = Vector3DF(0, 1, 0); r *= x; r += p;
	Vector4DF clr = (m == -1) ? Vector4DF(1, 1, 1, 1) : ((m > 0.999 || m < 0.001) ? Vector4DF(1, 1, 0, 1) : Vector4DF(1, .5, 0, 1));
	drawCircle3D(p, r, radius, clr );
}

// search for key in time range
int Sample::FindKey( float t, float& u)
{
	// binary search 
	int low = 0;
	int hi = mNumKeys-1;
	int mid;
	if ( t < mKeyTime[low] ) {u=0; return low;}
	if ( t > mKeyTime[hi] ) {u=0; return hi; }
	while ( (hi-low) > 1 ) {	
		mid = (low + hi) >> 1;
		if ( t >= mKeyTime[mid] ) {
			low = mid;
		} else {		
			hi = mid;
		}
	} 
	
	// linear search
	/*int low = 0;
	for (low=0; mKeyTime[low] < t && low < mNumKeys;)
		low++;	
	low--;	 */
	
	u = (t - mKeyTime[low]) / (mKeyTime[low+1]-mKeyTime[low]); 	
	return low;
}

void Sample::drawLinearSlerp (float dt)
{
	Quaternion q;
	Vector3DF p;
	int k;
	float m, r, u;
	Vector3DF drawoffs (-20,0,0);

	float tend = mKeyTime[ mNumKeys-1 ];

	for (float t = 0; t < tend; t += dt) {

		k = FindKey( t, u );	

		// Linear Slerp 
		// def. Linear interlopation for position, Spherical linear for rotation
		// SPLIT(q,t) = {SLERP(q0,q1,m), LERP(t0,t1,u)}     see Haarbach, 2018, eqn. 25
		//
		p = mKeyPnt[k] + (mKeyPnt[k + 1] - mKeyPnt[k]) * u;	// linear translation

		if (m_squad)  q = Squad(k, u, mKeyRot, mNumKeys);				// squad rotation
		else		q = q.slerp( mKeyRot[k], mKeyRot[k + 1], u);		// slerp rotation
		
		p += drawoffs;										// shift over for display
		r = (m_taper) ? 1.0 - t / tend : 1.0;		// taper [optional]
		drawOrientedCircle( q, p, r, u );					// draw
	}
	drawText3D(m_cam, drawoffs.x, drawoffs.y, drawoffs.z+5, "LinearSlerp", 1, 1, 1, 1);
}

void Sample::drawBSplineSlerp (float dt)
{	
	Quaternion q;
	Vector3DF p;
	int n, k;
	float m, u, r;
	Vector3DF drawoffs (0,0,0);

	float tend = mKeyTime[ mNumKeys-1 ];

	for (float t = 0; t < tend; t += dt) {

		k = FindKey( t, u );

		// B-Spline Slerp
		// def. B-Spline interlopation for position, Spherical linear for rotation
		
		p = BSpline(k, u, mKeyPnt, mNumKeys, m_knots, m_ktmp, m_degree);		// B-spline translation

		if (m_squad)	q = Squad(k, u, mKeyRot, mNumKeys);				// squad rotation
		else			q = q.slerp(mKeyRot[k], mKeyRot[k + 1], u);		// slerp rotation
				
		p += drawoffs;											// shift over for display
		r = (m_taper) ? 1.0 - t / tend : 1.0;					// taper [optional]
		drawOrientedCircle(q, p, r, u);							// draw 
	}
	// draw control points - showing that the B-Spline does not pass thru its controls
	for (n=0; n < mNumKeys; n++) {							
		drawOrientedCircle(mKeyRot[n], mKeyPnt[n] + drawoffs, 0.2, -1);
	}
	char msg[256];
	sprintf ( msg, "SplineSlerp (deg %d)", m_degree );
	drawText3D ( m_cam, drawoffs.x, 0, 5, msg, 1,1,1,1);
}


void Sample::drawCatmullRomSlerp(float dt)
{
	Quaternion q;
	Vector3DF p;
	int n, k;
	float m, u, r;
	Vector3DF drawoffs(20, 0, 0);

	float tend = mKeyTime[ mNumKeys-1 ];

	for (float t = 0; t < tend; t += dt) {

		k = FindKey( t, u );

		// Catmull-Rom Slerp
		// def. Catmull-Rom interlopation for position, Spherical linear for rotation

		p = CatmullRom (k, u, mKeyPnt, mNumKeys, m_mids );			// catmull-rom translation

		if (m_squad)	q = Squad(k, u, mKeyRot, mNumKeys);				// squad rotation
		else			q = q.slerp(mKeyRot[k], mKeyRot[k + 1], u);		// slerp rotation

		p += drawoffs;											// shift over for display
		r = (m_taper) ? 1.0 - t / tend : 1.0;					// taper [optional]
		drawOrientedCircle(q, p, r, u);							// draw 
	}
	char msg[256];
	sprintf(msg, "CatmullRomSlerp");
	drawText3D(m_cam, drawoffs.x, 0, 5, msg, 1, 1, 1, 1);
}

void Sample::drawBezierSlerp (float dt)
{
	Quaternion q;
	Vector3DF p;
	int k;
	float m, u, r;
	Vector3DF drawoffs(40, 0, 0);

	float tend = mKeyTime[ mNumKeys-1 ];

	for (float t = 0; t < tend; t += dt) {

		k = FindKey( t, u );

		// Bezier Slerp
		// def. Bezier interpolation for position, Spherical linear for rotation

		p = Bezier(k, u, mKeyPnt, mNumKeys, m_tans );				// Bezier translation

		if (m_squad)  q = Squad(k, u, mKeyRot, mNumKeys);			// squad rotation
		else		q = q.slerp(mKeyRot[k], mKeyRot[k + 1], u);		// slerp rotation

		p += drawoffs;											// shift over for display
		r = (m_taper) ? 1.0 - t / tend : 1.0;					// taper [optional]
		drawOrientedCircle(q, p, r, u);							// draw 
	}
	// draw tangents
	for (k = 0; k < mNumKeys; k++) {
		drawLine3D ( mKeyPnt[k]-m_tans[k] + drawoffs, mKeyPnt[k]+m_tans[k] + drawoffs, Vector4DF(1,1,1,1) );
	}

	char msg[256];
	sprintf(msg, "BezierSlerp");
	drawText3D(m_cam, drawoffs.x, 0, 5, msg, 1, 1, 1, 1);
}


Quaternion Sample::Squad (int k, float u, Quaternion* keys, int num_keys, int stride )
{
	Quaternion q, qi, qi1, qi2, qim1, qim2;
	Quaternion si, si1;
	Quaternion t0, t1;

	// stride is optional if keys are part of a user-defined struct, e.g. sizeof(MyStruct)
	// otherwise we assume Quaternions keys are tightly packed
	if (stride==0) stride = sizeof(Quaternion);

	// SQUAD
	// Implemented by R.C.Hoetzlein (c) 2021, CC-BY
	// Based on paper: eqn 15, 2018, Harrbach et.al, Survey of Higher Order Rigid Body Motion Interpolation Methods for Keyframe Animation and Continuous-Time Trajectory Estimation	
	// *NOTE* In order for this to work, your Quaternion class must have the 
	// correct implementations of slerp, log, exp and inverse. No normalization is needed except the last one.
	//
	// def. SQUAD(q,s,q+1,s+1) = SLERP( SLERP(q,q+1,u), SLERP(s,s+1,u), 2u(1-i) )
	//      si = qi * exp( -(log(qi^-1 qi+1) + log(qi^-1 qi-1))/4 )

	qi = *(Quaternion*) ((char*) keys + k*stride);
	qi1 = *(Quaternion*) ((char*) keys + (k+1) * stride);
	qi2 = (k == mNumKeys - 2) ? q.identity() : *(Quaternion*)((char*)keys + (k + 2) * stride);
	qim1 = (k == 0) ? q.identity() : *(Quaternion*)((char*)keys + (k - 1) * stride);

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

	setLight(S3D, 100, 100, 100);

	clearGL();

	start3D(m_cam);
		setText(2, 1);

		drawGrid();

		drawKeys();

		float dt = 0.01;
		
		drawLinearSlerp (dt);

		drawBSplineSlerp (dt);

		drawCatmullRomSlerp (dt);

		drawBezierSlerp(dt);

	end3D();

	start2D();			// help
		setText ( 20, 1);
		drawText(10,10, "Press 'g' to regenerate.", 1, 1, 1, 1);
		drawText(10,30, "Press 'r' to regenerate (very random).", 1, 1, 1, 1);
		drawText(10,50, "Press 'a' to align keys.", 1, 1, 1, 1);
		drawText(10,70, "Press 's' to switch SLERP or SQUAD interp for orientation", 1, 1, 1, 1);
		drawText(10,90, "Press 't' for optional tapering.", 1, 1, 1, 1);
		drawText(10,110, "Press < > to change number of keys.", 1, 1, 1, 1);
		drawText(10,130, "Press 1,2,3,4,5,6 to change B-Spline degree.", 1, 1, 1, 1);
		drawText(10,150, "Press - + to adjust Catmull-Rom tension.", 1, 1, 1, 1);

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
	case 'g': case 'G':	m_veryrandom = false;	Regenerate(true); break;
	case 'r': case 'R':	m_veryrandom = true;	Regenerate(true); break;
	case '1':			m_degree = 1;			Regenerate(false);	break;
	case '2':			m_degree = 2;			Regenerate(false);	break;
	case '3':			m_degree = 3;			Regenerate(false);	break;
	case '4':			m_degree = 4;			Regenerate(false);	break;
	case '5':			m_degree = 5;			Regenerate(false);	break;
	case '6':			m_degree = 6;			Regenerate(false);	break;
	case 'a':			AlignKeys();			break;
	case 's':			m_squad = !m_squad;		break;
	case ' ':			m_run = !m_run;			break;
	case 't':			m_taper = !m_taper;		break;
	case '-': case '_':
		m_tension -= 0.05; if (m_tension <0 ) m_tension=0;	
		Regenerate ( false );
		break;
	case '=': case '+':
		m_tension += 0.05; if (m_tension > 1) m_tension =1; 
		Regenerate(false);
		break;
	case ',': case '<':
		mNumKeys--; if ( mNumKeys < 3 ) mNumKeys = 3;
		Regenerate(true);
		break;
	case '.': case '>':
		mNumKeys++;  if (mNumKeys > 100) mNumKeys = 100;
		Regenerate(true);
		break;
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
	appStart ( "Quaternion Trajectories", "Quaternion Trajectories", w, h, 4, 2, 16, false );
}

void Sample::shutdown()
{
}

