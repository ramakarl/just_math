//--------------------------------------------------------
// JUST MATH:
// Surfaces
//
//
//--------------------------------------------------------

//--------------------------------------------------------------------------------
// Copyright 2019-2023 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
//
// * Derivative works may append below the copyright notices but should not remove or modify earlier notices.
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
#include "gxlib.h"		// rendering
using namespace glib;

#include "mersenne.h"
#include <GL/glew.h>

#define DMAX    512

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

	bool		intersect_ray_patch (Vec3F rpos, Vec3F rdir, Vec3F q00, Vec3F q10, Vec3F q11, Vec3F q01, float& t, Vec3F& n ) ;

	Vec3F	  getPoint ( float u, float v, Vec3F& norm, Vec4F& clr );
  Vec3F	  getSurf (float u, float v, Vec3F& norm, Vec4F& clr, float& e);
  void    ConstructSurface();
  void    RecomputeSurface();
	void		DrawPatch ();		
  void    Optimize();

	void		Resize ();
	void		DrawGrid ();	
	void		UpdateCamera();
	void		MoveCamera ( char t, Vec3F del );

	Vec3F	mP[3];
  Vec3F mN[3];
  Vec3F mC[3];


  float mD[DMAX][DMAX];
  float mDu[DMAX][DMAX];
  float mDv[DMAX][DMAX];
  int   mDR;

	Camera3D*	m_cam;			// camera		
  bool    m_show[10];
	bool		m_run;
  int     m_func;
  int     m_sel;
	int			mouse_down;
	Mersenne	m_rand;

};
Sample obj;


bool Sample::init()
{
	addSearchPath(ASSET_PATH);

	init2D( "arial" );
	setTextSz ( 16 );

	m_run = true;
  m_show[1] = true;
  m_show[2] = true;
  m_show[3] = true;
  m_show[4] = true;
  m_func = 4;
  m_sel = 0;

	m_rand.seed(12);

	// create a camera
	m_cam = new Camera3D;								
	m_cam->setFov ( 60 );
	m_cam->setAspect ( 1 );	
	m_cam->SetOrbit ( -20, 30, 0, Vec3F(1, 0, 1), 5, 1 );	
	Resize ();		
	UpdateCamera();

  ConstructSurface();
  RecomputeSurface ();

	return true;
}

void Sample::Resize ()
{
}

void Sample::DrawGrid ()
{
	for (int i = -10; i <= 10; i++) {
		drawLine3D( Vec3F(float(i),-0.01f, -10.f), Vec3F(float(i), -0.01f, 10.f), Vec4F(.2f, .2f, .2f, 1.f) );
		drawLine3D( Vec3F(-10.f,	-0.01f, float(i)), Vec3F(10.f, -0.01f, float(i)), Vec4F(.2f, .2f, .2f, 1.f) );
	}	
	drawLine3D ( Vec3F(0,0,0), Vec3F(1,0,0), Vec4F(1,0,0,1) );
	drawLine3D ( Vec3F(0,0,0), Vec3F(0,1,0), Vec4F(0,1,0,1) );
	drawLine3D ( Vec3F(0,0,0), Vec3F(0,0,1), Vec4F(0,0,1,1) );

}

void Sample::ConstructSurface ()
{
  mP[0] = Vec3F(0, 0, 0);
  mP[1] = Vec3F(2, 1, 0);
  mP[2] = Vec3F(0, 0, 2);

  mN[0] = Vec3F(-1, .5, 0).Normalize();
  mN[1] = Vec3F(1, 1, 0).Normalize();
  mN[2] = Vec3F(0, 1, 1).Normalize();

  mC[0] = Vec3F(1, 1, 0);
  mC[1] = Vec3F(1, 0, 0);
  mC[2] = Vec3F(0, 1, 0);
}

void Sample::RecomputeSurface ()
{
  float u,v;
  Vec3F n; Vec4F clr;

  mDR = 64;
 
  Vec3F Ng;

  // known:
  //   N'-Ng=Du(N'.Pv)-Dv(N'.Pu)
  // we want the displaced surface to change rapidly
  // when the interpolated normal changes rapidly.
  // 
  Vec3F Nu = mN[1] - mN[0];
  Vec3F Nv = mN[2] - mN[0];
  Vec3F Pu = mP[1] - mP[0];
  Vec3F Pv = mP[2] - mP[0];
  //Vec3F Ni = mN[0] + Nu * u + Nv * v;	n.Normalize();
  //float cu = 1.0 / mDR;
  //float cv = 1.0 / mDR;  

  float du, dv;
  float a, b, c;
  float mu, mv;  

  // define  
  for (int i=0; i < mDR; i++) {
    for (int j = 0; j < mDR; j++) {

      u = float(i)/mDR;
      v = float(j)/mDR;
      getPoint(u, v, n, clr);

      
      Ng = (mN[2]-mN[0]).Cross (mN[1]-mN[0]).Normalize();      
      du = n.Dot (Ng);
      dv = (n - Ng).Length();      

      // pretty good - approximate least-squares average of vertex corner contributions
      mu = 1 - u;
      mv = 1 - v;
      a = -n.Dot(mP[1] - mP[0]) * u - n.Dot(mP[2] - mP[0]) * v;
      b = -n.Dot(mP[0] - mP[1]) * (1 - u) - n.Dot(mP[2] - mP[0]) * v;
      c = -n.Dot(mP[1] - mP[0]) * u - n.Dot(mP[0] - mP[2]) * (1 - v);

      switch (m_func) {
      case 0: mD[i][j] = 1.0f;     break;
      case 1: mD[i][j] = dv * dv;     break;
      case 2: mD[i][j] = pow(du, 1/4.0);    break;
      case 3: mD[i][j] = pow(du, dv*dv);    break;
      case 4: mD[i][j] = 1.0 + (a + b + c)/4.0;  break;
      //case 4: mD[i][j] = b;  break;
      };
    }
  }
  
}

Vec3F Sample::getPoint ( float u, float v, Vec3F& n, Vec4F& c )
{
	Vec3F p;
	p = mP[0] + (mP[1] - mP[0])*u + (mP[2] - mP[0])*v;
  n = mN[0] + (mN[1] - mN[0])*u + (mN[2] - mN[0])*v;	n.Normalize();  
  c = Vec4F(mC[0] + (mC[1] - mC[0]) * u + (mC[2] - mC[0]) * v, 1);
  
	return p;
}
Vec3F Sample::getSurf(float u, float v, Vec3F& ns, Vec4F& c, float& e)
{
  Vec3F Pu, Pv, Nu, Nv;
  Pu = mP[1]-mP[0]; 
  Pv = mP[2]-mP[0];
  Nu = mN[1]-mN[0];
  Nv = mN[2]-mN[0];

  Vec3F p;
  p = mP[0] + Pu * u + Pv * v;
  Vec3F n = mN[0] + Nu * u + Nv * v;	n.Normalize();
  
  int i = u*mDR;
  int j = v*mDR;
  Vec3F S = p + n * mD[i][j];

  float cu = 1.0/mDR;
  float cv = 1.0/mDR;

  float Du = mD[i+1][j];
  float Dv = mD[i][j+1];
  
  Vec3F SuA, SuB;
  p = mP[0] + Pu * (u + cu) + Pv * v;
  n = mN[0] + Nu * (u + cu) + Nv * v; n.Normalize();
  SuA = p + n * mD[i+1][j];
  p = mP[0] + Pu * (u - cu) + Pv * v;
  n = mN[0] + Nu * (u - cu) + Nv * v; n.Normalize();
  SuB = p + n * mD[i-1][j];

  Vec3F SvA, SvB;
  p = mP[0] + Pu * u + Pv * (v + cv);
  n = mN[0] + Nu * u + Nv * (v + cv); n.Normalize();
  SvA = p + n * mD[i][j+1];
  p = mP[0] + Pu * u + Pv * (v - cv);
  n = mN[0] + Nu * u + Nv * (v - cv); n.Normalize();
  SvB = p + n * mD[i][j-1];

  ns = (SvB-SvA).Cross ( SuB - SuA).Normalize();

  //e = 1.0 - ns.Dot( n );
e = fmin(1, (n - ns).Length() );
  c = Vec4F(e, 0, 0.5-e*0.5, 0.5);

  return S;
}

void Sample::DrawPatch ()
{
  Vec3F p[4], n[4];
  Vec4F c[4];
  float u, v;
  float du = 1.0 / mDR;
  float dv = 1.0 / mDR;

  // draw triangle normals
  if (m_show[2]) {  
    for (u=0; u <= 1; u+=0.1) {
      for (v = 0; v <= 1; v += 0.1) {
        p[0] = getPoint(u, v, n[0], c[0]);
        if (u + v <= 1.05) 
          drawLine3D(p[0], p[0] +n[0] * 2.0f, c[0]);
      }
    }
  }
  float e;  

  // draw triangle 
  if (m_show[1]) {
    p[0] = getPoint(0, 0, n[0], c[0]);
    p[1] = getPoint(1, 0, n[1], c[1]);
    p[2] = getPoint(0, 1, n[2], c[2]);
    drawLine3D(p[0], p[1], Vec4F(1, 1, 0, 1));
    drawLine3D(p[1], p[2], Vec4F(1, 1, 0, 1));
    drawLine3D(p[2], p[0], Vec4F(1, 1, 0, 1));
  }

  // draw selected
  drawLine3D(p[m_sel], p[m_sel] + n[m_sel] * 2.0f, Vec4F(1, 1, 1, 1));


  // draw triangle shading
  float ox = 3;
  if (m_show[1]) {
    for (int i = 1; i < mDR - 1; i++) {
      for (int j = 1; j < mDR - 1; j++) {
        u = float(i) / mDR; v = float(j) / mDR;
        if (u + v <= 1) {
          p[0] = getPoint(u, v, n[0], c[0]);
          p[1] = getPoint(u + du, v, n[1], c[0]);
          p[2] = getPoint(u, v + dv, n[2], c[0]);
          p[3] = getPoint(u + du, v + dv, n[3], c[0]);
          p[0].x += ox; p[1].x += ox; p[2].x += ox; p[3].x += ox;
          drawTri3D(p[0], p[1], p[2], n[0], Vec4F(1, 1, 1, 1), false);
          drawTri3D(p[1], p[3], p[2], n[0], Vec4F(1, 1, 1, 1), false);
        }
      }
    }
  }

  // draw surface   
  if (m_show[3]) {    
    for (int i=1; i < mDR-1; i++) {
      for (int j=1; j < mDR-1; j++) {
        u = float(i)/mDR; v = float(j)/mDR;      
        if( u+v<=1 ) {
          p[0] = getSurf (u, v, n[0], c[0], e);
          p[1] = getSurf (u+du, v, n[1], c[0], e);
          p[2] = getSurf (u, v+dv, n[2], c[0], e);
          p[3] = getSurf (u+du, v + dv, n[3], c[0], e);
          drawTri3D ( p[0], p[1], p[2], n[0], Vec4F(1,1,1,1), false );
          drawTri3D ( p[1], p[3], p[2], n[0], Vec4F(1, 1, 1, 1), false);
         }
      }
    }
  }

  // draw normals & errors
  if (m_show[4]) {
    for (int i = 1; i < mDR - 1; i++) {
      for (int j = 1; j < mDR - 1; j++) {
        u = float(i) / mDR; v = float(j) / mDR;
        if (u + v <= 1) {
          p[0] = getSurf(u, v, n[0], c[0], e);
          drawLine3D(p[0], p[0] + n[0] * 0.4f, c[0]);          
        }
      }
    }

  }
  
  //dbgprintf ( "eave: %f, emin: %f, emax: %f\n", eave, emin, emax);
}

void Sample::Optimize()
{
  Vec3F p, ns, ni, ne;
  Vec4F c;
  float e;
  float u, v;


  start2D(getWidth(),getHeight());  

  // draw displaced surface
  if (m_show[4]) {
    for (int i = 1; i < mDR - 1; i++) {
      for (int j = 1; j < mDR - 1; j++) {
        u = float(i) / mDR;
        v = float(j) / mDR;
        if (u+v > 1 ) continue;

        getPoint(u, v, ni, c);
        p = getSurf(u, v, ns, c, e);
        ne = ni- ns;      
        c = Vec4F(e, 0, 1-e, 1);      
        drawFill(Vec2F(i, j)*4, Vec2F(i + 1, j + 1)*4, c);      
      }
    }
    
  }

  // get error
  float eave = 0;
  float ecnt = 0;
  for (int i = 1; i < mDR - 1; i++) {
    for (int j = 1; j < mDR - 1; j++) {
      u = float(i) / mDR;
      v = float(j) / mDR;
      if (u + v > 1) continue;
      p = getSurf(u, v, ns, c, e);
      eave += e;
      ecnt++;
    }
  }
  eave /= float(ecnt);
  std::string fstr;
  char msg[200];  
  switch (m_func) {
  case 0: fstr = "n"; break;
  case 1: fstr = "(n-ng)^2"; break;
  case 2: fstr = "(n.ng)^1/4"; break;
  case 3: fstr = "(n.ng)^(n-ng)^2"; break;
  case 4: fstr = "-n.(P1-P0)u - n.(P2-P0)v - b - c"; break;
  }
  sprintf (msg,"Function: %s", fstr.c_str() );
  drawText(Vec2F(10, 300), msg, Vec4F(1, 1, 1, 1));
  sprintf(msg, "Average error: %f", eave);
  drawText(Vec2F(10, 320), msg, Vec4F(1, 1, 1, 1)); 



  end2D();  
 }


void Sample::display()
{

	clearGL();

	// Draw grid
	start3D(m_cam);	
  setLight3D( Vec3F(10, 10, 10), Vec4F(1,1,1,1));
  setMaterial( Vec3F(0,0,0), Vec3F(.7,.7,.7), Vec3F(.9,.9,0.7), 100, 1 );

		DrawGrid();	
	
		DrawPatch ();
	end3D();

  Optimize();

	drawAll();

	appPostRedisplay();								// Post redisplay since simulation is continuous
}


void Sample::UpdateCamera()
{
	Vec3F a, t;
	a = m_cam->getAng();
	t = m_cam->getToPos();
	//dbgprintf ( "angs: %3.4f %3.4f %3.4f, to: %3.4f %3.4f %3.4f\n", a.x,a.y,a.z, t.x,t.y,t.z );

	appPostRedisplay();		// update display
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
	float fine = 0.5;

	switch (mouse_down) {
	case AppEnum::BUTTON_LEFT: {
    mN[m_sel].x -= dx*0.01;   
    mN[m_sel].z -= dy*0.01;  
    RecomputeSurface();
	} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
		m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
		UpdateCamera();
	} break;

	case AppEnum::BUTTON_RIGHT: {

		// Adjust camera orbit 
		Vec3F angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam->SetOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());		
		m_cam->SetOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());				
		UpdateCamera();
		
		Vec3F to = m_cam->getToPos();

		//dbgprintf ( "cam: angs %f,%f,%f  to %f,%f,%f  dist %f\n", angs.x, angs.y, angs.z, to.x, to.y, to.z, m_cam->getOrbitDist() );
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag

	if ( mouse_down==AppEnum::BUTTON_LEFT) {
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

	m_cam->SetOrbit( m_cam->getAng(), m_cam->getToPos(), dist, dolly);
	m_cam->SetOrbit( m_cam->getAng(), m_cam->getToPos(), dist, dolly);

	UpdateCamera();
}


void Sample::MoveCamera ( char t, Vec3F del )
{
	switch (t) {	
	case 'p': {
		float orbit = m_cam->getOrbitDist() - del.z;
		m_cam->SetOrbit( m_cam->getAng(), m_cam->getToPos(), orbit, m_cam->getDolly());
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
	case '1':   m_show[1] = !m_show[1];   break;
  case '2':   m_show[2] = !m_show[2];   break;
  case '3':   m_show[3] = !m_show[3];   break;
  case '4':   m_show[4] = !m_show[4];   break;		
  case ',': 
    if (--m_func < 0 ) m_func = 4;  
    RecomputeSurface();
    break;
  case '.': 
    if (++m_func > 4) m_func = 0; 
    RecomputeSurface();
    break;
  case '[': if (--m_sel < 0) m_sel = 2; break;
  case ']': if (++m_sel > 2) m_sel = 0; break;  
	case 'a': case 'A':		MoveCamera('t', Vec3F(-s, 0, 0));	break;	
	case 'd': case 'D':		MoveCamera('t', Vec3F(+s, 0, 0));	break;
	case 'q': case 'Q':		MoveCamera('t', Vec3F(0, -s, 0));	break;
	case 'z': case 'Z':		MoveCamera('t', Vec3F(0, +s, 0));	break;
	};
}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

	m_cam->setSize( w, h );
	m_cam->setAspect(float(w) / float(h));
	m_cam->SetOrbit( m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());	

	appPostRedisplay();
}

void Sample::startup()
{
	int w = 2048, h = 2048;
	appStart("Displacement Map Sphere", "Displacement Map Sphere", w, h, 4, 2, 8, false);
}

void Sample::shutdown()
{
}




