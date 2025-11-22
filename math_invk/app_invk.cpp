//--------------------------------------------------------
// JUST MATH:
// Invk - Inverse Kinematics 
// 
// This demo computes inverse kinematics in real-time for arbitrary joint configurations.
// Constructs the Jacobian and computes the Jacobian transpose. 
// For more details:
//   - Steven Rotenberg, Slides for "Inverse Kinematics (part 1 & 2)"
//   - Andreas Aristidou and Joan Lasenby, "Inverse Kinematics: a review of existing techniques and introduction of a new fast iterative solver."
// See joints.cpp for inverse kinematics code.
// This IK implementation uses Quaternion for simplicity. Quaternions allow for several benefits over Euler angles. 
// First, axis boundaries are greatly simplified as quaternions can interpolate thru two arbitrary vectors. 
// Second, IK requires incremental changes in angles which are well suited to quaternions. 
// Third, quaternions are more efficient to compute for certain operations.
// There are two drawbacks to using quaternions for inverse kinematics. Per-axis angle range limits are more 
// easily computed with Euler angles, so there is a conversion performed in the LimitQuaternion function to 
// handle this. Finally, care must be taken to normalize the quaternions frequently during calculations.
//
// In the demo, the end effector is the purple box. Move the end effector with mouse.
// The joint system is either robot or humanoid, press the 't' key to switch.
//   - Robot - simulates a pick-and-place type robot. 4x single axis motors. 4 DOF
//   - Human - simulates a humanoid right arm, with shoulder, elbow and wrist. 6 DOF
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
#include "geom_helper.h"
#include "gxlib.h"		// rendering
using namespace glib;

#include <GL/glew.h>
#include <algorithm>

#include "joints.h"

#define MOVE_GOAL_XY	1
#define MOVE_GOAL_XZ	2

class Sample : public Application {
public:
	virtual bool init();
	virtual void display();
	virtual void reshape(int w, int h);	
	virtual void keyboard(int keycode, AppEnum action, int mods, int x, int y);
	virtual void mouse (AppEnum button, AppEnum state, int mods, int x, int y);
	virtual void motion( AppEnum button, int x, int y, int dx, int dy);
	virtual void mousewheel(int delta);
	virtual void startup ();

	void		Reset ();

	int			mouse_down;
	int			mouse_action;
	int			m_adjust;
	int			m_type;
	bool		m_bRun, m_bStep;

	Camera3D*	m_cam;
	Vec3F	  m_goal;

	Joints		m_joints;	
};

// Application object
Sample sample_obj;			


void Sample::Reset ()
{
  dbgprintf("Model: %s, %s\n", (m_type == 0) ? "robot arm" : "human arm", m_bRun ? "sim" : "single step");

	// create joints
	m_joints.Clear ();

	switch (m_type) {
	case 0:					//---- Robot Arm		
		// AddJoint: name, length, angles, freedom x,y,z
		m_joints.AddJoint ( "J0", 1, Vec3F(0,0,0), 0, 1, 0 );		// base: Y-axis freedom
		m_joints.AddJoint ( "J1", 4, Vec3F(0,0,0), 1, 0, 0 );		// arm:  X-axis freedom
		m_joints.AddJoint ( "J2", 4, Vec3F(0,0,0), 1, 0, 0 );
		m_joints.AddJoint ( "J3", 2, Vec3F(0,0,0), 1, 0, 0 );	
		m_joints.SetLimits ( 1, Vec3F(-45, 0, 0), Vec3F(90, 0, 0) );	 
		m_joints.SetLimits ( 2, Vec3F(5, 0, 0), Vec3F(170, 0, 0) );	 
		m_joints.SetLimits ( 3, Vec3F(5, 0, 0), Vec3F(90, 0, 0) );	
		break;
	
	case 1:					//---- Human Arm			
		// J1) Shoulder
		//    Ball pivot: X-axis fwd, Y-axis along, Z-axis down
		//                             T-pose       Minimum   Maximum
		//    X-axis, swing up/down:  90=out right,  -5=over, 200=down, 0=straight up, 90=out,
		//    Y-axis, shoulder twist:  0=out front, -80=down, +90=up     ** +45 typical **
		//    Z-axis, swing fwd/back:  0=out right, -60=back, +130=forward		
		// J2) Elbow:
		//    Hinge jnt:  X-axis side (hinge), Y-axis along, Z-axis fwd
		//    X-axis, rotate:		   0=out,        +3=limit, +150=closed
		// J3) Wrist: 
		//    Ball pivot: X-axis side, Y-axis along, Z-axis fwd  (Z+ = thumb)
		//    X-axis, palm up/down:    0=out flat,  -80=palm extend, +80=palm down
		//    Y-axis, wrist twist:     0=thumb up,  -90=thumb fwd,   +60=thumb back
		//    Z-axis, wrist yaw:       0=out strt,  -25=cantor inwd, +25=cantor outwd

		// AddJoint: name, length, angles, freedom x,y,z
		m_joints.AddJoint ( "back",		5, Vec3F(0,0,0), 0, 0, 0 );					// fixed joint
		m_joints.AddJoint ( "shoulder", 4, Vec3F(90,0,0),1, 1, 1 );			
		m_joints.AddJoint ( "elbow",	3, Vec3F(0,0,0), 1, 0, 0 );
		m_joints.AddJoint ( "wrist",	1, Vec3F(0,0,0), 1, 0, 1 );	 
		m_joints.SetLimits ( 1, Vec3F(-90,  0, -60), Vec3F(200, 90, 130) );	 	// shoulder
		m_joints.SetLimits ( 2, Vec3F( 5,   0,  +5), Vec3F(150, 0,  0) );		// elbow
		m_joints.SetLimits ( 3, Vec3F( 0, -90, -25), Vec3F(+40,+60,+25) );		// wrist	
		break;
	}	

	m_joints.StartIK();

	m_goal.Set ( 2, 1, 4 );
}

bool Sample::init ()
{	
	int w = getWidth(), h = getHeight();			// window width & height
	
	addSearchPath(ASSET_PATH);
	init2D ( "arial" );
	setTextSz (16,1);
	
	glViewport ( 0, 0, w, h );

	m_cam = new Camera3D;
	m_cam->setNearFar ( 1, 2000 );
	m_cam->SetOrbit ( Vec3F(40,30,0), Vec3F(0,0,0), 20, 1 );
	m_adjust = -1;

	m_bRun = true;
	m_bStep = false;
	m_type = 0;		// robot

  // info display
  dbgprintf("\nInverse Kinematics with Quaternions (invk)\n");
  dbgprintf("by Rama Hoetzlein (c) 2018\n");
  dbgprintf(" Input:\n" );
  dbgprintf("   Orbit view      Right-click drag\n");
  dbgprintf("   Run             Spacebar.\n");
  dbgprintf("   Type of body    'T' key. Robot or human.\n");
  dbgprintf("   Move end X/Y    Left-click drag purple cube\n");
  dbgprintf("   Move end Z      Right-click drag purple cube\n");
  dbgprintf("   Single step     S key. Then move cube. Press S multiple times.\n\n");  
  
  // create joints
  Reset();


	return true;
}


void Sample::display()
{
	Vec3F a,b,c,p;
	int w = getWidth(), h = getHeight();

	clearGL();
	
	// run inverse kinematics
	if (m_bRun) 		
		m_joints.InverseKinematics ( m_goal );
	
	if (m_bStep) {
		m_bStep=false;
		m_joints.InverseKinematics ( m_goal, 1 );   // 1 step
	}

	// draw 3D
	start3D( m_cam );
	
		// draw joints
		m_joints.Sketch( m_cam );

		// sketch a grid
		for (int i = -10; i <= 10; i++) {
			drawLine3D( Vec3F(float(i),-0.01f, -10.f), Vec3F(float(i), -0.01f, 10.f), Vec4F(.2f, .2f, .2f, 1.f) );
			drawLine3D( Vec3F(-10.f,	-0.01f, float(i)), Vec3F( 10.f, -0.01f, float(i)), Vec4F(.2f, .2f, .2f, 1.f) );
		}	
		// world axes
		a = Vec3F(1,0,0); b = Vec3F(0,1,0); c = Vec3F(0,0,1); 
		p = Vec3F(-10.f,-0.01f,-10.f);
		drawLine3D ( p, p+a, Vec4F(1, 0, 0, 1) );
		drawLine3D ( p, p+b, Vec4F(0, 1, 0, 1) );
		drawLine3D ( p, p+c, Vec4F(0, 0, 1, 1) );
	
		// goal
		p = m_goal;
		drawBox3D ( Vec3F(p.x-0.1f, p.y-0.1f, p.z-0.1f), Vec3F(p.x+0.1f, p.y+0.1f, p.z+0.1f), Vec4F(1, 0, 1, 1) );
		drawBox3D ( Vec3F(p.x-0.1f, 0.f, p.z-0.1f), Vec3F(p.x+0.1f, 0.f, p.z+0.1f), Vec4F(.75f, 0, .75f, 1) );
		if ( mouse_action == MOVE_GOAL_XY )
			drawBox3D ( Vec3F(-10.f, 0.f, p.z-0.1f), Vec3F(10.f, 8.f, p.z+0.1f), Vec4F(1, 0, 1, 1) );
		if ( mouse_action == MOVE_GOAL_XZ )
			drawBox3D ( Vec3F(-10.f, p.y-0.1f, -10.f), Vec3F(10.f, p.y+0.1f, 10.f), Vec4F(1, 0, 1, 1) );

    // 3D cursor
    Vec3F dir = m_cam->inverseRay( getX(), getY(), w, h);
    Vec3F hit = intersectLinePlane(m_cam->getPos(), dir, Vec3F(0,0,0), Vec3F(0, 1, 0));
    drawCircle3D (hit, 0.05, Vec4F(1,1,1,1));

	end3D(); 

	// render in opengl
	drawAll();

	appPostRedisplay();								// Post redisplay since simulation is continuous
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
	appPostRedisplay();
}


void Sample::motion( AppEnum button, int x, int y, int dx, int dy) 
{
	// Get camera 
	bool shift = (getMods() & KMOD_SHIFT);		// Shift-key to modify light

	float fine = 0.5;

	switch ( mouse_down ) {	
	case AppEnum::BUTTON_LEFT: {

		if ( mouse_action == MOVE_GOAL_XZ ) {
			int w = getWidth(), h = getHeight();
			Vec3F dir = m_cam->inverseRay( x, y, w, h );
			Vec3F hit = intersectLinePlane ( m_cam->getPos(), dir, m_goal, Vec3F(0,1,0) );
			m_goal = hit;
		} else {
			if ( m_adjust != -1 ) {				
				bool bUseXAxis = (fabs(dy) > 2 );
				m_joints.MoveJoint ( m_adjust, bUseXAxis ? 0 : 1, bUseXAxis ? -dy*0.01f : dx*0.01f);
			}
		}
		appPostRedisplay();	// Update display
		} break;
	
	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam->moveRelative ( float(dx) * fine* m_cam->getOrbitDist()/1000, float(-dy) * fine* m_cam->getOrbitDist()/1000, 0 );
		appPostRedisplay();	// Update display
		} break;
	
	case AppEnum::BUTTON_RIGHT: {	

		if ( mouse_action == MOVE_GOAL_XY ) {
			int w = getWidth(), h = getHeight();			
			Vec3F dir = m_cam->inverseRay( x, y, w, h );
			Vec3F hit = intersectLinePlane (m_cam->getPos(), dir, m_goal, Vec3F(0,0,1) );
			m_goal = hit;
		} else {
			if ( m_adjust != -1 ) {			
				m_joints.MoveJoint ( m_adjust, 2, dx*0.01f );
			} else {
				// Adjust camera orbit 
				Vec3F angs = m_cam->getAng();
				angs.x += dx*0.2f*fine;
				angs.y -= dy*0.2f*fine;				
        m_cam->SetOrbit ( angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly() );
			} 		
		}
		appPostRedisplay();	// Update display
		} break;
	}
}

void Sample::mouse ( AppEnum button, AppEnum state, int mods, int x, int y)
{
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag

	mouse_action = 0;

	int w = getWidth(), h = getHeight();
	Vec3F dir = m_cam->inverseRay( x, y, w, h );

	float t;
	bool hit = intersectLineBox (m_cam->getPos(), dir, m_goal-Vec3F(0.1f,0.1f,0.1f), m_goal+Vec3F(0.1f,0.1f,0.1f), t );
	
	if ( hit && state==AppEnum::BUTTON_PRESS)
		if (button==AppEnum::BUTTON_LEFT)
			mouse_action = MOVE_GOAL_XZ;
		else
			mouse_action = MOVE_GOAL_XY;
}

void Sample::keyboard (int keycode, AppEnum action, int mods, int x, int y)
{
	if (action == AppEnum::BUTTON_RELEASE) return;

	switch ( keycode ) {
	case 's':	
    m_bRun = false; m_bStep = true;	 
    dbgprintf("Model: %s, %s\n", (m_type == 0) ? "robot arm" : "human arm", m_bRun ? "sim" : "single step");
    break;
	case ' ':	
    m_bRun = true;
    dbgprintf("Model: %s, %s\n", (m_type == 0) ? "robot arm" : "human arm", m_bRun ? "sim" : "single step");    
    break;
	
	case 't': case 'T': 
		m_type = 1-m_type;      
		Reset();
		break;
	case 'r':		
		Reset ();
		break;

	case 'c': case 'C':	m_adjust = -1;	break;
	case '0':		 	m_adjust = 0;	break;
	case '1':		 	m_adjust = 1;	break;
	case '2':		 	m_adjust = 2;	break;
	case '3':		 	m_adjust = 3;	break;
	};
}


void Sample::reshape (int w, int h)
{
	appPostRedisplay();
}

void Sample::startup () 
{
	int w=1200, h=700;
	
	appStart ( "Inverse Kinematics", "Inverse Kinematics", w, h, 4, 2, 8 );	
}

