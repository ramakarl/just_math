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
#include "nv_gui.h"			// gui system
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

	Camera3D*	cam;
	Vector3DF	m_goal;

	Joints		m_joints;	
};

// Application object
Sample sample_obj;			


void Sample::Reset ()
{
	// create joints
	m_joints.Clear ();

	switch (m_type) {
	case 0:					//---- Robot Arm		
		// AddJoint: name, length, angles, freedom x,y,z
		m_joints.AddJoint ( "J0", 1, Vector3DF(0,0,0), 0, 1, 0 );		// base: Y-axis freedom
		m_joints.AddJoint ( "J1", 4, Vector3DF(0,0,0), 1, 0, 0 );		// arm:  X-axis freedom
		m_joints.AddJoint ( "J2", 4, Vector3DF(0,0,0), 1, 0, 0 );
		m_joints.AddJoint ( "J3", 2, Vector3DF(0,0,0), 1, 0, 0 );	
		m_joints.SetLimits ( 1, Vector3DF(-45, 0, 0), Vector3DF(90, 0, 0) );	 
		m_joints.SetLimits ( 2, Vector3DF(5, 0, 0), Vector3DF(170, 0, 0) );	 
		m_joints.SetLimits ( 3, Vector3DF(5, 0, 0), Vector3DF(90, 0, 0) );	
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
		m_joints.AddJoint ( "back",		5, Vector3DF(0,0,0), 0, 0, 0 );					// fixed joint
		m_joints.AddJoint ( "shoulder", 4, Vector3DF(90,0,0),1, 1, 1 );			
		m_joints.AddJoint ( "elbow",	3, Vector3DF(0,0,0), 1, 0, 0 );
		m_joints.AddJoint ( "wrist",	1, Vector3DF(0,0,0), 1, 0, 1 );	 
		m_joints.SetLimits ( 1, Vector3DF(-90,  0, -60), Vector3DF(200, 90, 130) );	 	// shoulder
		m_joints.SetLimits ( 2, Vector3DF( 5,   0,  +5), Vector3DF(150, 0,  0) );		// elbow
		m_joints.SetLimits ( 3, Vector3DF( 0, -90, -25), Vector3DF(+40,+60,+25) );		// wrist	
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
	setText(16,1);
	setview2D ( w, h );	
	glViewport ( 0, 0, w, h );

	cam = new Camera3D;
	cam->setNearFar ( 1, 2000 );
	cam->setOrbit ( Vector3DF(40,30,0), Vector3DF(0,0,0), 70, 1 );
	m_adjust = -1;

	m_bRun = true;
	m_bStep = false;
	m_type = 0;		// robot

	// create joints
	Reset ();	

	return true;
}


void Sample::display()
{
	Vector3DF a,b,c,p;

	clearGL();
	
	// info display
	start2D();
		setview2D(getWidth(), getHeight());
		drawText(10, 20, "Input:", 1,1,1,1);
		drawText(10, 35, "  Orbit view	     Right-click drag", 1,1,1,1);		
		drawText(10, 50, "  Run              Spacebar.", 1,1,1,1);
		drawText(10, 65, "  Type of body     'T' key. Robot or human.", 1,1,1,1);
		drawText(10, 80, "  Move end X/Y     Left-click drag purple cube", 1,1,1,1);
		drawText(10, 95, "  Move end Z       Right-click drag purple cube", 1,1,1,1);				
		drawText(10, 110, "  Single step      S key. Then move cube. Press S again.", 1,1,1,1);
		drawText(10, 130, "Inverse Kinematics:", 1,1,1,1 );

		char msg[128];
		sprintf ( msg, "%s, %s", (m_type==0) ? "robot" : "human", m_bRun ? "running" : "single step" );
		Vector3DF clr = m_bRun ? Vector3DF(0,1,0) : Vector3DF(1,0.5,0);
		drawText(200, 130, msg, clr.x, clr.y, clr.z,1);		
	end2D();

	// run inverse kinematics
	if (m_bRun) 		
		m_joints.InverseKinematics ( m_goal );
	
	if (m_bStep) {
		m_bStep=false;
		m_joints.InverseKinematics ( m_goal, 1 );   // 1 step
	}


	// draw joints
	m_joints.Sketch( cam );

	start3D(cam);
	
	// sketch a grid
	for (int i = -10; i <= 10; i++) {
		drawLine3D(float(i),-0.01f, -10.f, float(i), -0.01f, 10.f, .2f, .2f, .2f, 1.f);
		drawLine3D(-10.f,	-0.01f, float(i), 10.f, -0.01f, float(i), .2f, .2f, .2f, 1.f);
	}	
	// world axes
	a = Vector3DF(1,0,0); b = Vector3DF(0,1,0); c = Vector3DF(0,0,1); 
	p = Vector3DF(-10.f,-0.01f,-10.f);
	drawLine3D ( p.x, p.y, p.z, p.x+a.x, p.y+a.y, p.z+a.z, 1, 0, 0, 1 );
	drawLine3D ( p.x, p.y, p.z, p.x+b.x, p.y+b.y, p.z+b.z, 0, 1, 0, 1 );
	drawLine3D ( p.x, p.y, p.z, p.x+c.x, p.y+c.y, p.z+c.z, 0, 0, 1, 1 );
		
	// goal
	p = m_goal;
	drawBox3D ( Vector3DF(p.x-0.1f, p.y-0.1f, p.z-0.1f), Vector3DF(p.x+0.1f, p.y+0.1f, p.z+0.1f), 1, 0, 1, 1 );
	drawBox3D ( Vector3DF(p.x-0.1f, 0.f, p.z-0.1f), Vector3DF(p.x+0.1f, 0.f, p.z+0.1f), .75f, 0, .75f, 1 );
	if ( mouse_action == MOVE_GOAL_XY )
		drawBox3D ( Vector3DF(-10.f, 0.f, p.z-0.1f), Vector3DF(10.f, 8.f, p.z+0.1f), 1, 0, 1, 1 );
	if ( mouse_action == MOVE_GOAL_XZ )
		drawBox3D ( Vector3DF(-10.f, p.y-0.1f, -10.f), Vector3DF(10.f, p.y+0.1f, 10.f), 1, 0, 1, 1 );

	end3D();

	// Draw in 2D/3D
	draw3D ();										// Render the 3D drawing groups
	drawGui (0);									// Render the GUI
	draw2D (); 

	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::mousewheel(int delta)
{
	// Adjust zoom
	float zoomamt = 1.0;
	float dist = cam->getOrbitDist();
	float dolly = cam->getDolly();
	float zoom = (dist - dolly) * 0.001f;
	dist -= delta * zoom * zoomamt;
	cam->setOrbit(cam->getAng(), cam->getToPos(), dist, dolly);
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
			Vector3DF dir = cam->inverseRay( x, y, w, h );
			Vector3DF hit = intersectLinePlane ( cam->getPos(), cam->getPos()+dir, m_goal, Vector3DF(0,1,0) );
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
		cam->moveRelative ( float(dx) * fine*cam->getOrbitDist()/1000, float(-dy) * fine*cam->getOrbitDist()/1000, 0 );	
		appPostRedisplay();	// Update display
		} break;
	
	case AppEnum::BUTTON_RIGHT: {	

		if ( mouse_action == MOVE_GOAL_XY ) {
			int w = getWidth(), h = getHeight();			
			Vector3DF dir = cam->inverseRay( x, y, w, h );
			Vector3DF hit = intersectLinePlane ( cam->getPos(), cam->getPos()+dir, m_goal, Vector3DF(0,0,1) );
			m_goal = hit;
		} else {
			if ( m_adjust != -1 ) {			
				m_joints.MoveJoint ( m_adjust, 2, dx*0.01f );
			} else {
				// Adjust camera orbit 
				Vector3DF angs = cam->getAng();
				angs.x += dx*0.2f*fine;
				angs.y -= dy*0.2f*fine;				
				cam->setOrbit ( angs, cam->getToPos(), cam->getOrbitDist(), cam->getDolly() );				
			} 		
		}
		appPostRedisplay();	// Update display
		} break;
	}
}

void Sample::mouse ( AppEnum button, AppEnum state, int mods, int x, int y)
{
	if ( guiHandler ( button, state, x, y ) ) return;	
	
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag

	mouse_action = 0;

	int w = getWidth(), h = getHeight();
	Vector3DF dir = cam->inverseRay( x, y, w, h );
	Vector3DF t = intersectLineBox ( cam->getPos(), dir, m_goal-Vector3DF(0.1f,0.1f,0.1f), m_goal+Vector3DF(0.1f,0.1f,0.1f) );
	
	if ( t.z >= 0 && state==AppEnum::BUTTON_PRESS)
		if (button==AppEnum::BUTTON_LEFT)
			mouse_action = MOVE_GOAL_XZ;
		else
			mouse_action = MOVE_GOAL_XY;
}

void Sample::keyboard (int keycode, AppEnum action, int mods, int x, int y)
{
	if (action == AppEnum::BUTTON_RELEASE) return;

	switch ( keycode ) {
	case 's':	m_bRun = false; m_bStep = true;	 break;
	case ' ':	m_bRun = true;	break;
	
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
	int w=1400, h=1000;
	
	appStart ( "Inverse Kinematics", "Inverse Kinematics", w, h, 4, 2, 8 );	
}

