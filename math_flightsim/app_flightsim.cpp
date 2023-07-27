//--------------------------------------------------------
// JUST MATH:
// Flight simulator
// 
//
//
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

	void		Advance ();
	void		UpdateCamera();
	void		drawGrid();
	
	Vector3DF	m_pos, m_vel, m_accel;
	Vector3DF	m_lift, m_thrust, m_drag, m_force;
	Vector3DF	m_angvel;
	Quaternion	m_orient;	
	
	
	Matrix4F	m_inertia, m_inv_inertia;
	Vector3DF   m_ctr_of_pressure;

	float		m_speed;	
	float		m_roll, m_pitch, m_pitch_adv, m_power;
	float		m_DT;

	Mersenne	mt;
	float		m_time;
	bool		m_run, m_flightcam;
	Camera3D*	m_cam;
	int			mouse_down;
};
Sample obj;


bool Sample::init ()
{
	int w = getWidth(), h = getHeight();			// window width & height
	m_run = true;
	m_flightcam = false;

	addSearchPath ( ASSET_PATH );
	init2D ( "arial" );
	setview2D ( w, h );	
	setText ( 16, 1 );		
	
	m_cam = new Camera3D;
	m_cam->setFov ( 120 );
	m_cam->setNearFar ( 0.1, 5000 );
	m_cam->SetOrbit ( Vector3DF(30,40,0), Vector3DF(5,0,0), 30, 1 );

	mt.seed(164);

	m_pos.Set (0, 0, 0);
	m_vel.Set (0, 0, 0);			// speed = 10 m/s
	m_roll = 0;
	m_pitch = 0;
	m_pitch_adv = 0.1;
	m_power = 1;
	m_accel.Set(0,0,0);
	m_orient.fromDirectionAndRoll ( Vector3DF(1,0,0), m_roll );
	m_DT = 0.001;

	m_ctr_of_pressure.Set (10.0f, 0, 0.0);

	 /* m_inertia.toBasis ( Vector3DF(48531.0f, -1320.0f, 0.0f),
					    Vector3DF(-1320.0f, 256608.0f, 0.0f),
					    Vector3DF(0.0f, 0.0f, 21133.0f ) ); */
	
	/*m_inertia.toBasis ( Vector3DF( 5.0f,  -1.0f, 0.0f),
					    Vector3DF(-1.0f,   1.0f, 0.0f),
					    Vector3DF( 0.0f,   0.0f, 1.0f ) );  */

	m_inertia.toBasis ( Vector3DF( 1.0f, 0.0f, 0.0f),
					    Vector3DF( 0.0f, 1.0f, 0.0f),
					    Vector3DF( 0.0f, 0.0f, 1.0f ) ); 
	
	m_inv_inertia.makeInverse3x3 ( m_inertia );

	return true;
}

void Sample::drawGrid()
{
	float o	 = -0.05;		// offset
	for (int n=-5000; n <= 5000; n+=20 ) {
		drawLine3D ( n, o,-5000, n, o,5000, 1,1,1,1);
		drawLine3D (-5000, o, n, 5000, o, n, 1,1,1,1);
	}
}

void Sample::Advance ()
{
	Vector3DF force, torque;
	
	float m_LiftFactor = 0.00065;
	float m_DragFactor = 0.005;

	float mass = 0.1;	// kg. weight of starling = 3.6 oz = 0.1 kg
	
	// Body frame of reference
	Vector3DF fwd = Vector3DF(1,0,0) * m_orient;
	Vector3DF up  = Vector3DF(0,1,0) * m_orient;
	Vector3DF left = Vector3DF(0,0,1) * m_orient;

	// Velocity limit
	m_speed = m_vel.Length();
	Vector3DF vaxis = m_vel / m_speed;	
	if ( m_speed < 0 ) m_speed =  0;		// planes dont go in reverse
	if ( m_speed > 50 ) m_speed = 50;
	if ( m_speed==0) vaxis = fwd;

	// Pitch inputs - modify direction of target velocity 
	Quaternion ctrl_pitch;
	m_pitch_adv = m_pitch_adv * 0.9995 + m_pitch * 0.005;
	ctrl_pitch.fromAngleAxis ( m_pitch_adv*0.0001, Vector3DF(0,0,1) * m_orient );
	vaxis *= ctrl_pitch; 				

	m_vel = vaxis * m_speed;

	m_force = 0;
	torque = 0;

	// Dynamic pressure
	float p = 1.225;				// kg/m^3m, air density
	float dynamic_pressure = 0.5f * p * m_speed * m_speed;

	// Lift force	
	float aoa = fwd.Dot( vaxis );
	float L = (aoa*aoa) * dynamic_pressure * m_LiftFactor;
	m_lift = up * L;
	m_force += m_lift;	

	// Drag force	
	m_drag = vaxis * (dynamic_pressure / (m_speed+0.001f) ) * m_DragFactor * -1.0f;	
	m_force += m_drag; 

	// Thrust force
	m_thrust = fwd * m_power;
	m_force += m_thrust;

	// Integrate orientation
	// airplane will reorient toward the velocity vector
	Quaternion angvel;
	angvel.fromRotationFromTo (  fwd, vaxis, 0.02 );
	if ( !isnan(angvel.X) ) {
		m_orient *= angvel;
		m_orient.normalize();
	}	
	// Roll inputs - modify body orientation
	Quaternion ctrl_roll;
	ctrl_roll.fromAngleAxis ( m_roll*0.001, Vector3DF(1,0,0) * m_orient );
	m_orient *= ctrl_roll; m_orient.normalize();		// roll inputs

	// Integrate position		
	m_accel = m_force / mass;
	m_accel += Vector3DF(0,-9.8,0);	
	m_pos += m_vel * m_DT;			// do this first so that vel is limited			
	if (m_pos.y <= 0 ) { m_pos.y = 0; m_vel.y = 0; m_accel += Vector3DF(0,9.8,0); }		// on ground condition
	m_vel += m_accel * m_DT;
	
}


void Sample::UpdateCamera()
{
	// View direction	
	Vector3DF fwd = m_vel; fwd.Normalize();
	Vector3DF angs;
	m_orient.toEuler ( angs );
	
	m_cam->setDirection ( m_pos, m_pos + fwd, -angs.x );
}

void Sample::display ()
{	
	Vector3DF pnt;
	Vector4DF clr;

	int w = getWidth();
	int h = getHeight();

	if (m_run) { 
		//m_run = false;
		Advance ();
	}
	if (m_flightcam) UpdateCamera();
	
	char msg[128];

	clearGL();
	start2D();
		setview2D(getWidth(), getHeight());
		
		sprintf ( msg, "power: %f", m_power );	drawText(10, 20, msg, 1,1,1,1);
		sprintf ( msg, "speed: %f", m_speed );	drawText(10, 40, msg, 1,1,1,1);
		sprintf ( msg, "roll:  %f", m_roll );	drawText(10, 60, msg, 1,1,1,1);
		sprintf ( msg, "pitch: %f", m_pitch_adv );	drawText(10, 80, msg, 1,1,1,1);
	end2D();


	start3D(m_cam);

		drawGrid();
		Vector3DF a,b,c;
		a = Vector3DF(1,0,0)*m_orient;
		b = Vector3DF(0,1,0)*m_orient;
		c = Vector3DF(0,0,1)*m_orient;
		drawLine3D ( m_pos, m_pos+a, Vector4DF(1,1,1,1) );
		drawLine3D ( m_pos, m_pos+b, Vector4DF(1,1,1,1) );
		drawLine3D ( m_pos, m_pos+c, Vector4DF(1,1,1,1) );
		drawLine3D ( m_pos, m_pos+m_lift, Vector4DF(0,1,0,1) );
		drawLine3D ( m_pos, m_pos+m_thrust, Vector4DF(1,0,0,1) );
		drawLine3D ( m_pos, m_pos+m_drag, Vector4DF(1,0,1,1) );
		drawLine3D ( m_pos, m_pos+m_vel, Vector4DF(1,1,0,0.5) );

		drawLine3D ( Vector3DF(0,0,0), a, Vector4DF(1,1,1,1) );
		drawLine3D ( Vector3DF(0,0,0), b, Vector4DF(1,1,1,1) );
		drawLine3D ( Vector3DF(0,0,0), c, Vector4DF(1,1,1,1) );
		drawLine3D ( Vector3DF(0,0,0), m_lift, Vector4DF(0,1,0,1) );
		drawLine3D ( Vector3DF(0,0,0), m_thrust, Vector4DF(1,0,0,1) );
		drawLine3D ( Vector3DF(0,0,0), m_drag, Vector4DF(1,0,1,1) );
		drawLine3D ( Vector3DF(0,0,0), m_vel, Vector4DF(1,1,0,1) );
		drawLine3D ( Vector3DF(0,0,0), m_force, Vector4DF(0,1,1,1) );

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
	
		} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		float zoom = (m_cam->getOrbitDist() - m_cam->getDolly()) * 0.0003f;
		m_cam->moveRelative ( float(dx) * zoom, float(-dy) * zoom, 0 );	
		} break; 

	case AppEnum::BUTTON_RIGHT: {
		// Adjust orbit angles
		Vector3DF angs = m_cam->getAng();
		angs.x += dx*0.2f*fine;
		angs.y -= dy*0.2f*fine;				
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
	if (action == AppEnum::BUTTON_RELEASE) {
		switch ( keycode ) {
		case KEY_LEFT: case KEY_RIGHT: m_roll = 0; break;
		case KEY_UP: case KEY_DOWN: m_pitch = 0; break;
		};
		return;
	}

	switch ( keycode ) {
	case ' ':	m_run = !m_run;	break;
	case 'c':	
		m_flightcam = !m_flightcam; 
		if (!m_flightcam) {
			m_cam->SetOrbit ( Vector3DF(30,40,0), Vector3DF(0,0,0), 100, 1 );
		}
		break;
	case 'w':	
		m_power += 1.0; 
		break;
	case 's':	
		m_power -= 1.0; 
		if (m_power < 0) m_power = 0; 
		break;
	case KEY_LEFT:	m_roll = -1.0; break;
	case KEY_RIGHT:	m_roll = +1.0; break;
	case KEY_UP:	m_pitch = -1.0; break;
	case KEY_DOWN:	m_pitch = +1.0; break;
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
	appStart ( "Cell simulation", "Cell simulation", w, h, 4, 2, 16, false );
}

void Sample::shutdown()
{
}

