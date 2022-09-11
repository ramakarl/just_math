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


#include "joints.h"
#include "camera3d.h"
#include "main.h"
#include "nv_gui.h"

#include <stack>
#include <vector>

#define JUNDEF		-1

Joints::Joints ()
{
}

// To Do:
// - angle constraints
// - ball joints
// - 6-DOF end effector

float circleDelta ( float a, float b )
{
	a = (float) fmod (a, 360 ); if ( a < 0 ) a += 360;

	float d = fabs( b - a);
	float r = d > 180 ? 360 - d : d;
	int sign = (b - a >= 0 && b - a <= 180) || (b - a <=-180 && b - a>= -360) ? 1 : -1; 
	return r * sign;
}

void Joints::Sketch ( Camera3D* cam )
{
	Vector3DF dir, ang;
	Vector3DF a,b,c, p;
	Matrix4F world, local;	

	if ( m_Joints.size()==0 ) {
		dbgprintf ( "ERROR: No joints defined.\n" );
		exit(-1);
	}

	Vector3DF angs;
	

/*	start2D();
	char msg[512];
	sprintf ( msg, "action: %s -> %s (%d of %d)  %4.1f\n", m_CycleSet->getName(m_CCurr.cycle), m_CycleSet->getName(m_CNext.cycle), m_CFrame, m_CEnd, mTargetDist );
	drawText( 10, 10, msg, 1,1,1,1);
	end2D();*/

	start3D( cam );

	//-------- draw end effector
	p = m_Effector;
	drawBox3D ( Vector3DF(p.x-.05f, p.y-.05f, p.z-.05f), Vector3DF(p.x+.05f, p.y+.05f, p.z+.05f), 1,1,1,1 );
	c = m_Goal - m_Effector;
	c.Normalize();
	drawLine3D ( p.x, p.y, p.z, p.x+c.x, p.y+c.y, p.z+c.z, 1,0.5,1, 1 );
	

	//-------- draw joints
	//	
	m_Joints[0].orient.getMatrix ( world );
	world.PostTranslate ( m_Joints[0].pos );
	EvaluateJoints( m_Joints, world );	
	
	Matrix4F mtx;							// N(n) = Oi^-1(n-1) Rn(n) Oi(n)    (eqn 4.9)
	Matrix4F OiInv;
	Matrix4F Ri, Rj0, Ri2, pmtx, Mv;
		
	Vector3DF u, q, xangs;
	
	for (int n=0; n < m_Joints.size(); n++ ) {
		
		p = m_Joints[n].pos;
		
		// draw joint center
		drawBox3D ( Vector3DF(p.x-.1f, p.y-.1f, p.z-.1f), Vector3DF(p.x+.1f, p.y+.1f, p.z+.1f), 0.6, 0.6, 0.6, 1 );

		// draw joint orientation 
		local = m_Joints[n].Mworld;
		local.PostTranslate ( m_Joints[n].pos * -1.0f);			
		a = Vector3DF(1,0,0); a *= local; 
		b = Vector3DF(0,1,0); b *= local; 
		c = Vector3DF(0,0,1); c *= local; 
		drawLine3D ( p.x, p.y, p.z, p.x+a.x, p.y+a.y, p.z+a.z, 1, 0, 0, 1 );	// red, x-axis
		drawLine3D ( p.x, p.y, p.z, p.x+b.x, p.y+b.y, p.z+b.z, 0, 1, 0, 1 );	// green, y-axis
		drawLine3D ( p.x, p.y, p.z, p.x+c.x, p.y+c.y, p.z+c.z, 0, 0, 1, 1 );	// blue, z-axis

		c = b * (m_Joints[n].length - 0.1f);
		b *= 1.1f;
		drawLine3D ( p.x+b.x, p.y+b.y, p.z+b.z, p.x+c.x, p.y+c.y, p.z+c.z, 1, 1, 0, 1 );   // bone

		drawLine3D ( p.x+b.x, 0, p.z+b.z, p.x+c.x, 0, p.z+c.z, 0.5, 0.5, 0.3, 1 );	// bone shadow
	}

	end3D();
}

int Joints::getLastChild ( int p )
{
	int child = m_Joints[p].child;
	if ( child == JUNDEF ) return JUNDEF;
	while ( m_Joints[child].next != JUNDEF ) {
		child = m_Joints[child].next;
	}
	return child;
}
void Joints::Clear ()
{
	m_Joints.clear ();
}

void Joints::SetLimits ( int j, Vector3DF lmin, Vector3DF lmax )
{
	m_Joints[j].min_limit = lmin;
	m_Joints[j].max_limit = lmax;
}

int Joints::AddJoint ( char* name, float length, Vector3DF angs, int cx, int cy, int cz )
{
	Joint jnt;	
	int parent = m_Joints.size()-1;		// use last joint added as parent

	strncpy_s ( jnt.name, 64, name, 64 );
	jnt.parent = parent;	
	jnt.pos = Vector3DF(0,0,0);
	jnt.length = length;
	jnt.angs = angs;
	jnt.dof = Vector3DI(cx,cy,cz);
	jnt.orient.set ( angs );	// quaternion from euler angles
	jnt.min_limit.Set(-180.f,-180.f,-180.f);
	jnt.max_limit.Set( 180.f, 180.f, 180.f);
	jnt.clr = 0;	
	jnt.child = JUNDEF;
	jnt.next = JUNDEF;
	
	int curr = (int) m_Joints.size();

	if ( parent == JUNDEF ) {	
		// this is root
		jnt.lev = 0;		
	} else {
		Joint* pjnt = getJoint ( parent );
		jnt.lev = pjnt->lev + 1;
		int plastchild = getLastChild ( parent );	// last child of parent
		if ( plastchild != JUNDEF ) {
			getJoint(plastchild)->next = curr;			
		} else {
			pjnt->child = curr;			
		}		
	}

	m_Joints.push_back ( jnt );					// add joint

	return (int) m_Joints.size()-1;
}

void Joints::MoveJoint ( int j, int axis_id, float da )
{
	if ( j >= m_Joints.size() ) return;

	Vector3DF axis;
	switch ( axis_id ) {
	case 0: axis.Set(1,0,0); break;
	case 1: axis.Set(0,1,0); break;
	case 2: axis.Set(0,0,1); break;
	};
	bool allow = (axis.Dot ( m_Joints[j].dof ) > 0 );
	if ( !allow ) return;

	Quaternion delta;
	delta.fromAngleAxis ( da, axis );
	delta.normalize(); 	
	
	m_Joints[j].orient = m_Joints[j].orient * delta;		// local rotation
	m_Joints[j].orient.normalize();	

	m_Joints[j].orient.toEuler ( m_Joints[j].angs );


}

Joint* Joints::FindJoint ( std::string name )
{
	for (int n=0; n < getNumJoints(); n++ ) {
		if ( name.compare(m_Joints[n].name) == 0 )
			return &m_Joints[n];
	}
	return 0x0;
}

void Joints::EvaluateJoints ( std::vector<Joint>& joints, Matrix4F& world )
{
	EvaluateJointsRecurse ( joints, 0, world );

	// Compute end effector
	int n = joints.size()-1;	
	Matrix4F  local = m_Joints[n].Mworld;	
	Vector3DF b (0.f, m_Joints[n].length, 0.f); 
	local.PostTranslate ( m_Joints[n].pos * -1.0f);					
	b *= local; 
	m_Effector = m_Joints[n].pos + b;
}

// recursive funcs
void Joints::EvaluateJointsRecurse ( std::vector<Joint>& joints, int curr_jnt, Matrix4F world )
{
	// Evaluation of joint chain
	//
	// local orientation
	Matrix4F orient;
	Vector3DF a; 
	// joints[curr_jnt].orient.toEuler ( a );			// cast to Euler ZYX angles first
	// orient.RotateZYX ( a );						
	joints[curr_jnt].orient.getMatrix ( orient );		// Ri' = orientation angles (animated)		
	
	// set world transform
	if ( curr_jnt > 0 ) 
		world *= orient;								// Mw = M(w-1) Ri' v''
	
	joints[curr_jnt].Mworld = world;					
	joints[curr_jnt].pos = world.getTrans();			// Tworld
	
														// translate children to end of bone
	world.PreTranslate ( Vector3DF(0.f, joints[curr_jnt].length, 0.f) );		// v'' = bone length

	// recurse	
	int child_jnt = joints[curr_jnt].child;
	while ( child_jnt != JUNDEF ) {
		EvaluateJointsRecurse ( joints, child_jnt, world );
		child_jnt = joints[child_jnt].next;
	}
}

void Joints::StartIK ()
{
	// Count degrees-of-freedom (DOFs)
	int M = 0;
	for (int n=0; n < m_Joints.size(); n++ ) {
		M += m_Joints[n].dof.x;
		M += m_Joints[n].dof.y;
		M += m_Joints[n].dof.z;
	}

	// Construct Jacobian
	m_Jacobian.Resize ( M, 3 );
}

void Joints::InverseKinematics ( Vector3DF goal, int maxiter )
{	
	Matrix4F world;
	float dE;
	float amt;
	int iter = 0;

	m_Goal = goal;
	dE = (m_Goal - m_Effector).Length();

	while ( dE > 0.1f && iter++ < maxiter  ) {

		// check convergence
		dE = (m_Goal - m_Effector).Length();

		amt = pow(dE * 0.2f, 1.5);		
		if ( amt > 0.5f ) amt = 0.5f;

		// compute jacobian
		ComputeJacobian ();

		// apply jacobian transpose
		ApplyJacobianTranspose ( amt );

		// re-evaluate joints
		m_Joints[0].orient.getMatrix ( world );
		world.PostTranslate ( m_Joints[0].pos );
		EvaluateJoints( m_Joints, world );
	}
}

void Joints::LimitQuaternion ( Quaternion& o, int limitaxis_id, float limitang )
{
	Vector3DF angs;	
	Vector3DF a1, a2;

	o.toEuler ( angs );
	a1 = angs;

	char c=' ';
	switch ( abs(limitaxis_id) ) {
	case 1: angs.x = limitang; c='X'; break;
	case 2: angs.y = limitang; c='Y'; break;
	case 3: angs.z = limitang; c='Z'; break;
	}
	o.set ( angs );
	o.normalize();
	o.toEuler( a2 );

	/*printf ( "Limit %c%c\n", (limitaxis_id<0) ? '-' : '+', c );
	printf ( "  before: <%3.2f, %3.2f, %3.2f>\n", a1.x,a1.y,a1.z);
	printf ( "  after:  <%3.2f, %3.2f, %3.2f>\n", a2.x,a2.y,a2.z);*/

}

void Joints::ApplyJacobianTranspose ( float amt )
{
	Vector3DF jT;
	Vector3DF dE; 
	Quaternion dq, o1;
	Vector3DF angs, a1, a2, a3, a4, a5, a6;
	float dang;
	float lz;
	std::string msg;
	bool limit=false;

	// effector delta (dE)
	dE = m_Goal - m_Effector;
	dE.Normalize();

	int M=0;
	for (int n=0; n < m_Joints.size(); n++ ) {
				
		m_Joints[n].orient.toEuler ( angs );				
		if (angs.z < -90) angs.z = 360 + angs.z;
		
		if ( m_Joints[n].dof.x==1 ) {		
			jT.x = m_Jacobian(M, 0);
			jT.y = m_Jacobian(M, 1); 
			jT.z = m_Jacobian(M, 2);
			dang = jT.x * dE.x + jT.y * dE.y + jT.z * dE.z;			// multiply one row of J^T by dE vector
			dang *= amt;			
			if ( angs.x + dang < m_Joints[n].min_limit.x ) {									
				LimitQuaternion ( m_Joints[n].orient, -1, m_Joints[n].min_limit.x );				
			} else if ( angs.x + dang > m_Joints[n].max_limit.x ) {
				LimitQuaternion ( m_Joints[n].orient, 1, m_Joints[n].max_limit.x );				
			} else {				
				dq.fromAngleAxis ( dang, Vector3DF(1,0,0) );	// rotate around local X-axis	
				m_Joints[n].orient = m_Joints[n].orient * dq;
				m_Joints[n].orient.normalize();
			}									
			M++;
		}

		if ( m_Joints[n].dof.y==1 ) {		
			jT.x = m_Jacobian(M, 0);
			jT.y = m_Jacobian(M, 1); 
			jT.z = m_Jacobian(M, 2);
			dang = jT.x * dE.x + jT.y * dE.y + jT.z * dE.z;			// multiply one row of J^T by dE vector
			dang *= amt;
			if ( angs.y + dang < m_Joints[n].min_limit.y ) {					
				LimitQuaternion ( m_Joints[n].orient, -2, m_Joints[n].min_limit.y );						
			} else if ( angs.y + dang > m_Joints[n].max_limit.y ) {			
				LimitQuaternion ( m_Joints[n].orient, 2, m_Joints[n].max_limit.y );					
			} else {				
				dq.fromAngleAxis ( dang, Vector3DF(0,1,0) );	// rotate around local Y-axis			
				m_Joints[n].orient = m_Joints[n].orient * dq;
				m_Joints[n].orient.normalize();			
			}
			M++;
		}		

		if ( m_Joints[n].dof.z==1 ) {			
			jT.x = m_Jacobian(M, 0);
			jT.y = m_Jacobian(M, 1); 
			jT.z = m_Jacobian(M, 2);
			msg="";
			dang = jT.x * dE.x + jT.y * dE.y + jT.z * dE.z;			// multiply one row of J^T by dE vector
			dang *= amt;			
			if ( angs.z + dang < m_Joints[n].min_limit.z ) {	
				LimitQuaternion ( m_Joints[n].orient, -3, m_Joints[n].min_limit.z );				
			} else if ( angs.z + dang > m_Joints[n].max_limit.z  ) {				
				LimitQuaternion ( m_Joints[n].orient, 3, m_Joints[n].max_limit.z );							
			} else {				
				dq.fromAngleAxis ( dang, Vector3DF(0,0,1) );	// rotate around local Z-axis
				m_Joints[n].orient = m_Joints[n].orient * dq;
				m_Joints[n].orient.toEuler(a4);
				m_Joints[n].orient.normalize();
				m_Joints[n].orient.toEuler(a5);
			}
			M++;
		}

		m_Joints[n].orient.toEuler ( angs );		
		//printf ("J%d  <%3.2f, %3.2f, %3.2f>  %3.2f\n", n, angs.x, angs.y, angs.z, a1.z );
	}

}

void Joints::ComputeJacobian ()
{
	Vector3DF r, c, axis, delta;
	Vector3DF dE;
	Matrix4F mtx;
	
	// effector delta (dE)
	dE = m_Goal - m_Effector;
	dE.Normalize();

	// process each joint to find DOFs
	int M =0;
	for (int n=0; n < m_Joints.size(); n++ ) {
		
		r = m_Effector - m_Joints[n].pos;			// r = e - joint_pos
		r.Normalize();
		mtx = m_Joints[n].Mworld;					// local orientation of joint
		mtx.PostTranslate ( m_Joints[n].pos * -1.0f );
		
		if ( m_Joints[n].dof.x == 1) {
			// Use x-axis rotation on this joint
			axis.Set( 1, 0, 0 );					// get the joints x-axis
			axis *= mtx;							// axis in world space			
			axis.Normalize();
			delta = axis.Cross ( r );				// J(phi) = axis X (E - p)    // X=cross product, E=end effector, p=joint position, axis=joint axis (in world space)
			delta.Normalize();
			m_Jacobian(M,0) = delta.x;				// write to jacobian
			m_Jacobian(M,1) = delta.y;
			m_Jacobian(M,2) = delta.z;			
			M++;
		}
		if ( m_Joints[n].dof.y == 1) {
			// Use y-axis rotation on this joint
			axis.Set( 0, 1, 0 );					// get the joints y-axis
			axis *= mtx;							// rotation axis in world space
			axis.Normalize();
			delta = axis.Cross ( r );				
			delta.Normalize();
			m_Jacobian(M,0) = delta.x;				// write to jacobian
			m_Jacobian(M,1) = delta.y;
			m_Jacobian(M,2) = delta.z;			
			M++;
		}
		if ( m_Joints[n].dof.z == 1) {
			// Use z-axis rotation on this joint
			axis.Set( 0, 0, 1 );					// get the joints z-axis
			axis *= mtx;							// rotation axis in world space
			axis.Normalize();
			delta = axis.Cross ( r );
			delta.Normalize();
			m_Jacobian(M,0) = delta.x;				// write to jacobian
			m_Jacobian(M,1) = delta.y;
			m_Jacobian(M,2) = delta.z;			
			M++;
		}	
	}

}