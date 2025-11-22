

#ifndef DEF_CHARACTER
	#define DEF_CHARACTER

	#include <vector>
	#include <queue>
	#include "quaternion.h"
	#include "vec.h"

	struct Joint {
		char		name[64];
		char		lev;
		uint		clr;
		float		length;		// bone length, v''		
		Vec3F	pos;		// bone position, T
		Vec3I	dof;
		Quaternion	orient;		// orientation angles, Ri (converted to ideal format, Mv = T Ri' v'', see Meredith & Maddock paper)
		Matrix4F	Mworld;		// world transform (computed on-the-fly)
		int			parent;
		int			child;
		int			next;
		
		Vec3F	min_limit;
		Vec3F	max_limit;

		//-- debugging
		Vec3F	angs;		// BVH angles (debug purposes, will be removed in future)
		Vec3F	bonevec;	// BVH bone vec
	};

	class Camera3D;

	class Joints  {
	public:
		Joints () ;

		virtual void Sketch ( Camera3D* cam );
		
		void Clear ();

		int AddJoint ( char* name, float length, Vec3F angs, int cx, int cy, int cz );		
		void SetLimits ( int j, Vec3F lmin, Vec3F lmax );
		void MoveJoint ( int j, int axis_id, float da);
		void EvaluateJoints ( std::vector<Joint>& joints, Matrix4F& world );			// Evaluate cycle to joints
		void EvaluateJointsRecurse ( std::vector<Joint>& joints, int curr_jnt, Matrix4F tform );	// Update joint transforms				
		void StartIK ();
		void InverseKinematics ( Vec3F goal, int maxiter=100 );
		void ComputeJacobian ();
		void ApplyJacobianTranspose ( float amt );
		void LimitQuaternion ( Quaternion& o, int axis, float limitang );

		Joint* FindJoint ( std::string name );
		Joint* getJoint ( int i )		{ return &m_Joints[i]; }
		std::vector<Joint>& getJoints()	{ return m_Joints; }
		int getNumJoints ()				{ return (int) m_Joints.size(); }							
		int getLastChild ( int p );

	private:
	
		std::vector<Joint>		m_Joints;				// joint state

		Vec3F				m_Goal;
		Vec3F				m_Effector;
		MatrixF					m_Jacobian;		
	};

#endif