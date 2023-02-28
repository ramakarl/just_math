
#ifndef DEF_SPLINE
	#define DEF_SPLINE

	#include "datax.h" 
	#include "quaternion.h"	

	#define SP_TIME		0
	#define SP_KNOTS	1

	struct SplineChan {
		char		m_buf;
		char		m_eval;		// evaluation: '0' off, 'p' point, 'l' linear, 'b' bspline, 'c' catmull-rom, 'z' bezier
		char		m_type;		//  data type: '3' vec3, '4' vec4, 'q' quaternion		
		Vector4DF	m_result;
	};


	class HELPAPI Spline : public DataX {
	public:
		Spline ();

		// Construct Spline
		void		CreateSpline ();
		void		AddChannel ( int ch, char eval, char dtype );
		void		AddKeyToSpline (float t);
		void		SetKey (int ch, Vector4DF val, int i=-1);
		void		AlignSpline ();
		void		UpdateSpline ();
		int			FindKey( float t, float& u);
		
		// Evaluate Spline
		void		EvaluateSpline ( float t );							
		Vector4DF	EvaluatePoint		( int b, int k, float u );			// vec3
		Vector4DF	EvaluateLinear		( int b, int k, float u );			
		Vector4DF	EvaluateBSpline		( int b, int k, float u );
		Vector4DF	EvaluateCatmullRom	( int b, int k, float u );
		Vector4DF	EvaluateBezier		( int b, int k, float u );
		
		Vector4DF	EvaluateQSlerp		( int b, int k, float u );			// quaternion (spherical-linear)

		Vector3DF	getSplineV3 ( int ch )	{ return (Vector3DF) m_Channels[ch].m_result; }			// get evaluation results
		Vector4DF	getSplineV4 ( int ch )	{ return (Vector4DF) m_Channels[ch].m_result; }
		Quaternion	getSplineQ ( int ch )	{ return (Quaternion) m_Channels[ch].m_result; }

		int			getNumKeys();
		float		getKeyTime(int i)	{ return GetElemFloat(SP_TIME, i); }
		//Vector3DF	getKeyPos(int i)	{ return *GetElemVec3(SP_POS, i); }
		//Quaternion  getKeyRot(int i)	{ return *GetElemQuat(SP_ROT, i); }

	public:		
		
		bool			m_rebuild;
		int				m_num;
		int				m_k;				// current key index

		int				m_degree;
		float			m_tension;
		float			m_control_amt;
		float			m_normal_amt;

		std::vector<SplineChan>	m_Channels;
	};

#endif
	
