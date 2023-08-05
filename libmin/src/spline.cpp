
#include "spline.h"

Spline::Spline()
{
	m_num = 0;
	m_degree = 3;
	m_tension = 0.5f;
	m_normal_amt = 0.4f;
	m_control_amt = 0.2f;
}

void Spline::CreateSpline ()
{	
	m_Channels.clear ();		// clear all channels
	DeleteAllBuffers ();		// reset all buffers	

	AddBuffer ( SP_TIME,  "time",	sizeof(float),	0, DT_CPU );
	AddBuffer ( SP_KNOTS, "knots",	sizeof(int),	0, DT_CPU );
	m_rebuild = true;
}

void Spline::AddChannel ( int ci, char eval, char dt )
{
	int c = m_Channels.size();		// channel #
	int b = (c+1)*2;				// buffer # - each channel takes 2x buffers
	assert ( c==ci );

	SplineChan ch;
	ch.m_eval = eval;	
	ch.m_type = dt;
	
	// primary buffer
	ch.m_buf = AddBuffer ( b, "ch", sizeof(Vector4DF), 0, DT_CPU );		

	// secondary buffer
	AddBuffer ( b+1, "chx", sizeof(Vector4DF),	0 );			// stores: bspline temps, or catmull-rom midpoints, or bezier tangents

	m_Channels.push_back ( ch );
}

void Spline::AddKeyToSpline (float t)
{
	m_k = AddElem();
	SetElemFloat ( SP_TIME, m_k, t );	
	m_rebuild = true;
}

void Spline::SetKey (int ch, Vector4DF val, int k)
{
	if ( k==-1 ) k = m_k;	
	int b = (ch+1)*2;
	SetElemVec4 ( b, k, val );	
	m_rebuild = true;
}

int	Spline::FindKey( float t, float& u)
{
	float* keyT = (float*) GetElem (SP_TIME, 0);

	// binary search 
	int low = 0;
	int hi = GetNumElem ( SP_TIME )-1;
	int mid;
	if ( t < *(keyT+low) ) {u=0; return low;}
	if ( t > *(keyT+hi) ) {u=0; return hi; }
	while ( (hi-low) > 1 ) {	
		mid = (low + hi) >> 1;
		if ( t >= *(keyT+mid) ) {
			low = mid;
		} else {		
			hi = mid;
		}
	} 

	// linear search 
	/*int low = 0;
	int hi = GetNumElem ( SP_POS )-1;
	for (low=0; *(keyT+low) < t && low <= hi;)
		low++;	
	low--;	*/

	u = (t - *(keyT+low)) / (*(keyT+low+1) - *(keyT+low));
	return low;
}


void Spline::UpdateSpline()
{
	SplineChan ch;
	int b;
	m_num = GetNumElem ( SP_TIME );
		
	if ( m_rebuild ) {
		m_rebuild = false;

		for (int c=0; c < m_Channels.size(); c++) {
			b = (c+1)*2;
			ch = m_Channels[c];

			switch (ch.m_eval) {
			case 'b': {											//---- B-Spline preparation
				// resize btmp vector
				ResizeBuffer ( b+1, m_degree + 1 );
				// resize knot vector				
				ResizeBuffer ( SP_KNOTS, m_num + m_degree*2 );				
				int* knots = (int*) GetElem( SP_KNOTS, 0 );
				for (int n = 0; n < m_degree; n++)	*knots++ = 0;			// knots[n] = 0					-- left padding
				for (int n = 0; n < m_num; n++)		*knots++ = n;			// knots[n+p] = n								
				for (int n = 0; n < m_degree; n++)	*knots++ = m_num - 1;	// knots[n+p+num] = num - 1		-- right padding
				} break;
			case 'c': {											//---- Catmull-Rom preparation
				// construct midpoints				
				Vector4DF* keyR = GetElemVec4 (b, 2 );		// inputs
				Vector4DF* keyL = GetElemVec4 (b, 0 );
				Vector4DF* mids = GetElemVec4 (b+1, 0);		// output
				*mids = (*keyR - *keyL) * m_tension;			// mids[0] = (keys[2] - keys[0]) * tension;		
				for (int n = 1; n < m_num - 1; n++) {
					*mids = (*keyR - *keyL) * m_tension;		// mids[n] = (keys[n + 1] - keys[n - 1]) * tension;		// central difference
					mids++; keyR++; keyL++;
				}
				keyR--; keyL--;
				*mids = (*keyR - *keyL) * m_tension;				// mids[num_keys - 1] = (keys[num_keys - 1] - keys[num_keys - 3]) * tension;		
				} break;
			case 'z': {											//---- Bezier preparation
				// construct tangents	
				// There are many ways to construct Bezier tangents. The most natural is user-controlled, direct manipulation of tangents.
				// One automatic way is to central difference the control pnts. This ignores orientation of keys.
				// Another way is to use the normal of the key orientation. This ignores the curve shape.
				// Here we blend these two with varying amounts. 
				Vector4DF* keyR = GetElemVec4 (b, 2);			// inputs
				Vector4DF* keyL = GetElemVec4 (b, 0);				
				Vector4DF* tans = GetElemVec4 (b+1, 0);			// output
				Vector3DF v(0, 1.0, 0);								// normal axis of keys
				*tans = (*keyR - *keyL) * m_control_amt;		
				for (int n = 1; n < m_num - 1; n++) {
					//*tans = (v * *keyO) * m_normal_amt; keyO++;	// normal of orientation, blended with..
					*tans += (*keyR - *keyL) * m_control_amt;		// central difference of control pnts
					tans++; keyL++; keyR++; 
				}
				keyR--; keyL--;
				*tans = (*keyR - *keyL) * m_control_amt;
				} break;
			};
		}
	}
}

void Spline::AlignSpline ()
{
	/*Vector3DF* keyP = GetElemVec3 (SP_POS, 0);
	Quaternion* keyO = GetElemQuat (SP_ROT, 0);
	Quaternion q;
	Vector3DF dir;
	
	for (int n = 0; n < m_num - 1; n++) {		
		dir = *(keyP+1) - *keyP; dir.Normalize();					// aim at next key
		q.rotationFromTo(Vector3DF(0, 1, 0), dir); q.normalize();
		*keyO = q;
		keyO++; keyP++;
	}*/
}

void Spline::EvaluateSpline (float t)
{
	SplineChan* ch;
	float u;
	int k = FindKey ( t, u );
	int b;

	if  (m_num==0) return;
	
	b=2;
	for (int c=0; c < m_Channels.size(); c++) {
		ch = &m_Channels[c];		

		switch (ch->m_eval) {				
		case '0': break;	// no eval
		case 'p': ch->m_result = EvaluatePoint		( b, k, u );	break;
		case 'l': ch->m_result = EvaluateLinear		( b, k, u );	break;
		case 'b': ch->m_result = EvaluateBSpline	( b, k, u );	break;
		case 'c': ch->m_result = EvaluateCatmullRom ( b, k, u );	break;
		case 'z': ch->m_result = EvaluateBezier	    ( b, k, u );	break;
		};		
		b += 2;		// next channel buffers
	}
}

// Rotation interpolation
// - Spherical linear quaternions
Vector4DF Spline::EvaluateQSlerp ( int b, int k, float u )
{
	Quaternion* keyO = (Quaternion*) GetElem (b, k); 
	Quaternion q;
	/*if ( k >= m_num) 
		return *keyO;
	else
		return q.slerp(*keyO, *(keyO + 1), u);		// spherical linear interpolated rotation (SLERP) */

	return Vector4DF(0,0,0,0);
}

// - Point
Vector4DF Spline::EvaluatePoint ( int b, int k, float u )
{
	Vector4DF* key = GetElemVec4 (b, k);
	return *key;	
}

// Scale interpolation
// - Linear vector interpolation
Vector4DF Spline::EvaluateLinear ( int b, int k, float u )
{
	Vector4DF* key = GetElemVec4 (b, k);
	if (k >= m_num) 
		return *key;
	else
		return *key + (*(key + 1)-*(key))*u;		// linear interpolation
}

// Vector interpolation

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
Vector4DF Spline::EvaluateBSpline ( int b, int k, float u )
{
	float a;
	int p = m_degree;
	int ki = k + p;		// must offset knot index by p	
	Vector4DF* pos = GetElemVec4 ( b, 0 );

	//if ( m_num==1 ) return *pos;								// point
	//if ( m_num==2 ) return *pos + (*(pos+1) - *pos) * u;		// line

	int* knots =	(int*)			GetStart (SP_KNOTS);
	Vector4DF* d =	(Vector4DF*)	GetStart ( b+1 );

	// De Boor's algorithm, 1972, On Calculating with B-Splines, also see Wikipedia pseudo-code
	int i;
	for (int j = 0; j <= p; j++) {
		i = j + ki - p - 1; i = (i < 0) ? 0 : ((i >= m_num) ? m_num - 1 : i);
		*(d+j) = *(pos + i);
	}
	for (int r = 1; r <= p; r++) {
		for (int j = p; j >= r; j--) {
			a = ( (k+u) - *(knots+j + ki - p) ) / (*(knots+j + 1 + ki - r) - *(knots+j + ki - p) );
			//assert ( j >= 1 && j <= p );
			*(d+j) = *(d + j - 1) * (1 - a) + *(d+j) * a;
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
Vector4DF Spline::EvaluateCatmullRom( int b, int k, float u )
{
	Vector4DF* keys	= GetElemVec4 ( b, 0 );		// keys
	Vector4DF* mids = GetElemVec4 ( b+1, 0 );		// midpoints

	float u2 = u*u; float u3 = u2*u;
	
	float h00 = 2*u3 - 3*u2 +1;		// Hermite basis funcs
	float h10 = u3 - 2*u2 + u;
	float h01 = -2*u3 + 3*u2;
	float h11 = u3 - u2;
	Vector3DF kn1 = (k + 1 >= m_num) ? *(keys+m_num - 1) : *(keys+k + 1);

	return *(keys+k) * h00 + *(mids+k) * h10 + kn1 * h01 + *(mids+k + 1) * h11;
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
Vector4DF Spline::EvaluateBezier ( int b, int k, float u )
{
	Vector4DF* keys	= GetElemVec4 ( b, 0 );		// keys
	Vector4DF* tans = GetElemVec4 ( b+1, 0 );	// tangents

	float um = 1 - u;

	float b0 = um*um*um;
	float b1 = 3*u*um*um;
	float b2 = 3*u*u*um;
	Vector4DF kn1 = (k + 1 >= m_num) ? *(keys + m_num - 1) : *(keys + k + 1);

	return *(keys+k) * b0 + (*(keys+k) + *(tans+k)) * b1 + (kn1 - *(tans+k)) * b2 + kn1 * u * u * u;
}



