
// Copyright (C) 2002-2012 Nikolaus Gebhardt
// Modified 2021 by Rama Hoetzlein (C) 2021
// See quaternion.h for more details

#include "quaternion.h"

// matrix assignment operator
Quaternion& Quaternion::operator=(const Matrix4F& m)
{
	// 0  1  2  3    m00 m10 m20 m30
	// 4  5  6  7    m01 m11 m21 m31
	// 8  9  10 11   m02 m12 m22 m32
	// 12 13 14 15   m03 m13 m23 m33

	f64 T = m.data[0] + m.data[5] + m.data[10];			// trace of matrix (diagonal), T = m0,0 + m1,1 + m2,2
	f64 scale;
	if (T > 0.0f) {
		scale = sqrtf(T + 1.0) * 2.0f;	// get scale from diagonal
		X = (m.data[6] - m.data[9]) / scale;			// X = (m2,1 - m1,2) / s
		Y = (m.data[8] - m.data[2]) / scale;			// Y = (m0,2 - m2,0) / s
		Z = (m.data[1] - m.data[4]) / scale;			// Z = (m1,0 - m0,1) / s
		W = 0.25f * scale;
	}
	else {
		if (m.data[0] > m.data[5] && m.data[0] > m.data[10]) {		// 1st element of diag is greatest value
			scale = sqrtf(1.0f + m.data[0] - m.data[5] - m.data[10]) * 2.0f;	// s = sqrt(1 + m0,0 - m1,1 - m2,2) * 2			
			X = 0.25f * scale;	
			Y = (m.data[1] + m.data[4]) / scale;		// Y = (m1,0 + m0,1) / s
			Z = (m.data[8] + m.data[2]) / scale;		// Z = (m0,2 + m2,0) / s
			W = (m.data[6] - m.data[9]) / scale;		// W = (m2,1 - m1,2) / s
		}
		else if (m.data[5] > m.data[10]) {							// 2nd element of diag is greatest value
			scale = sqrtf(1.0f + m.data[5] - m.data[0] - m.data[10]) * 2.0f;	// s = sqrt(1 + m1,1 - m0,0 - m2,2) * 2
			X = (m.data[1] + m.data[4]) / scale;		// X = (m1,0 + m0,1) / s
			Y = 0.25f * scale;
			Z = (m.data[6] + m.data[9]) / scale;		// Z = (m2,1 + m1,2) / s
			W = (m.data[8] - m.data[2]) / scale;		// W = (m0,2 - m2,0) / s
		}
		else {													// 3rd element of diag is greatest value
			scale = sqrtf(1.0f + m.data[10] - m.data[0] - m.data[5]) * 2.0f;	// s = sqrt(1 + m2,2 - m0,0 - m1,1) * 2
			X = (m.data[8] + m.data[2]) / scale;		// X = (m0,2 + m2,0) / s
			Y = (m.data[6] + m.data[9]) / scale;		// Y = (m2,1 + m1,2) / s
			Z = 0.25f * scale;							
			W = (m.data[1] - m.data[4]) / scale;		// W = (m1,0 - m0,1) / s
		}
	}

	return normalize();
}

// multiplication operator
Quaternion Quaternion::operator*(const Quaternion& other) const
{
	Quaternion tmp;

	/*tmp.W = (other.W * W) - (other.X * X) - (other.Y * Y) - (other.Z * Z);
	tmp.X = (other.W * X) + (other.X * W) + (other.Y * Z) - (other.Z * Y);
	tmp.Y = (other.W * Y) + (other.Y * W) + (other.Z * X) - (other.X * Z);
	tmp.Z = (other.W * Z) + (other.Z * W) + (other.X * Y) - (other.Y * X);*/

	tmp.W = (W * other.W) - (X * other.X) - (Y * other.Y) - (Z * other.Z);
	tmp.X = (W * other.X) + (X * other.W) + (Y * other.Z) - (Z * other.Y);
	tmp.Y = (W * other.Y) + (Y * other.W) + (Z * other.X) - (X * other.Z);
	tmp.Z = (W * other.Z) + (Z * other.W) + (X * other.Y) - (Y * other.X);

	return tmp;
}

// multiplication operator
Quaternion Quaternion::operator*(f32 s) const
{
	return Quaternion(s * X, s * Y, s * Z, s * W);
}


// multiplication operator
Quaternion& Quaternion::operator*=(f32 s)
{
	X *= s;
	Y *= s;
	Z *= s;
	W *= s;
	return *this;
}

// multiplication operator
Quaternion& Quaternion::operator*=(const Quaternion& other)
{
	return (*this = other * (*this));
}

// multiplication with a vector, giving a rotated vector
Vector3DF Quaternion::operator* (const Vector3DF& v) const
{
	// nVidia SDK implementation

	Vector3DF uv, uuv;
	Vector3DF qvec; qvec.Set(float(X), float(Y), float(Z));
	uv = qvec.Cross(v);
	uuv = qvec.Cross(uv);
	uv *= (2.0f * float(W));
	uuv *= 2.0f;

	uuv += uv;
	uuv += v;
	return uuv;
}

// add operator
Quaternion Quaternion::operator+(const Quaternion& b) const
{
	return Quaternion(X + b.X, Y + b.Y, Z + b.Z, W + b.W);
}

// sets new Quaternion based on Euler angles
Quaternion& Quaternion::set(f32 x, f32 y, f32 z)
{
	f64 angle;

	angle = x * 0.5;
	const f64 sr = sin(angle);
	const f64 cr = cos(angle);

	angle = y * 0.5;
	const f64 sp = sin(angle);
	const f64 cp = cos(angle);

	angle = z * 0.5;
	const f64 sy = sin(angle);
	const f64 cy = cos(angle);

	const f64 cpcy = cp * cy;
	const f64 spcy = sp * cy;
	const f64 cpsy = cp * sy;
	const f64 spsy = sp * sy;

	X = (f32)(sr * cpcy - cr * spsy);
	Y = (f32)(cr * spcy + sr * cpsy);
	Z = (f32)(cr * cpsy - sr * spcy);
	W = (f32)(cr * cpcy + sr * spsy);

	return normalize();
}

// sets new Quaternion based on euler angles
Quaternion& Quaternion::set(const Vector3DF& vec)
{
	return set(vec.x * DEGtoRAD, vec.y * DEGtoRAD, vec.z * DEGtoRAD);
}

// Construct the dual portion of a dual-Quaternion (where 'this' is assumed to be the real/rotation part)			
Quaternion Quaternion::dual(Vector3DF pos)
{
  return Quaternion(pos, 0.0f) * (*this) * 0.5f;
	//return (Quaternion(pos, 0.0f) * (*this) * 0.5f);
}

//! Check Quaternion equals the other one (within floating point rounding tolerance)
bool Quaternion::equals(const Quaternion& other, const f32 tolerance) const
{
	return fequal(X, other.X, tolerance) &&
		fequal(Y, other.Y, tolerance) &&
		fequal(Z, other.Z, tolerance) &&
		fequal(W, other.W, tolerance);
}

void Quaternion::fixround()
{
	if (fabs(X) < ROUNDING_ERROR_f64) X = 0;
	if (fabs(Y) < ROUNDING_ERROR_f64) Y = 0;
	if (fabs(Z) < ROUNDING_ERROR_f64) Z = 0;
	if (fabs(W) < ROUNDING_ERROR_f64) W = 0;
}

// normalizes the Quaternion
Quaternion& Quaternion::normalize()
{
	const f32 n = X * X + Y * Y + Z * Z + W * W;
	if (n == 1) return *this;
	//n = 1.0f / sqrtf(n);
	return (*this *= fast_inv_squareroot(n));
}

// q^-1 = inverse = <-Qxyz, Qw> / ||Q||
Quaternion Quaternion::inverse()
{
	float N = fast_inv_squareroot(X*X+Y*Y+Z*Z+W*W);
	return Quaternion(-X*N, -Y*N, -Z*N, W*N);
}		

// q* = conjugate inverse
Quaternion Quaternion::conjugate()
{
	return Quaternion(-X, -Y, -Z, W);		// note: q* = q^-1 when N=||q||=1  (NOT WHEN W=1)
}

// dot product of two Quaternions
f32 Quaternion::dotProduct(const Quaternion& q2) const
{
	return (X * q2.X) + (Y * q2.Y) + (Z * q2.Z) + (W * q2.W);
}

// Quaternion from orthogonal basis
Quaternion& Quaternion::toBasis (Vector3DF a, Vector3DF b, Vector3DF c)
{
	float T = a.x + b.y + c.z;
	float s;
	if (T > 0) {
		float s = sqrt(T + 1) * 2.f;
		X = (b.z - c.y) / s;
		Y = (c.x - a.z) / s;
		Z = (a.y - b.x) / s;
		W = 0.25f * s;
	} else if ( a.x > b.y && a.x > c.z) {
		s = sqrt(1 + a.x - b.y - c.z) * 2;
		X = 0.25f * s;
		Y = (a.y + b.x) / s;
		Z = (c.x + a.z) / s;
		W = (b.z - c.y) / s;
	} else if (b.y > c.z) {
		s = sqrt(1 + b.y - a.x - c.z) * 2;
		X = (a.y + b.x) / s;
		Y = 0.25f * s;
		Z = (b.z + c.y) / s;
		W = (c.x - a.z) / s;
	} else {
		s = sqrt(1 + c.z - a.x - b.y) * 2;
		X = (c.x + a.z) / s;
		Y = (b.z + c.y) / s;
		Z = 0.25f * s;
		W = (a.y - b.x) / s;
	}
	normalize();
	return *this;
}

// Linear interpolation - set this Quaternion to the result of the linear interpolation
Quaternion& Quaternion::lerp(Quaternion q1, Quaternion q2, f32 time)
{
	const f32 scale = 1.0f - time;
	return (*this = (q1 * scale) + (q2 * time));
}


// Spherical interpolation - set this Quaternion to the result of the spherical interpolation
//
// def. SLERP(q0,q1) = q0 exp( u log(q0.conj q1))
//
Quaternion Quaternion::slerp(Quaternion q0, Quaternion q1, f32 u, f32 threshold)
{
	Quaternion q;
	q = q0 * ((q0.conjugate()*q1).log() * u).exp();
	return q;
	
	// *NOTE*: THIS CODE HAS BEEN FLOATING AROUND FOR SLERP. IT IS WRONG...
	/* ..
		const f32 theta = acosf(angle);
		const f32 invsintheta = 1.0f / sinf(theta);
		const f32 scale = sinf(theta * (1.0f - time)) * invsintheta;
		const f32 invscale = sinf(theta * time) * invsintheta;
		return (*this = (q1 * scale) + (q2 * invscale));
	*/
}

// Quaternion from angle (in radians) and axis (must be unit length)
Quaternion& Quaternion::fromAngleAxis(f32 angle, const Vector3DF& axis)
{
	angle *= 0.5f;
	const f32 fSin = sinf(angle);
	W = cosf(angle);
	X = fSin * axis.x;
	Y = fSin * axis.y;
	Z = fSin * axis.z;
	return *this;
}
Quaternion& Quaternion::changeAngle ( f32 angle )
{
	W = cosf(angle*0.5f);
	return *this;
}

// Return angle and axis from quaternion
void Quaternion::toAngleAxis(f32& angle, Vector3DF& axis) const
{
	const f32 scale = sqrtf(X * X + Y * Y + Z * Z);

	if (fabs(scale) < Q_EPS || W > 1.0f || W < -1.0f)
	{
		angle = 0.0f;
		axis.x = 0.0f;
		axis.y = 1.0f;
		axis.z = 0.0f;
	}
	else
	{
		const f32 invscale = 1.0 / scale;
		angle = 2.0f * acosf(W);
		axis.x = X * invscale;
		axis.y = Y * invscale;
		axis.z = Z * invscale;
	}
}

// Convert quaternion rotation to Euler (XYZ) angles
inline void Quaternion::toEuler(Vector3DF& euler) const
{
	const f64 test = (X * Y + Z * W);

	if (fequal(test, 0.5, 0.000001))
	{
		euler.x = 0;							// roll = rotation about x-axis (bank)
		euler.y = (f32)(PI64 / 2.0);			// pitch = rotation about y-axis (attitude)
		euler.z = (f32)(-2.0 * atan2(X, W));		// yaw = rotation about z-axis (heading)		
	}
	else if (fequal(test, -0.5, 0.000001))
	{
		// heading = rotation about z-axis
		euler.z = (f32)(2.0 * atan2(X, W));
		// bank = rotation about x-axis
		euler.x = 0;
		// attitude = rotation about y-axis
		euler.y = (f32)(PI64 / -2.0);
	}
	else
	{
		euler.x = (f32)atan2(2.0 * (X * W - Y * Z), 1.0 - 2.0f * (X * X + Z * Z));	// roll = rotation about x-axis (bank)
		euler.y = (f32)asin(2.0 * test);								// pitch = rotation about y-axis (attitude)
		euler.z = (f32)atan2(2.0 * (X * Z - Y * W), 1.0 - 2.0f * (Y * Y + Z * Z));	// yaw = rotation about z-axis (heading)
	}

	euler.x *= RADtoDEG;
	euler.y *= RADtoDEG;
	euler.z *= RADtoDEG;
}

// Rotate a vector using quaternion
Vector3DF Quaternion::rotateVec(Vector3DF p)
{
	//--- efficient way		
	Vector3DF u;
	u.Set(float(X), float(Y), float(Z));
	Vector3DF q;
	q = u * float(2.0f * u.Dot(p)) + p * float(W * W - u.Dot(u)) + q.Cross(u, p) * float(2.0f * W);		// Cross(u,p) - u and p preserved

	/*--- full quaterion-to-matrix solution (correct)
	Vector3DF q;
	q.x = p.x*(1.0f - 2.0f*Y*Y - 2.0f*Z*Z)		+ p.y*(2.0f*X*Y - 2.0f*Z*W)			+ p.z*(2.0f*X*Z + 2.0f*Y*W);
	q.y = p.x*(2.0f*X*Y + 2.0f*Z*W)				+ p.y*(1.0f - 2.0f*X*X - 2.0f*Z*Z)	+ p.z*(2.0f*Z*Y - 2.0f*X*W);
	q.z = p.x*(2.0f*X*Z - 2.0f*Y*W)				+ p.y*(2.0f*Z*Y + 2.0f*X*W)			+ p.z*(1.0f - 2.0f*X*X - 2.0f*Y*Y); */

	return q;
}

//---------------------------------------------------------------- rotation from one vector to another
// Based on Stan Melax's article in Game Programming Gems
// Copy, since cannot modify local
/*Vector3DF v0 = from;
Vector3DF v1 = to;
v0.Normalize();
v1.Normalize();

const f32 d = v0.Dot (v1);
if (d >= 1.0f) // If dot == 1, vectors are the same
{
	return makeIdentity();
}
else if (d <= -1.0f) // exactly opposite
{
	Vector3DF axis(1.0f, 0.f, 0.f);
	axis = axis.Cross(v0);
	if (axis.Length()==0)
	{
		axis.Set(0.f,1.f,0.f);
		axis.Cross(v0);
	}
	// same as fromAngleAxis(core::PI, axis).normalize();
	return set(axis.x, axis.y, axis.z, 0).normalize();
}

const f32 s = sqrtf( (1+d)*2 ); // optimize inv_sqrt
const f32 invs = 1.f / s;
const Vector3DF c = v0.Cross(v1)*invs;
return set(c.x, c.y, c.z, s * 0.5f).normalize();*/

//---------------------------------------------------------------- rotation from one vector to another
// ---- See rotationFromTo. This func does same thing.
/*Quaternion Quaternion::fromBasis(Vector3DF from, Vector3DF to)
{
	// *assumes* input vectors are all normalized
	Vector3DF v2;
	v2 = v2.Cross(from, to);
	v2.Normalize();											// cross product = axis of rotation, cross of 'from' and 'to' vectors
	Quaternion q1 = fromAngleAxis(acos(from.Dot(to)), v2);	// dot product = angle of rotation between 'from' and 'to' vectors

	(*this) = q1;

	return *this;
}*/


// Construct a rotation from one vector to another
// - caller must ensure from and to have been normalized
void Quaternion::rotationFromTo(Vector3DF from, Vector3DF to)
{
	Vector3DF axis = to;	
	axis.Cross( from, to );						// axis of rotation between vectors (from and to unmodified)
	axis.Normalize();							
	fromAngleAxis(acos(from.Dot(to)), axis);	// dot product = angle of rotation between 'from' and 'to' vectors
	normalize();								// normalize quaternion
}

// Creates a matrix from this Quaternion
Matrix4F Quaternion::getMatrix() const
{
	Matrix4F m;
	getMatrix(m);
	return m;
}

// Creates a matrix from this Quaternion
void Quaternion::getMatrix(Matrix4F& dest,
	const Vector3DF& center) const
{
	dest.data[0] = 1.0f - 2.0f * Y * Y - 2.0f * Z * Z;		// m0,0  (col,row)
	dest.data[1] = 2.0f * X * Y + 2.0f * Z * W;				// m1,0
	dest.data[2] = 2.0f * X * Z - 2.0f * Y * W;				
	dest.data[3] = 0.0f;

	dest.data[4] = 2.0f * X * Y - 2.0f * Z * W;				// m0,1 (col,row)
	dest.data[5] = 1.0f - 2.0f * X * X - 2.0f * Z * Z;
	dest.data[6] = 2.0f * Z * Y + 2.0f * X * W;
	dest.data[7] = 0.0f;

	dest.data[8] = 2.0f * X * Z + 2.0f * Y * W;
	dest.data[9] = 2.0f * Z * Y - 2.0f * X * W;
	dest.data[10] = 1.0f - 2.0f * X * X - 2.0f * Y * Y;
	dest.data[11] = 0.0f;

	dest.data[12] = center.x;
	dest.data[13] = center.y;
	dest.data[14] = center.z;
	dest.data[15] = 1.f;

	//dest.Identity();
}


/*!
	Creates a matrix from this Quaternion
	Rotate about a center point. Shortcut for
	core::Quaternion q;
	q.rotationFromTo(vin[i].Normal, forward);
	q.getMatrix(lookat, center);

	core::Matrix4F m2;
	m2.setInverseTranslation(center);
	lookat *= m2;
*/
void Quaternion::getMatrixCenter(Matrix4F& dest,
	const Vector3DF& center,
	const Vector3DF& translation) const
{
	dest.data[0] = 1.0f - 2.0f * Y * Y - 2.0f * Z * Z;
	dest.data[1] = 2.0f * X * Y + 2.0f * Z * W;
	dest.data[2] = 2.0f * X * Z - 2.0f * Y * W;
	dest.data[3] = 0.0f;

	dest.data[4] = 2.0f * X * Y - 2.0f * Z * W;
	dest.data[5] = 1.0f - 2.0f * X * X - 2.0f * Z * Z;
	dest.data[6] = 2.0f * Z * Y + 2.0f * X * W;
	dest.data[7] = 0.0f;

	dest.data[8] = 2.0f * X * Z + 2.0f * Y * W;
	dest.data[9] = 2.0f * Z * Y - 2.0f * X * W;
	dest.data[10] = 1.0f - 2.0f * X * X - 2.0f * Y * Y;
	dest.data[11] = 0.0f;

	//dest.setRotationCenter ( center, translation );
}

// Creates a matrix from this Quaternion
void Quaternion::getMatrix_transposed(Matrix4F& dest) const
{
	dest.data[0] = 1.0f - 2.0f * Y * Y - 2.0f * Z * Z;
	dest.data[4] = 2.0f * X * Y + 2.0f * Z * W;
	dest.data[8] = 2.0f * X * Z - 2.0f * Y * W;
	dest.data[12] = 0.0f;

	dest.data[1] = 2.0f * X * Y - 2.0f * Z * W;
	dest.data[5] = 1.0f - 2.0f * X * X - 2.0f * Z * Z;
	dest.data[9] = 2.0f * Z * Y + 2.0f * X * W;
	dest.data[13] = 0.0f;

	dest.data[2] = 2.0f * X * Z + 2.0f * Y * W;
	dest.data[6] = 2.0f * Z * Y - 2.0f * X * W;
	dest.data[10] = 1.0f - 2.0f * X * X - 2.0f * Y * Y;
	dest.data[14] = 0.0f;

	dest.data[3] = 0.f;
	dest.data[7] = 0.f;
	dest.data[11] = 0.f;
	dest.data[15] = 1.f;

	//dest.setDefinitelyIdentityMatrix(false);
}


Quaternion Quaternion::log()
{
	float b = sqrt(X*X + Y*Y + Z*Z);
	float v = atan2(b, W);
	float f = v / b;
	return Quaternion ( f*X, f*Y, f*Z, ::log(W * W + b * b) / 2.0f);
}

Quaternion Quaternion::unit_log()
{
	return Quaternion ( 0, 0, 0, ::log(W) );
}

Quaternion Quaternion::exp()
{
	float b = sqrt(X * X + Y * Y + Z * Z);
	float s = sin(b) / b;
	float e = ::exp(W);
	return Quaternion ( e*s*X, e*s*Y, e*s*Z, e * cos(b));
}
Quaternion Quaternion::unit_exp()
{
	return Quaternion ( 0, 0, 0, ::exp(W) );
}


