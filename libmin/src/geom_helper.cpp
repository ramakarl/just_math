
#include "geom_helper.h"
#include "vec.h"
#include "camera3d.h"

//----------------- Geometry utilities
//

#define EPS		0.000001

// Line A: p1 to p2
// Line B: p3 to p4
bool intersectLineLine(Vector3DF p1, Vector3DF p2, Vector3DF p3, Vector3DF p4, Vector3DF& pa, Vector3DF& pb, double& mua, double& mub)
{
	Vector3DF p13, p43, p21;
	double d1343, d4321, d1321, d4343, d2121;
	double numer, denom;

	p13 = p1;	p13 -= p3;
	p43 = p4;	p43 -= p3;
	if (fabs(p43.x) < EPS && fabs(p43.y) < EPS && fabs(p43.z) < EPS) return false;
	p21 = p2;	p21 -= p1;
	if (fabs(p21.x) < EPS && fabs(p21.y) < EPS && fabs(p21.z) < EPS) return false;

	d1343 = p13.Dot(p43);
	d4321 = p43.Dot(p21);
	d1321 = p13.Dot(p21);
	d4343 = p43.Dot(p43);
	d2121 = p21.Dot(p21);

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < EPS) return false;
	numer = d1343 * d4321 - d1321 * d4343;

	mua = numer / denom;
	mub = (d1343 + d4321 * (mua)) / d4343;

	pa = p21;	pa *= (float)mua;		pa += p1;
	pb = p43;	pb *= (float)mub;		pb += p3;

	return true;
}

Vector3DF intersectLineLine(Vector3DF p1, Vector3DF p2, Vector3DF p3, Vector3DF p4)
{
	Vector3DF pa, pb;
	double ma, mb;
	if (intersectLineLine(p1, p2, p3, p4, pa, pb, ma, mb)) {
		return pa;
	}
	return p2;
}

Vector3DF intersectLineBox(Vector3DF p1, Vector3DF p2, Vector3DF bmin, Vector3DF bmax)
{
	// p1 = ray position, p2 = ray direction
	register float ht[8];
	ht[0] = (bmin.x - p1.x)/p2.x;
	ht[1] = (bmax.x - p1.x)/p2.x;
	ht[2] = (bmin.y - p1.y)/p2.y;
	ht[3] = (bmax.y - p1.y)/p2.y;
	ht[4] = (bmin.z - p1.z)/p2.z;
	ht[5] = (bmax.z - p1.z)/p2.z;
	ht[6] = fmax(fmax(fmin(ht[0], ht[1]), fmin(ht[2], ht[3])), fmin(ht[4], ht[5]));
	ht[7] = fmin(fmin(fmax(ht[0], ht[1]), fmax(ht[2], ht[3])), fmax(ht[4], ht[5]));	
	ht[6] = (ht[6] < 0 ) ? 0.0 : ht[6];
	return Vector3DF( ht[6], ht[7], (ht[7]<ht[6] || ht[7]<0) ? -1 : 0 );
}

Vector3DF intersectLinePlane(Vector3DF p1, Vector3DF p2, Vector3DF p0, Vector3DF pnorm)
{
	Vector3DF u, w;
	u = p2;	u -= p1;					// ray direction
	w = p1;	w -= p0;

	float dval = pnorm.Dot(u);
	float nval = -pnorm.Dot(w);

	if (fabs(dval) < EPS) {			// segment is parallel to plane
		if (nval == 0) return p1;       // segment lies in plane
		else			return p1;      // no intersection
	}
	// they are not parallel, compute intersection
	float t = nval / dval;
	u *= t;
	u += p1;
	return u;
}

bool intersectRayTriangle ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& t, Vector3DF& hit )
{
	Vector3DF e1, e2, h, s, q;
	float a, u, v;

	// Moller-Trumbore algorithm
	e1 = v1 - v0;
	e2 = v2 - v0;
	h = h.Cross ( dir, e2 );		
	a = e1.Dot ( h );			// determinant
	if ( a > -EPS && a < EPS ) {t=0; return false;}
	a = 1.0/a;					// inv determinant
	s = orig - v0;
	u = a * s.Dot ( h );
	if ( u < 0.0 || u > 1.0 ) {t=0; return false;}
	q = q.Cross ( s, e1 );
	v = a * dir.Dot ( q );
	if ( v < 0.0 || u+v > 1.0) {t=0; return false;}
	
	t = a * e2.Dot ( q );
	//if ( t < EPS ) { t=0; return false; }
	hit = orig + dir * t;
	
	return true;
}

Vector3DF projectPointLine(Vector3DF p, Vector3DF p0, Vector3DF p1 )
{
	Vector3DF dir = p1-p0;
	return p0 + dir * float( (p-p0).Dot(dir) / dir.Dot(dir));
}
Vector3DF projectPointLine(Vector3DF p, Vector3DF dir, float& t )
{
	t = float(p.Dot(dir) / dir.Dot(dir));
	return dir * t;
}

bool checkHit3D(Camera3D* cam, int x, int y, Vector3DF target, float radius)
{
	Vector3DF dir = cam->inverseRay(x, y, cam->mXres, cam->mYres);	dir.Normalize();
	Vector3DF bmin = target - Vector3DF(radius*0.5f, radius*0.5, radius*0.5);
	Vector3DF bmax = target + Vector3DF(radius*0.5f, radius*0.5, radius*0.5);
	Vector3DF t = intersectLineBox(cam->getPos(), dir, bmin, bmax);
	return (t.z >= 0);
}

Vector3DF moveHit3D(Camera3D* cam, int x, int y, Vector3DF target, Vector3DF plane_norm)
{
	Vector3DF dir = cam->inverseRay(x, y, cam->mXres, cam->mYres);
	Vector3DF hit = intersectLinePlane(cam->getPos(), cam->getPos() + dir, target, plane_norm);
	return hit;	
}



