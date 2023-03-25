
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

bool intersectLineBox (Vector3DF p1, Vector3DF p2, Vector3DF bmin, Vector3DF bmax, float& t)
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
	if ( ht[7]<ht[6] || ht[7]<0 ) return false;
	t = ht[6];
	return true;
}

/*bool intersectLineBox (Vector3DF p1, Vector3DF p2, Vector3DF bmin, Vector3DF bmax, float& t)
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
	if (ht[7] < 0 || ht[6] > ht[7]) return false;
	t = ht[6];
	return true; 
}
*/

Vector3DF intersectLinePlane(Vector3DF rpos, Vector3DF rdir, Vector3DF p0, Vector3DF pnorm)
{
	float dval = pnorm.Dot( rdir );
	float nval = -pnorm.Dot( rpos - p0 );
	
	if (fabs(dval) < EPS) {			    // segment is parallel to plane
		if (nval == 0)  return rpos;    // segment lies in plane
		else			return rpos;    // no intersection
	}	
	float t = nval / dval;				// they are not parallel, compute intersection
	return (rpos + rdir * t);
}


//--- test where point projects within the inf prism of triangle
bool pointInTriangle(Vector3DF pnt, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, double& u, double& v)
{    
    Vector3DF e0 = v2 - v1;
    Vector3DF e1 = v0 - v2;
    Vector3DF n = e0.Cross ( e1 );
	float ndot = n.Dot(n);    
    // Barycentric coordinates of the projection P' of P onto T:
	u = e0.Cross(pnt-v1).Dot(n) / ndot;    
	v = e1.Cross(pnt-v2).Dot(n) / ndot;    
    // The point P' lies inside T if:
    return (u>=0.0 && v>=0.0 && (u+v<=1) );
}

// Ray-Triangle intersection
// returns front and back hits 
bool intersectRayTriangle ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& t, float& alpha, float& beta, bool& front )    
{
	Vector3DF e0 = v2 - v1;
    Vector3DF e1 = v2 - v0;
    Vector3DF n = e1.Cross ( e0 );
	float ndotr = n.Dot( dir );	
	Vector3DF e2 = (v2 - orig) / ndotr;
	Vector3DF i = dir.Cross ( e2 );
	alpha =	i.x*e0.x + i.y*e0.y + i.z*e0.z;	
	beta =	i.x*-e1.x + i.y*-e1.y + i.z*-e1.z;	
	t = n.Dot ( e2 );
	front = (ndotr<0);  // front-facing, triangle normal toward ray
	return (t>0.0 && alpha>=0.0 && beta>=0.0 && (alpha+beta<=1));
}

//----- Triangle intersection, slower methods
// GIVEN: e0,e1,n    
//-- Barycentric Inside-Outside algorithm
// Step 1: Find hit point
// check if the ray and plane are parallel. ndotr = normal dot ray direction
/*float ndotr = n.Dot (dir);
if (fabs(ndotr) < EPS) 
    return false; // they are parallel so they don't intersect! 
// compute t
float d = -n.Dot (v0);
t = -(n.Dot(orig) + d) / ndotr;
if (t < 0) return false; // the triangle is behind
// Barycentric Inside-Outside test    	    
u = e0.Cross(hit-v1).Dot(n) / ndot;    
v = e1.Cross(hit-v2).Dot(n) / ndot;
float a = (1-u-v);
return ((0 <= a) && (a <= 1) && (0 <= u)  && (u  <= 1) && (0 <= v) && (v <= 1));  */

//-- Moller-Trumbore algorithm	
/*a = e0.Dot ( h );			// determinant
if ( a > -EPS && a < EPS ) {t=0; return false;}
a = 1.0/a;					// inv determinant
q = orig - v0;
u = a * q.Dot ( h );
if ( u < 0.0 || u > 1.0 ) {t=0; return false;}
q = q.Cross ( e0 );
v = a * dir.Dot ( q );
if ( v < 0.0 || u+v > 1.0) {t=0; return false;}
t = -a * e1.Dot ( q );
if ( t < EPS ) { t=0; return false; } */




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

Vector3DF projectPointPlane(Vector3DF p, Vector3DF p0, Vector3DF pn )
{
	// plane: p0=orig, pn=normal	
	return p - pn * (float) pn.Dot( p - p0 );
}

float distancePointPlane(Vector3DF p, Vector3DF p0, Vector3DF pn )
{
	return pn.Dot( p - p0 );
}

bool checkHit3D(Camera3D* cam, int x, int y, Vector3DF target, float radius)
{
	Vector3DF dir = cam->inverseRay(x, y, cam->mXres, cam->mYres);	dir.Normalize();
	Vector3DF bmin = target - Vector3DF(radius*0.5f, radius*0.5, radius*0.5);
	Vector3DF bmax = target + Vector3DF(radius*0.5f, radius*0.5, radius*0.5);
	float t;
	return intersectLineBox(cam->getPos(), dir, bmin, bmax, t);
}

Vector3DF moveHit3D(Camera3D* cam, int x, int y, Vector3DF target, Vector3DF plane_norm)
{
	Vector3DF dir = cam->inverseRay(x, y, cam->mXres, cam->mYres);
	Vector3DF hit = intersectLinePlane(cam->getPos(), cam->getPos() + dir, target, plane_norm);
	return hit;	
}



