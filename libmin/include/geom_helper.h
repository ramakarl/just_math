
#ifndef GEOM_HELPER

	#define GEOM_HELPER

	#include "common_defs.h"
	#include "vec.h"
	class Camera3D;
	
	// Geometry utility functions	
	HELPAPI bool		intersectLineLine(Vector3DF p1, Vector3DF p2, Vector3DF p3, Vector3DF p4, Vector3DF& pa, Vector3DF& pb, double& mua, double& mub);
	HELPAPI Vector3DF	intersectLineLine(Vector3DF p1, Vector3DF p2, Vector3DF p3, Vector3DF p4);
	HELPAPI Vector3DF	intersectLineBox(Vector3DF p1, Vector3DF p2, Vector3DF bmin, Vector3DF bmax);
	HELPAPI Vector3DF	intersectLinePlane(Vector3DF p1, Vector3DF p2, Vector3DF p0, Vector3DF pnorm);
	HELPAPI bool		intersectRayTriangle ( Vector3DF orig, Vector3DF dir, Vector3DF& v0, Vector3DF& v1, Vector3DF& v2, float& t, Vector3DF& hit );
	HELPAPI Vector3DF	projectPointLine(Vector3DF p, Vector3DF p0, Vector3DF p1 );
	HELPAPI Vector3DF	projectPointLine(Vector3DF p, Vector3DF pdir, float& t );
	HELPAPI bool 		checkHit3D(Camera3D* cam, int x, int y, Vector3DF target, float radius);
	HELPAPI Vector3DF	moveHit3D(Camera3D* cam, int x, int y, Vector3DF target, Vector3DF plane_norm);
  
#endif