
//--------------------------------------------------
// Voxelizer
// Voxelization of a triangle
//
// A "Just Math" Demo
// by Rama Karl Hoetzlein (c) 2021
// License CC-BY.
//
//--------------------------------------------------

// Sample utils
#include <GL/glew.h>
#include <algorithm>

#include "main.h"			// window system 
#include "gxlib.h"			// rendering
using namespace glib;

#include "geom_helper.h"


#define V_EXPLICIT	0
#define V_SCHWARZ	1
#define V_AKENINE	2
#define V_RASTER	3

class Sample : public Application {
public:
	virtual void startup();
	virtual bool init();
	virtual void display();
	virtual void reshape(int w, int h);
	virtual void motion(AppEnum button, int x, int y, int dx, int dy);
	virtual void keyboard(int keycode, AppEnum action, int mods, int x, int y);
	virtual void mouse(AppEnum button, AppEnum state, int mods, int x, int y);
	virtual void mousewheel(int delta);
	virtual void shutdown();

	void		allocVoxels(int id, Vec3I res);
	void		clearVoxels(int id);
	void		setVoxel(int id, int x, int y, int z, char val);
	void		drawVoxels(int id);
	void		Voxelize();
	void		VoxelizeExplicit();
	void		VoxelizeSchwarzSeidel();
	void		VoxelizeAkenineMoller();
	void		VoxelizeRaster();
	void		fillTriangle();

	void		Reset();

	int			mouse_down;
	int			mouse_action;
	int			m_adjust, m_voxshow;

	Camera3D*	m_cam;
	Vec3F	m_goal;

	Vec3F	m_vmin, m_vmax;
	Vec3I	m_vres;

	Vec3F	tri[3];

	char* vox[5];

	int			m_voxel_visit, m_voxel_set, m_voxel_count;
};
Sample obj;


bool Sample::init()
{
	int w = getWidth(), h = getHeight();			// window width & height
	for (int n = 0; n < 5; n++) vox[n] = 0x0;

	addSearchPath(ASSET_PATH);
	init2D("arial");	
	setTextSz ( 16, 1);

	glViewport(0, 0, w, h);

	m_cam = new Camera3D;
	m_cam->setNearFar(1, 2000);
	m_cam->SetOrbit(Vec3F(-40, 30, 0), Vec3F(32, 16, 32), 120, 1);
	m_adjust = 0;

	// create triangle
	tri[0].Set(10.4, 3.2, 4);
	tri[1].Set(46.1, 38.5, 10.7);
	tri[2].Set(48.1, 8.6, 56.2);

	// create volume
	m_vmin.Set(0, 0, 0);
	m_vmax.Set(64, 64, 64);
	m_vres.Set(32, 32, 32);

	allocVoxels(V_EXPLICIT, m_vres);
	allocVoxels(V_SCHWARZ, m_vres);
	allocVoxels(V_AKENINE, m_vres);
	allocVoxels(V_RASTER, m_vres);

	// voxelize triangle into volume
	m_voxshow = V_RASTER;
	Voxelize();

	return true;
}

void Sample::allocVoxels(int id, Vec3I res)
{
	if (vox[id] != 0x0) free(vox[id]);
	vox[id] = (char*)malloc(res.x * res.y * res.z * sizeof(char));
	m_vres = res;
	clearVoxels(id);
}

void Sample::clearVoxels(int id)
{
	memset(vox[id], 0, m_vres.x * m_vres.y * m_vres.z * sizeof(char));
}

void Sample::setVoxel(int id, int x, int y, int z, char val)
{
	if (x < 0 || y < 0 || z < 0 || x >= m_vres.x || y >= m_vres.y || z >= m_vres.z) return;
	m_voxel_set++;
	char* v = vox[id];
	*(v + (z * m_vres.y + y) * m_vres.x + x) = val;
}
void Sample::drawVoxels(int id)
{
	Vec3F p, d;
	Vec3F o(0.01, 0.01, 0.01f);
	d = (m_vmax - m_vmin) / m_vres;

	m_voxel_count = 0;
	int i = 0;
	char* v = vox[id];
	for (int z = 0; z < m_vres.z; z++)
		for (int y = 0; y < m_vres.y; y++)
			for (int x = 0; x < m_vres.x; x++) {
				i = (z * m_vres.y + y) * m_vres.x + x;
				p.Set(m_vmin.x + x * (m_vmax.x - m_vmin.x) / m_vres.x, m_vmin.y + y * (m_vmax.y - m_vmin.y) / m_vres.y, m_vmin.z + z * (m_vmax.z - m_vmin.z) / m_vres.z);
				if (v[i] != 0) {
					m_voxel_count++;
					drawCube3D( p + o, p + d - o, Vec4F(float(x)/m_vres.x, float(y)/m_vres.y, float(z)/m_vres.z, 1.0) );
				}
			}
}


Vec3F fabs3(Vec3F f)
{
	return Vec3F(abs(f.x), abs(f.y), abs(f.z));
}
Vec3F floor3(Vec3F f)
{
	return Vec3F(floor(f.x), floor(f.y), floor(f.z));
}

void Sample::fillTriangle()
{
	// note:
	// this triangle will appear outside the voxelized region.
	// inside the voxelized region, if conservative voxelization is supported, it should be fully contain inside voxels.

	int p0, p1, p2, p3, axis;
	float t1, t2, t, tend;
	float dt1, dt2;
	Vec3F e1, e2, q1, q2, a, b;
	Vec3F v[3];

	// transform to unit grid
	v[0] = (tri[0] - m_vmin) * m_vres / (m_vmax - m_vmin);
	v[1] = (tri[1] - m_vmin) * m_vres / (m_vmax - m_vmin);
	v[2] = (tri[2] - m_vmin) * m_vres / (m_vmax - m_vmin);

	// sort vertices
	p0 = 0; p1 = 1; p2 = 2;
	if (v[p0].y > v[p1].y) { p3 = p0; p0 = p1; p1 = p3; }
	if (v[p0].y > v[p2].y) { p3 = p0; p0 = p2; p2 = p3; }
	if (v[p1].y > v[p2].y) { p3 = p1; p1 = p2; p2 = p3; }

	// edge increments
	e1 = v[p1] - v[p0];
	e2 = v[p2] - v[p0];
	dt1 = 0.025 / (e1.y);
	dt2 = 0.025 / (e2.y);
	t1 = 0; t2 = 0;

	for (float y = v[p0].y; y < v[p1].y; y += 0.025) {
		a = (v[p0] + e1 * t1);
		b = (v[p0] + e2 * t2);
		q1 = m_vmin + a * (m_vmax - m_vmin) / m_vres;
		q2 = m_vmin + b * (m_vmax - m_vmin) / m_vres;
		drawLine3D( q1, q2, Vec4F(1,0,0,1) );
		t1 += dt1;
		t2 += dt2;
	}

	e1 = v[p2] - v[p1];
	dt1 = 0.025 / (e1.y);
	t1 = 0;
	for (float y = v[p1].y; y < v[p2].y; y += 0.025) {
		a = (v[p1] + e1 * t1);
		b = (v[p0] + e2 * t2);
		q1 = m_vmin + a * (m_vmax - m_vmin) / m_vres;
		q2 = m_vmin + b * (m_vmax - m_vmin) / m_vres;
		drawLine3D( q1, q2, Vec4F(1,0,0,1) );
		t1 += dt1;
		t2 += dt2;
	}
}


void Sample::VoxelizeRaster()
{
	clearVoxels(V_RASTER);

	int p0, p1, p2, p3, axis;			// vertex sorting
	float t, tend, vend;
	Vec3F a, b, d1, d2;				// edge variables	
	Vec3F c, d, dstep, mask, s;		// 2DDA variables
	Vec3F v[3];

	// transform to unit voxel space
	v[0] = (tri[0] - m_vmin) * m_vres / (m_vmax - m_vmin);
	v[1] = (tri[1] - m_vmin) * m_vres / (m_vmax - m_vmin);
	v[2] = (tri[2] - m_vmin) * m_vres / (m_vmax - m_vmin);
	d = d.Cross(v[1] - v[0], v[2] - v[0]);
	d.Normalize();

	// determine raster axis
	axis = fabs(d.x) > fabs(d.y) ? (fabs(d.x) > fabs(d.z) ? 0 : 2) : (fabs(d.y) > fabs(d.z)) ? 1 : 2;

	// sort vertices
	p0 = 0; p1 = 1; p2 = 2;
	if (axis == 1) {
		// use x axis (triangle oriented towad y)
		if (v[p0].x > v[p1].x) { p3 = p0; p0 = p1; p1 = p3; }
		if (v[p0].x > v[p2].x) { p3 = p0; p0 = p2; p2 = p3; }
		if (v[p1].x > v[p2].x) { p3 = p1; p1 = p2; p2 = p3; }

		// edge setup
		a = v[p0]; b = v[p0];
		d1 = (v[p1] - v[p0]) / (v[p1].x - v[p0].x);
		d2 = (v[p2] - v[p0]) / (v[p2].x - v[p0].x);
		a = a + d1 * float(1.0 - v[p0].x + floor(v[p0].x));
		b = b + d2 * float(1.0 - v[p0].x + floor(v[p0].x));
		vend = v[p1].x;

		// rasterize	
		for (; b.x < v[p2].x; ) {

			// prepare 2DDA
			t = 0; c = a;
			d = b - c; tend = d.Length(); d.Normalize();		// scan vector
			d.x = 1;
			dstep.Set((d.x > 0) ? 1 : -1, (d.y > 0) ? 1 : -1, (d.z > 0) ? 1 : -1);	// signed direction	
			d = fabs3(d);
			s.Set(0,
				((int(c.y) - c.y + 0.5f) * dstep.y + 0.5) / d.y + t,
				((int(c.z) - c.z + 0.5f) * dstep.z + 0.5) / d.z + t);

			// 2DDA in the YZ-plane
			while (t < tend) {
				setVoxel(V_RASTER, c.x + 0.1, c.y, c.z, 1);
				setVoxel(V_RASTER, c.x - 0.1, c.y, c.z, 1);
				mask = (s.y < s.z) ? Vec3I(0, 1, 0) : Vec3I(0, 0, 1);	// choose next voxel
				t = mask.y ? s.y : s.z;				// advance t
				s += mask / d;						// advance x/y/z intercepts by the inverse normal (not obvious)
				c += mask * dstep;					// advance to next voxel
			}
			c = b;
			setVoxel(V_RASTER, c.x + 0.1, c.y, c.z, 1);
			setVoxel(V_RASTER, c.x - 0.1, c.y, c.z, 1);

			// advance along edges
			a += d1;
			b += d2;
			if (a.x >= vend) {			// transition for third edge				
				vend = a.x - v[p1].x;	// measure amount we overshot
				a -= d1 * vend;			// back up to corner
				d1 = (v[p2] - v[p1]) / (v[p2].x - v[p1].x);
				a += d1 * vend;			// advance to y-unit along new edge
				vend = v[p2].x;
			}
		}


	}
	else {
		// use y axis (triangle oriented toward x or z)
		if (v[p0].y > v[p1].y) { p3 = p0; p0 = p1; p1 = p3; }
		if (v[p0].y > v[p2].y) { p3 = p0; p0 = p2; p2 = p3; }
		if (v[p1].y > v[p2].y) { p3 = p1; p1 = p2; p2 = p3; }

		// edge setup
		a = v[p0]; b = v[p0];
		d1 = (v[p1] - v[p0]) / (v[p1].y - v[p0].y);
		d2 = (v[p2] - v[p0]) / (v[p2].y - v[p0].y);
		a = a + d1 * float(1.0 - v[p0].y + floor(v[p0].y));
		b = b + d2 * float(1.0 - v[p0].y + floor(v[p0].y));
		vend = v[p1].y;

		// rasterize	
		for (; b.y < v[p2].y; ) {

			// prepare 2DDA
			t = 0; c = a;
			d = b - c; tend = d.Length(); d.Normalize();		// scan vector
			d.y = 1;
			dstep.Set((d.x > 0) ? 1 : -1, (d.y > 0) ? 1 : -1, (d.z > 0) ? 1 : -1);	// signed direction	
			d = fabs3(d);
			s.Set(((int(c.x) - c.x + 0.5f) * dstep.x + 0.5) / d.x + t, 0,
				((int(c.z) - c.z + 0.5f) * dstep.z + 0.5) / d.z + t);

			// 2DDA in the XZ-plane
			while (t < tend) {
				setVoxel(V_RASTER, c.x, c.y + 0.1, c.z, 1);
				setVoxel(V_RASTER, c.x, c.y - 0.1, c.z, 1);
				mask = (s.x < s.z) ? Vec3I(1, 0, 0) : Vec3I(0, 0, 1);	// choose next voxel
				t = mask.x ? s.x : s.z;				// advance t
				s += mask / d;						// advance x/y/z intercepts by the inverse normal (not obvious)
				c += mask * dstep;					// advance to next voxel
			}
			c = b;
			setVoxel(V_RASTER, c.x, c.y + 0.1, c.z, 1);
			setVoxel(V_RASTER, c.x, c.y - 0.1, c.z, 1);

			// advance along edges
			a += d1;
			b += d2;
			if (a.y >= vend) {			// transition for third edge			
				vend = a.y - v[p1].y;	// measure amount we overshot
				a -= d1 * vend;			// back up to corner
				d1 = (v[p2] - v[p1]) / (v[p2].y - v[p1].y);
				a += d1 * vend;			// advance to y-unit along new edge
				vend = v[p2].y;
			}
		}
	}

	m_voxel_visit = m_voxel_set;
}

bool explicit_test(Vec3F vmin, Vec3F vmax, Vec3F* tri)
{
	Vec3F v[3], e[3], norm;
	Vec3F p, n;
	float min, max, rad;

	// triangle normalized to test cube
	v[0] = (tri[0] - vmin) / (vmax - vmin);
	v[1] = (tri[1] - vmin) / (vmax - vmin);
	v[2] = (tri[2] - vmin) / (vmax - vmin);
	e[0] = v[1] - v[0];	// triangle edges
	e[1] = v[2] - v[1];
	e[2] = v[0] - v[2];
	norm = norm.Cross(e[0], e[1]);
	norm.Normalize();

	//-- fast box-plane test
	float r = 0.5 * fabs(norm.x) + 0.5 * fabs(norm.y) + 0.5 * fabs(norm.z);
	float s = norm.x * (0.5f - v[0].x) + norm.y * (0.5f - v[0].y) + norm.z * (0.5f - v[0].z);
	if (fabs(s) > r) return false;

	//-- explicit edge tests
	// ops: (6*4+7)*3*3+3 = 282 ops
	float k[4];
	Vec3F c((norm.x >= 0) ? 1 : -1, (norm.y >= 0) ? 1 : -1, (norm.z >= 0) ? 1 : -1);
	for (int i = 0; i < 3; i++) {
		k[0] = ((0 - v[i].x) * e[i].y - (0 - v[i].y) * e[i].x) * c.z;
		k[1] = ((0 - v[i].x) * e[i].y - (1 - v[i].y) * e[i].x) * c.z;
		k[2] = ((1 - v[i].x) * e[i].y - (0 - v[i].y) * e[i].x) * c.z;
		k[3] = ((1 - v[i].x) * e[i].y - (1 - v[i].y) * e[i].x) * c.z;
		if (k[0] > 0 && k[1] > 0 && k[2] > 0 && k[3] > 0) return false;
	}
	for (int i = 0; i < 3; i++) {
		k[0] = ((0 - v[i].z) * e[i].x - (0 - v[i].x) * e[i].z) * c.y;
		k[1] = ((0 - v[i].z) * e[i].x - (1 - v[i].x) * e[i].z) * c.y;
		k[2] = ((1 - v[i].z) * e[i].x - (0 - v[i].x) * e[i].z) * c.y;
		k[3] = ((1 - v[i].z) * e[i].x - (1 - v[i].x) * e[i].z) * c.y;
		if (k[0] > 0 && k[1] > 0 && k[2] > 0 && k[3] > 0) return false;
	}
	for (int i = 0; i < 3; i++) {
		k[0] = ((0 - v[i].y) * e[i].z - (0 - v[i].z) * e[i].y) * c.x;
		k[1] = ((0 - v[i].y) * e[i].z - (1 - v[i].z) * e[i].y) * c.x;
		k[2] = ((1 - v[i].y) * e[i].z - (0 - v[i].z) * e[i].y) * c.x;
		k[3] = ((1 - v[i].y) * e[i].z - (1 - v[i].z) * e[i].y) * c.x;
		if (k[0] > 0 && k[1] > 0 && k[2] > 0 && k[3] > 0) return false;
	}
	return true;
}

void Sample::VoxelizeExplicit()
{
	Vec3F p, d;
	float r;

	clearVoxels(V_EXPLICIT);

	for (int z = 0; z < m_vres.z; z++)
		for (int y = 0; y < m_vres.y; y++)
			for (int x = 0; x < m_vres.x; x++) {
				p.Set(m_vmin.x + x * (m_vmax.x - m_vmin.x) / m_vres.x, m_vmin.y + y * (m_vmax.y - m_vmin.y) / m_vres.y, m_vmin.z + z * (m_vmax.z - m_vmin.z) / m_vres.z);
				d = (m_vmax - m_vmin) / m_vres;

				if (explicit_test(p, p + d, &tri[0]))
					setVoxel(V_EXPLICIT, x, y, z, 1);
			}

	m_voxel_visit = m_vres.x * m_vres.y * m_vres.z;
}

bool schwarz_seidel_test(Vec3F vmin, Vec3F vmax, Vec3F* tri)
{
	Vec3F v[3], e[3], norm;
	Vec3F p, n;
	float min, max, rad;

	// triangle normalized to test cube
	v[0] = (tri[0] - vmin) / (vmax - vmin);
	v[1] = (tri[1] - vmin) / (vmax - vmin);
	v[2] = (tri[2] - vmin) / (vmax - vmin);
	e[0] = v[1] - v[0];	// triangle edges
	e[1] = v[2] - v[1];
	e[2] = v[0] - v[2];
	norm = norm.Cross(e[0], e[1]);
	norm.Normalize();

	//-- fast box-plane test
	float r = 0.5 * fabs(norm.x) + 0.5 * fabs(norm.y) + 0.5 * fabs(norm.z);
	float s = norm.x * (0.5f - v[0].x) + norm.y * (0.5f - v[0].y) + norm.z * (0.5f - v[0].z);
	if (fabs(s) > r) return false;

	//-- schwarz-seidel tests
	// ops: 12*9+3 = 111 ops
	Vec3F c((norm.x >= 0) ? 1 : -1, (norm.y >= 0) ? 1 : -1, (norm.z >= 0) ? 1 : -1);
	if (-(-e[0].y * v[0].x + e[0].x * v[0].y) * c.z + fmaxf(0, -e[0].y * c.z) + fmaxf(0, e[0].x * c.z) < 0) return false;
	if (-(-e[1].y * v[1].x + e[1].x * v[1].y) * c.z + fmaxf(0, -e[1].y * c.z) + fmaxf(0, e[1].x * c.z) < 0) return false;
	if (-(-e[2].y * v[2].x + e[2].x * v[2].y) * c.z + fmaxf(0, -e[2].y * c.z) + fmaxf(0, e[2].x * c.z) < 0) return false;
	if (-(-e[0].x * v[0].z + e[0].z * v[0].x) * c.y + fmaxf(0, -e[0].x * c.y) + fmaxf(0, e[0].z * c.y) < 0) return false;
	if (-(-e[1].x * v[1].z + e[1].z * v[1].x) * c.y + fmaxf(0, -e[1].x * c.y) + fmaxf(0, e[1].z * c.y) < 0) return false;
	if (-(-e[2].x * v[2].z + e[2].z * v[2].x) * c.y + fmaxf(0, -e[2].x * c.y) + fmaxf(0, e[2].z * c.y) < 0) return false;
	if (-(-e[0].z * v[0].y + e[0].y * v[0].z) * c.x + fmaxf(0, -e[0].z * c.x) + fmaxf(0, e[0].y * c.x) < 0) return false;
	if (-(-e[1].z * v[1].y + e[1].y * v[1].z) * c.x + fmaxf(0, -e[1].z * c.x) + fmaxf(0, e[1].y * c.x) < 0) return false;
	if (-(-e[2].z * v[2].y + e[2].y * v[2].z) * c.x + fmaxf(0, -e[2].z * c.x) + fmaxf(0, e[2].y * c.x) < 0) return false;

	return true;
}

void Sample::VoxelizeSchwarzSeidel()
{
	Vec3F p, d;
	float r;

	clearVoxels(V_SCHWARZ);

	for (int z = 0; z < m_vres.z; z++)
		for (int y = 0; y < m_vres.y; y++)
			for (int x = 0; x < m_vres.x; x++) {
				p.Set(m_vmin.x + x * (m_vmax.x - m_vmin.x) / m_vres.x, m_vmin.y + y * (m_vmax.y - m_vmin.y) / m_vres.y, m_vmin.z + z * (m_vmax.z - m_vmin.z) / m_vres.z);
				d = (m_vmax - m_vmin) / m_vres;

				if (schwarz_seidel_test(p, p + d, &tri[0]))
					setVoxel(V_SCHWARZ, x, y, z, 1);
			}

	m_voxel_visit = m_vres.x * m_vres.y * m_vres.z;
}

bool akenine_moller_test(Vec3F vmin, Vec3F vmax, Vec3F* tri)
{
	Vec3F v[3], e[3], norm;
	Vec3F p, n;
	float min, max, rad;

	// triangle normalized to test cube
	v[0] = (tri[0] - vmin) / (vmax - vmin);
	v[1] = (tri[1] - vmin) / (vmax - vmin);
	v[2] = (tri[2] - vmin) / (vmax - vmin);
	e[0] = v[1] - v[0];	// triangle edges
	e[1] = v[2] - v[1];
	e[2] = v[0] - v[2];
	norm = norm.Cross(e[0], e[1]);
	norm.Normalize();

	//-- fast box-plane test
	float r = 0.5 * fabs(norm.x) + 0.5 * fabs(norm.y) + 0.5 * fabs(norm.z);
	float s = norm.x * (0.5f - v[0].x) + norm.y * (0.5f - v[0].y) + norm.z * (0.5f - v[0].z);
	if (fabs(s) > r) return false;

	//--- akenine-moller tests
	p.x = e[0].z * v[0].y - e[0].y * v[0].z;
	p.z = e[0].z * v[2].y - e[0].y * v[2].z;
	if (p.x < p.z) { min = p.x; max = p.z; }
	else { min = p.z; max = p.x; }
	rad = fabsf(e[0].z) * 0.5f + fabsf(e[0].y) * 0.5f; if (min > rad || max < -rad) return false;

	p.x = -e[0].z * v[0].x + e[0].x * v[0].z;
	p.z = -e[0].z * v[2].x + e[0].x * v[2].z;
	if (p.x < p.z) { min = p.x; max = p.z; }
	else { min = p.z; max = p.x; }
	rad = fabsf(e[0].z) * 0.5f + fabsf(e[0].x) * 0.5f; if (min > rad || max < -rad) return false;

	p.y = e[0].y * v[1].x - e[0].x * v[1].y;
	p.z = e[0].y * v[2].x - e[0].x * v[2].y;
	if (p.z < p.y) { min = p.z; max = p.y; }
	else { min = p.y; max = p.z; }
	rad = fabsf(e[0].y) * 0.5f + fabsf(e[0].x) * 0.5f; if (min > rad || max < -rad) return false;

	p.x = e[1].z * v[0].y - e[1].y * v[0].z;
	p.z = e[1].z * v[2].y - e[1].y * v[2].z;
	if (p.x < p.z) { min = p.x; max = p.z; }
	else { min = p.z; max = p.x; }
	rad = fabsf(e[1].z) * 0.5f + fabsf(e[1].y) * 0.5f; if (min > rad || max < -rad) return false;

	p.x = -e[1].z * v[0].x + e[1].x * v[0].z;
	p.z = -e[1].z * v[2].x + e[1].x * v[2].z;
	if (p.x < p.z) { min = p.x; max = p.z; }
	else { min = p.z; max = p.x; }
	rad = fabsf(e[1].z) * 0.5f + fabsf(e[1].x) * 0.5f; if (min > rad || max < -rad) return false;

	p.x = e[1].y * v[0].x - e[1].x * v[0].y;
	p.y = e[1].y * v[1].x - e[1].x * v[1].y;
	if (p.x < p.y) { min = p.x; max = p.y; }
	else { min = p.y; max = p.x; }
	rad = fabsf(e[1].y) * 0.5f + fabsf(e[1].x) * 0.5f; if (min > rad || max < -rad) return false;

	p.x = e[2].z * v[0].y - e[2].y * v[0].z;
	p.y = e[2].z * v[1].y - e[2].y * v[1].z;
	if (p.x < p.y) { min = p.x; max = p.y; }
	else { min = p.y; max = p.x; }
	rad = fabsf(e[2].z) * 0.5f + fabsf(e[2].y) * 0.5f; if (min > rad || max < -rad) return false;

	p.x = -e[2].z * v[0].x + e[2].x * v[0].z;
	p.y = -e[2].z * v[1].x + e[2].x * v[1].z;
	if (p.x < p.y) { min = p.x; max = p.y; }
	else { min = p.y; max = p.x; }
	rad = fabsf(e[2].z) * 0.5f + fabsf(e[2].x) * 0.5f; if (min > rad || max < -rad) return false;

	p.y = e[2].y * v[1].x - e[2].x * v[1].y;
	p.z = e[2].y * v[2].x - e[2].x * v[2].y;
	if (p.z < p.y) { min = p.z; max = p.y; }
	else { min = p.y; max = p.z; }
	rad = fabsf(e[2].y) * 0.5f + fabsf(e[2].x) * 0.5f; if (min > rad || max < -rad) return false;

	return true;
}

void Sample::VoxelizeAkenineMoller()
{
	Vec3F p, d;
	float r;

	clearVoxels(V_AKENINE);

	for (int z = 0; z < m_vres.z; z++)
		for (int y = 0; y < m_vres.y; y++)
			for (int x = 0; x < m_vres.x; x++) {
				p.Set(m_vmin.x + x * (m_vmax.x - m_vmin.x) / m_vres.x, m_vmin.y + y * (m_vmax.y - m_vmin.y) / m_vres.y, m_vmin.z + z * (m_vmax.z - m_vmin.z) / m_vres.z);
				d = (m_vmax - m_vmin) / m_vres;

				if (akenine_moller_test(p, p + d, &tri[0]))
					setVoxel(V_AKENINE, x, y, z, 1);
			}

	m_voxel_visit = m_vres.x * m_vres.y * m_vres.z;
}

void Sample::Voxelize()
{
	m_voxel_visit = 0;
	m_voxel_set = 0;
	m_voxel_count = 0;

	switch (m_voxshow) {
	case V_EXPLICIT:	VoxelizeExplicit();			break;
	case V_SCHWARZ:		VoxelizeSchwarzSeidel();	break;
	case V_AKENINE:		VoxelizeAkenineMoller();	break;
	case V_RASTER:		VoxelizeRaster();			break;
	}
}

void Sample::display()
{
	Vec3F a, b, c;
	Vec3F p, q, d;
	int w =getWidth(), h=getHeight();

	clearGL();

	// draw in 3D
	start3D(m_cam);
	  setLight3D ( Vec3F(20, 100, 20), Vec4F(1,1,1,1) );
    setMaterial ( Vec3F(0.1,0.1,0.1), Vec3F(1,1,1), Vec3F(0.4,0.4,0.4), 40, 1 );

	  // sketch a grid
	  for (int z = 0; z < m_vres.z; z++)
		  for (int x = 0; x < m_vres.x; x++) {
			  p.Set(m_vmin.x + x * (m_vmax.x - m_vmin.x) / m_vres.x, 0, m_vmin.z + z * (m_vmax.z - m_vmin.z) / m_vres.z);
			  d = (m_vmax - m_vmin) / m_vres; d.y = 0;
			  drawBox3D (p, p + d, Vec4F(1, 1, 1, 0.2) );
		  }

	  // world axes
	  a = Vec3F(5, 0, 0); b = Vec3F(0, 5, 0); c = Vec3F(0, 0, 5);
	  p = Vec3F(0.f, 0.1f, 0.f);
	  drawLine3D( p, a, Vec4F(1,0,0,1));
	  drawLine3D( p, b, Vec4F(0,1,0,1));
	  drawLine3D( p, c, Vec4F(0,0,1,1));


	  // triangle fill - helpful to see if conservative voxelization is correct (red triangle should not be visible)
	  fillTriangle();

	  // draw triangle to be voxelized
	  drawLine3D(tri[0], tri[1], Vec4F(1,0,0,1));		// edges
	  drawLine3D(tri[1], tri[2], Vec4F(1,0,0,1));
	  drawLine3D(tri[2], tri[0], Vec4F(1,0,0,1));

	  // draw voxels
	  drawVoxels(m_voxshow);
	
	  drawCircle3D (tri[0], 2.0, Vec4F(1, 1, (m_adjust == 0) ? 1 : 0, 1) );	// corners of triangle (interactive)
	  drawCircle3D (tri[1], 2.0, Vec4F(1, 1, (m_adjust == 1) ? 1 : 0, 1) );
	  drawCircle3D (tri[2], 2.0, Vec4F(1, 1, (m_adjust == 2) ? 1 : 0, 1) );

	end3D();

	// draw text
	start2D(w, h);	
	setTextSz(16, 1);

	char buf[1024];
	char* msg;
	switch (m_voxshow) {
	case V_EXPLICIT:	msg = "Explicit Edge Tests";	break;
	case V_SCHWARZ:		msg = "Schwarz-Seidel";		break;
	case V_AKENINE:		msg = "Akenine-Moller";		break;
	case V_RASTER:		msg = "Rasterizer";			break;
	}

	sprintf(buf, "Algorithm (press 1,2,3,4): %s", msg);							drawText( Vec2F(10, 30), buf, Vec4F(1,1,1,1) );
	sprintf(buf, "Move the yellow vertices with left-click drag.");	drawText( Vec2F(10, 50), buf, Vec4F(1,1,1,1) );
	sprintf(buf, "Rotate the camera with right drag.");							drawText( Vec2F(10, 70), buf, Vec4F(1,1,1,1) );
	sprintf(buf, "Voxels total:    %d", m_vres.x * m_vres.y * m_vres.z);	drawText( Vec2F(10, 100), buf, Vec4F(1,1,1,1) );
	sprintf(buf, "Voxels visited:  %d", m_voxel_visit);										drawText( Vec2F(10, 120), buf, Vec4F(1,1,1,1) );
	sprintf(buf, "Voxels set:      %d", m_voxel_set);											drawText( Vec2F(10, 140), buf, Vec4F(1,1,1,1) );
	sprintf(buf, "Voxels active:   %d", m_voxel_count);										drawText( Vec2F(10, 160), buf, Vec4F(1,1,1,1) );
	end2D();

	// render with opengl
	drawAll();

	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
	float fine = 0.5;

	switch (mouse_down) {
	case AppEnum::BUTTON_LEFT: {

		if ( m_adjust >= 0  ) {
			// move triangle point in xz
			tri[m_adjust] = moveHit3D(m_cam, x, y, tri[m_adjust], Vec3F(0, 1, 0));	// xz plane			
			Voxelize();
		}
		
		appPostRedisplay();	// Update display
	} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
		appPostRedisplay();	// Update display
	} break;

	case AppEnum::BUTTON_RIGHT: {		
		
		if (m_adjust >= 0) {
			// move triangle point in y
			tri[m_adjust] = moveHit3D(m_cam, x, y, tri[m_adjust], Vec3F(0, 0, 1));		// xy plane			
			Voxelize();
		} else {
			// Adjust camera orbit 
			Vec3F angs = m_cam->getAng();
			angs.x += dx * 0.2f * fine;
			angs.y -= dy * 0.2f * fine;
			m_cam->SetOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
		}		
		appPostRedisplay();	// Update display
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag
	m_adjust = -1;
	if (mouse_down != -1) {
		// check for interaction with corners of triangle
		for (int i = 0; i < 3; i++) {
			if (checkHit3D(m_cam, x, y, tri[i], 5.0)) m_adjust = i;
		}
	}
	mouse_action = 0;

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
	switch (keycode) {
	//case 'c': case 'C':	m_adjust = -1;	break;
	case ' ':			m_voxshow = (m_voxshow + 1) % 4; Voxelize(); break;
	case '1':			m_voxshow = V_EXPLICIT;	Voxelize(); break;
	case '2':			m_voxshow = V_SCHWARZ;	Voxelize();  break;
	case '3':			m_voxshow = V_AKENINE;	Voxelize(); break;
	case '4':			m_voxshow = V_RASTER;	Voxelize(); break;
	case 'a':		 	m_adjust = 0;	break;
	case 'b':		 	m_adjust = 1;	break;
	case 'c':		 	m_adjust = 2;	break;

	};
}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

  if (m_cam==0x0) return;
	m_cam->setSize( w, h );
	m_cam->setAspect(float(w) / float(h));
	m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());	

	appPostRedisplay();
}

void Sample::startup()
{
	int w = 1200, h = 700;
	appStart("Voxelizer", "Voxelizer", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}




