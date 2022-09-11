//--------------------------------------------------------------------------------
// JUST MATH:
// Belief Propagation - on a 3D grid domain
//
// Demonstration of Sum-Product Belief Propagation on a 3D spatial domain.
// Computes:
//     mu_{i,j}[b] = SUM f_{i,j}[a,b] g_i[a] PROD mu_{k,i}[b]
// The message function 'mu' is stored sparsely for neighboring cells in 3D, with size 6*R^3*B,
// where R is the grid resolution, B is the number of discrete values, and 6 is number of neighbors.
//
// To render the result, the belief is estimated at each vertex (voxel), and
// raytraced as a density volume where value probabilities are mapped to color.
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
// Sample utils
#include "main.h"			// window system 
#include "nv_gui.h"			// gui system
#include "image.h"
#include "mersenne.h"
#include <GL/glew.h>
#include <algorithm>

#include "dataptr.h"
#include "common_cuda.h"

#define BUF_VOL			0		// volume: n^3
#define BUF_G			1		// beliefprop, G(a) vector
#define BUF_H			2		// beliefprop, H(a) vector
#define BUF_F			3		// beliefprop, F(a,b) vector - assume independent of i,j
#define BUF_MU			4		// beliefprop, mu{i,j}(a,b) vector 
#define BUF_MU_NXT		5		// beliefprop, mu'{i,j}(a,b) vector

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
	
	// volumes
	void		AllocBuffer(int id, Vector3DI res, int chan=1);
	float		getVoxel ( int id, int x, int y, int z );
	Vector4DF	getVoxel4 ( int id, int x, int y, int z );
	void		WriteFunc (int id, float time);
	void		RaycastCPU ( Camera3D* cam, int id, Image* img );

	// belief prop
	void		Restart();
	void		AllocBPVec (int id, int cnt);									// vector alloc	
	void		AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals);	// matrix alloc
	void		AllocBPMap (int id, int nbrs, int vals);
	uint64_t	getNeighbor(uint64_t j, int nbr);				// 3D spatial neighbor function
	Vector3DI	getVertexPos(uint64_t j);
	uint64_t	getVertex(int x, int y, int z);
	float		getVertexBelief ( uint64_t j, float* bi);
	int			getNumNeighbors(int j)				{return 6;}		 
	int			getNumValues(int j)					{return m_num_values;}
	int			getNumVerts()						{return m_num_verts;}
	
	// belief matrix packing
	float		getVal(int id, int a)				{return *(float*) buf[id].getPtr (a);}						// G and H vectors, size B
	void		SetVal(int id, int a, float val)	{*(float*) buf[id].getPtr(a) = val;}
	float		getVal(int id, int n, int j, int a) {return *(float*) buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }	// mu matrix, NxDxB, where D=R^3, N=nbrs=6
	void		SetVal(int id, int n, int j, int a, float val ) { *(float*) buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ) = val; }
	float		getValF(int id, int a, int b, int n)			{ return *(float*) buf[id].getPtr ( (b*m_num_values + a)*6 + n ); }	// belief mapping (f), BxB
	void		SetValF(int id, int a, int b, int n, float val ) { *(float*) buf[id].getPtr ( (b*m_num_values + a)*6 + n ) = val; }

	void		BeliefProp();
	void		ComputeBelief (int id, int id_vol);
	void		UpdateMU ();

	void		ConstructF ();
	void		ConstructGH ();
	void		ConstructMU ();
	void		NormalizeMU ();
	
	uint64_t	m_num_verts;		// Xi = 0..X (graph domain)
	uint64_t	m_num_values;		//  B = 0..Bm-1 (value domain)	
	Vector3DI	m_bpres;			// 3D spatial belief prop res

	Camera3D*	m_cam;				// camera
	Image*		m_img;				// output image	
	Vector3DI	m_res;				// volume res

	DataPtr		buf[128];			// data buffers (CPU & GPU)	

	int			mouse_down;	
	bool		m_run;
	bool		m_run_cuda;
	bool		m_save;
	float		m_frame;
	int			m_peak_iter;
	int			m_seed;

	Mersenne	m_rand;
};
Sample obj;

//---- volume buffers
void Sample::AllocBuffer (int id, Vector3DI res, int chan)		// volume alloc
{
	uint64_t cnt = res.x*res.y*res.z;
	int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
	buf[id].Resize( chan*sizeof(float), cnt, 0x0, flags );
}
float Sample::getVoxel ( int id, int x, int y, int z )
{
	float* dat = (float*) buf[id].getPtr ( (z*m_res.y + y)*m_res.x + x );
	return *dat;
}
Vector4DF Sample::getVoxel4 ( int id, int x, int y, int z )
{
	Vector4DF* dat = (Vector4DF*) buf[id].getPtr ( (z*m_res.y + y)*m_res.x + x );	
	return *dat;
}

//---- belief prop buffers
void Sample::AllocBPVec (int id, int cnt)						// vector alloc (h and g)
{
	int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
	buf[id].Resize( sizeof(float), cnt, 0x0, flags );
}

// allocate message matrix mu
void Sample::AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals)		// belief matrix alloc (mu)
{
	// NOTE: matrix is stored sparesly. 
	// full matrix: mem=D*D*B, mu{D->D}[B] is full vertex-to-vertex messages over values B, where D=R^3, e.g. D=64^3, B=4, mem=262144^2*4 floats= 1 terabyte
	// sparse mtrx: mem=6*D*B, mu{6->D}[B] since only 6x neigbors are non-zero in 3D. explicitly index those six.
	// final: mem = 6*D*B, e.g. D=64^3, B=4, mem=6*262144*4 floats = 25 megabytes
	uint64_t cnt = nbrs * verts * vals;					
	int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
	buf[id].Resize( sizeof(float), cnt, 0x0, flags );
}
void Sample::AllocBPMap (int id, int nbrs, int vals)				// belief value mapping (f)
{
	uint64_t cnt = vals * vals * nbrs;		// size B*B*6
	int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
	buf[id].Resize( sizeof(float), cnt, 0x0, flags );
}

uint64_t Sample::getVertex(int x, int y, int z)
{
	return uint64_t(z*m_bpres.y + y)*m_bpres.x + x;
}

// domain index to 3D pos
Vector3DI Sample::getVertexPos(uint64_t j)
{
	Vector3DI p;
	p.z = j / (m_bpres.x*m_bpres.y);	j -= p.z * (m_bpres.x*m_bpres.y);
	p.y = j / m_bpres.x;				j -= p.y * m_bpres.x;
	p.x = j;
	return p;
}

// get 3D grid neighbor 
uint64_t Sample::getNeighbor( uint64_t j, int nbr )
{
	Vector3DI jp = getVertexPos(j);

	// 3D spatial neighbor function
	switch (nbr) {
	case 0:		return (jp.x < m_bpres.x-1) ?	j+1 : -1;
	case 1:		return (jp.x > 0) ?				j-1 : -1;
	case 2:		return (jp.y < m_bpres.y-1) ?	j+m_bpres.x : -1;
	case 3:		return (jp.y > 0) ?				j-m_bpres.x : -1;
	case 4:		return (jp.z < m_bpres.z-1) ?	j+(m_bpres.x*m_bpres.y) : -1;
	case 5:		return (jp.z > 0) ?				j-(m_bpres.x*m_bpres.y) : -1;
	};
	return -1;
}

void Sample::ConstructF ()
{
	int B = m_num_values;
	AllocBPMap ( BUF_F, 6, B );

	// randomly enable interactions among values	
	memset( buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );	
	
	
	std::vector<Vector4DF> rules;

	// x = src tile, y=dest tile, z=probability, w=neighbor, w<0=all nbrs		
	
	/*float OK = 1.0;
	rules.push_back( Vector4DF(0,0,0,OK) );		// red<->red, X+
	rules.push_back( Vector4DF(0,0,1,OK) );		// red<->red, X-
	rules.push_back( Vector4DF(0,0,2,0) );		// red<->red, Y+
	rules.push_back( Vector4DF(0,0,3,0) );		// red<->red, Y-

	rules.push_back( Vector4DF(0,0,2,0) );		// red<->red, Y+
	rules.push_back( Vector4DF(0,0,3,0) );		// red<->red, Y- */

	float h=0.5;
	rules.push_back( Vector4DF(0,1,-1,h) );		// red<->grn
	rules.push_back( Vector4DF(1,0,-1,h) );	
	rules.push_back( Vector4DF(1,2,-1,h) );		// grn<->blue
	rules.push_back( Vector4DF(2,1,-1,h) );		
	rules.push_back( Vector4DF(0,3,-1,h) );		// red<->empty
	rules.push_back( Vector4DF(3,0,-1,h) );	
	rules.push_back( Vector4DF(2,3,-1,h) );		// blue<->empty
	rules.push_back( Vector4DF(3,2,-1,h) );	
	rules.push_back( Vector4DF(3,3,-1,1) );		// empty<->empty
	rules.push_back( Vector4DF(2,2,-1,1) ); 	// blue<->blue
	rules.push_back( Vector4DF(1,1,-1,1) );		// green<->green
	rules.push_back( Vector4DF(0,0,-1,1) );		// red<->red	- red can connect to itself, in any direction (w<0) 

	/*rules.push_back( Vector4DF(0,0,1,0) );		// red<->red
	rules.push_back( Vector4DF(0,0,1,1) );
	rules.push_back( Vector4DF(1,1,1,2) );		// grn<->grn
	rules.push_back( Vector4DF(1,1,1,3) );		
	
	rules.push_back( Vector4DF(2,0,1,0) );		// red->blue, X+	
	rules.push_back( Vector4DF(2,0,1,1) );		// red->blue, X-
	rules.push_back( Vector4DF(2,1,1,2) );		// grn->blue, Y+
	rules.push_back( Vector4DF(2,1,1,3) );		// grn->blue, Y-

	/*rules.push_back( Vector4DF(0,3,h,-1) );		// any color to empty		
	rules.push_back( Vector4DF(1,3,h,-1) );		
	rules.push_back( Vector4DF(2,3,h,-1) );*/
	
	Vector4DF r;
	for (int n=0; n < rules.size(); n++) {
		r = rules[n];
		if ( r.z<0) {				// negative w - indicates a rule for all neighbors
			for (int nbr=0; nbr < 6; nbr++)
				SetValF ( BUF_F, r.x, r.y, nbr, r.w);
		} else {
			SetValF ( BUF_F, r.x, r.y, r.z, r.w );
		}
	}
	/*float p = 0.5;
	float q = 0.45;
	SetVal ( BUF_F, 0, 0, 1 );
	SetVal ( BUF_F, 1, 1, 1 );
	SetVal ( BUF_F, 2, 2, 1 );
	SetVal ( BUF_F, 3, 3, 1 );

	SetVal ( BUF_F, 0, 1, q );		// red-grn
	SetVal ( BUF_F, 1, 0, q );

	SetVal ( BUF_F, 0, 2, 0 );		// red-blue
	SetVal ( BUF_F, 2, 0, 0 );

	SetVal ( BUF_F, 0, 3, p );		// red-empty
	SetVal ( BUF_F, 3, 0, p );

	SetVal ( BUF_F, 1, 2, 0 );		// green-blue
	SetVal ( BUF_F, 2, 1, 0 );

	SetVal ( BUF_F, 1, 3, 0 );		// green-empty
	SetVal ( BUF_F, 3, 1, 0 );

	SetVal ( BUF_F, 2, 3, p );		// blue-empty
	SetVal ( BUF_F, 3, 2, p );	 */
}

void Sample::ConstructGH ()
{
	AllocBPVec ( BUF_G, m_num_values );
	AllocBPVec ( BUF_H, m_num_values );
	
	float weight = 1.0 / m_num_values;
	for (int a=0; a < m_num_values; a++ ) {
		SetVal ( BUF_G, a, weight );
	}
}



void Sample::ConstructMU ()
{
	AllocBPMtx ( BUF_MU, 6, m_num_verts, m_num_values );
	AllocBPMtx ( BUF_MU_NXT, 6, m_num_verts, m_num_values );

	float w;
	float* mu = (float*) buf[BUF_MU].getData();
	uint64_t cnt = 6 * m_num_verts * m_num_values;
	memset ( mu, 0, cnt * sizeof(float) );

	int i;
	for (int j=0; j < m_num_verts; j++) 
		for (int jnbr=0; jnbr < getNumNeighbors(j); jnbr++) {
			i = getNeighbor(j, jnbr);
			 for (int a=0; a < m_num_values;a++) {
				w = m_rand.randF();
				SetVal( BUF_MU, jnbr, j, a, w );			// randomize MU
			}
		}
}

void Sample::NormalizeMU ()
{
	int i;
	float v, sum;
	
	for (int j=0; j < m_num_verts; j++) {
		for (int in=0; in < getNumNeighbors(j); in++) {
			i = getNeighbor(j, in);
			sum = 0;
			for (int a=0; a < m_num_values; a++) {
				sum += getVal(BUF_MU, in, j, a );
			}		
			if ( sum > 0 ) {
				for (int a=0; a < m_num_values; a++) {			
					v = getVal(BUF_MU, in, j, a);
					SetVal( BUF_MU, in, j, a, v / sum);
				}
			}
		}		
	}
}

void Sample::BeliefProp ()
{	
	uint64_t i, j, k;
	float H_ij_a;
	float u_nxt_b, u_prev_b;
	float mu_j, du;
	int jchk = getVertex(3,3,3);

	float rate = .98;

	// for all i->j messages in graph domain
	for ( uint64_t j=0; j < getNumVerts(); j++ ) {				// <-- this is a 3D grid

		for (int in=0; in < getNumNeighbors(j); in++) {			// <-- 6 neighbors of j in 3D
			i = getNeighbor(j, in);
			
			if ( i != -1 ) {
				// compute message from i to j			
				// for each a..
				for (int a=0; a < getNumValues(j); a++) {

					// first compute Hij_t
					H_ij_a = getVal(BUF_G, a);							// initialize Hij(a) = gi(a)
					for (int kn=0; kn < getNumNeighbors(i); kn++ ) {
						k = getNeighbor(i, kn);
						if (k!=-1 && k!=j) H_ij_a *= getVal(BUF_MU, kn, i, a);	// Hij(a) = gi(a) * PROD mu{ki}_a
					}
					//mu_j = getVal(BUF_MU, j, i, a);					// exclude mu{ji}_a - message from j to i
					//if (mu_j > 0) H_ij_a /= mu_j;
					SetVal (BUF_H, a, H_ij_a);
				}

				// now compute mu_ij_t+1 = Fij * hij 				
				//max_mu = 0;
				for (int b=0; b < getNumValues(j); b++) {			// b = rows in f{ij}(a,b), also elements of mu(b)
					u_nxt_b = 0;					
					for (int a=0; a < getNumValues(j); a++) {		// a = cols in f{ij}(a,b), also elements of h(a)
						u_nxt_b += getValF(BUF_F, a, b, in) * getVal(BUF_H, a);
					}
					u_prev_b = getVal(BUF_MU, in, j, b);			// in = neighbor index of i, for mu{i,j}(b)
					du = u_nxt_b - u_prev_b;
					//if ( j < 8 ) dbgprintf ( "%d: %f\n", j, du );
					SetVal (BUF_MU_NXT, in, j, b, u_prev_b + du*rate );
				}	
				
			}
		}
	}
}

void Sample::UpdateMU ()
{	
	float* mu_curr = (float*) buf[BUF_MU].getData();
	float* mu_next = (float*) buf[BUF_MU_NXT].getData();

	uint64_t cnt = 6 * m_num_verts * m_num_values;
	memcpy ( mu_curr, mu_next, cnt * sizeof(float) );
}

float Sample::getVertexBelief ( uint64_t j, float* bi)
{
	uint64_t k;
	float sum = 0;
	for (int a=0; a < m_num_values; a++) {
		bi[a] = 1.0;  //getVal(BUF_G, a);
		for (int kn=0; kn < getNumNeighbors(j); kn++) {
			k = getNeighbor(j, kn);
			if (k!=-1) bi[a] *= getVal(BUF_MU, kn, j, a );		// mu{k,j}(a)
		}
		sum += bi[a];
	}
	if ( sum > 0 )
		for (int a=0; a < m_num_values; a++) 
			bi[a] /= sum;
	
	return sum;
}

void Sample::ComputeBelief (int id, int id_vol)
{
	int k;
	float sum;
	float bi[16];

	Vector4DF* dat = (Vector4DF*) buf[id_vol].getData();
	Vector4DF* vox = dat;
	
	uint64_t j, i;

	for ( j=0; j < getNumVerts(); j++ ) {		
		getVertexBelief (j, bi);		
 		vox->x = bi[0] + bi[3];
		vox->y = bi[1] + bi[3];
		vox->z = bi[2];
		//vox->w = max(bi[0], max(bi[1], max(bi[2], bi[3])));
		vox->w = max(bi[0], max(bi[1], bi[2]));
		vox++;
	}

	//----------- alternative: select highest b and write only that to volume
	// find highest probability
	/* int j_best = 0;
	int a_best = 0;
	float bi_best = 0;	
	vox = dat;
	for ( j=0; j < getNumVerts(); j++ ) {		
		// compute belief at j			
		if ( vox->w == 0 ) {								// select only from those not yet decided
			getVertexBelief ( j, bi );			
			for (int a=0; a < m_num_values; a++) {
				if ( bi[a] > bi_best ) {
					j_best = j;
					a_best = a;
					bi_best = bi[a];				
				}
			}	
		}
		vox++;
	}	
	// recompute bi[a] at best j
	getVertexBelief (j_best, bi );
	
	// write belief directly to volume for visualization
	// * we are assuming the spatial layout x/y/z of vol matches the indexing of j						
	Vector3DF b ( bi[0], bi[1], bi[2] );
	vox = dat + j_best;						// voxel for highest probability
 	vox->x = b.x;
	vox->y = b.y;
	vox->z = b.z;
	vox->w = max(b.x, max(b.y, b.z));	*/

	
}

void Sample::WriteFunc (int id, float time)
{
	Vector3DF c, d;
	float v;
	Vector4DF* dat = (Vector4DF*) buf[id].getData();
	Vector4DF* vox = dat;
	
	c = m_res / 2;

	float maxv = sqrt(c.x*c.x + c.y*c.y + c.z*c.z );
	Vector3DF s;

	for (int z=0; z < m_res.z; z++)
		for (int y=0; y < m_res.y; y++)
			for (int x=0; x < m_res.x; x++) {
				d.Set ( (x-c.x), (y-c.y), (z-c.z) );
				
				//v = max(0.6 - (sqrt(d.x*d.x + d.y*d.y + d.z*d.z) / maxv), 0);		// sphere
				
				v = sin(d.x*d.y*d.z*0.0001 + time*0.1)*0.5+0.5;						// sin(x*y*z + t)
				v = v*v*v;
				
				/*s.x = sin (d.x*0.1 + time*0.1)*0.25+0.5;							// sin(x)+cos(z) < y
				s.z = cos (d.z*0.2 + time*0.1)*0.25+0.5;
				v = (y < (s.x+s.z)*c.y ) ? 1.0 : 0;	 */

				vox->x = float(x)/m_res.x;
				vox->y = float(y)/m_res.y;
				vox->z = float(z)/m_res.z;
				vox->w = v; 
				vox++;
			}

	if ( m_run_cuda )
		buf[id].Commit ();			// commit to GPU
}

void Sample::RaycastCPU ( Camera3D* cam, int id, Image* img )
{
	Vector3DF rpos, rdir;
	Vector4DF clr;

	Vector3DF vmin (0,0,0);
	Vector3DF vmax (1,1,1);
	Vector3DF wp, dwp, p, dp, t;
	Vector3DF vdel = m_res;
	Vector4DF val;
	int iter;
	float alpha, k;
	float pStep = 0.005;				// volume quality   - lower=better (0.01), higher=worse (0.1)
	float kDensity = 4.0;				// volume density   - lower=softer, higher=more opaque
	float kIntensity = 8.0;				// volume intensity - lower=darker, higher=brighter
	float kWidth = 4.0;					// transfer func    - lower=broader, higher=narrower (when sigmoid transfer enabled)

	int xres = img->GetWidth();
	int yres = img->GetHeight();
	
	// for each pixel in image..
	m_peak_iter = 0;
	for (int y=0; y < yres; y++) {
		for (int x=0; x < xres; x++) {
			
			// get camera ray
			rpos = cam->getPos();
			rdir = cam->inverseRay ( x, y, xres, yres );	
			rdir.Normalize();

			// intersect with volume box
			t = intersectLineBox ( rpos, rdir, vmin, vmax );
			clr.Set(0,0,0,0);
			if ( t.z >= 0 ) {
				// hit volume, start raycast...		
				wp = rpos + rdir * t.x + Vector3DF(0.001, 0.001, 0.001);		// starting point in world space				
				dwp = (vmax-vmin) * rdir * pStep;								// ray sample stepping in world space
				p = Vector3DF(m_res-1) * (wp - vmin) / (vmax-vmin);				// starting point in volume				
				dp = Vector3DF(m_res-1) * rdir * pStep;							// step delta along ray
				
				// accumulate along ray
				for (iter=0; iter < 512 && clr.w < 0.95 && p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < m_res.x && p.y < m_res.y && p.z < m_res.z; iter++) {
					val = getVoxel4 ( BUF_VOL, p.x, p.y, p.z );					// get voxel value
					alpha = val.w;												// opacity = linear transfer
					//alpha = 1.0 / (1+exp(-(val.w-1.0)*kWidth));				// opacity = sigmoid transfer - accentuates boundaries at 0.5
					clr += Vector4DF(val.x,val.y,val.z, 0) * (1-clr.w) * alpha * kIntensity * pStep;	// accumulate color						
					clr.w += alpha * kDensity * pStep;							// attenuate alpha					
					p += dp; 													// next sample
				}	
				if (iter > m_peak_iter) m_peak_iter = iter;
				if (clr.x > 1.0) clr.x = 1;
				if (clr.y > 1.0) clr.y = 1;
				if (clr.z > 1.0) clr.z = 1;
				clr *= 255.0;
			}	
			// set pixel
			img->SetPixel ( x, y, clr.x, clr.y, clr.z );
		}
	}

	// commit image to OpenGL (hardware gl texture) for on-screen display
	img->Commit ( DT_GLTEX );			
	
	// optional write to disk
	if ( m_save ) {
		char savename[256];
		sprintf ( savename, "out%04d.png", (int) m_frame );
		img->Save ( savename );				
	}
}

void Sample::Restart()
{
	m_rand.seed ( m_seed++ );	

	ConstructF ();
	ConstructGH ();
	ConstructMU ();
	NormalizeMU ();
}

bool Sample::init()
{
	addSearchPath(ASSET_PATH);
	init2D("arial_256");
	setText(24,1);

	// options
	m_frame = 0;
	m_run = false;
	m_run_cuda = false;									// run cuda pathway
	m_save = false;										// save image sequence to disk
	
	int R = 32;

	// belief propagation setup
	m_bpres.Set ( R, R, R );		// D = R^3
	m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
	m_num_values = 4;
	
	m_res.Set ( R, R, R );							// volume resolution

	Restart();

	m_cam = new Camera3D;								// create camera
	m_cam->setOrbit ( 30, 20, 0, Vector3DF(.5,.5,.5), 10, 1 );

	m_img = new Image;									// create image
	m_img->ResizeImage ( 256, 256, ImageOp::RGB24 );	// image resolution (output)

	#ifdef USE_CUDA		
		if ( m_run_cuda ) {
			CUcontext ctx; 
			CUdevice dev;
			cuStart ( DEV_FIRST, 0, dev, ctx, 0, true );		// start CUDA
		}
	#endif

	AllocBuffer ( BUF_VOL, m_res, 4 );					// allocate color volume (4 channel)

	return true;
}

void Sample::display()
{
	char msg[256];
	Vector3DF a, b, c;
	Vector3DF p, q, d;

	clearGL();
	
	// advance
	if (m_run) {									// time update
		//m_run = false;		// single stepping
		
		ComputeBelief ( BUF_MU, BUF_VOL );
		BeliefProp ();		
		UpdateMU();
		NormalizeMU();	

		//WriteFunc ( BUF_VOL, m_frame );				// write to volume
		//m_frame++;
	}

	// raycast
	RaycastCPU ( m_cam, BUF_VOL, m_img );			// raycast volume

	// draw 2D
	start2D();
	setview2D(getWidth(), getHeight());	

	drawImg ( m_img->getGLID(), 0, 0, getWidth(), getHeight(), 1,1,1,1 );	// draw raycast image 	

	//sprintf ( msg, "Peak iter: %d\n", m_peak_iter );			// overlay text
	//drawText ( 10, 10, msg, 1,1,1,1);

	end2D();
	draw2D();										// complete 2D rendering to OpenGL

	// draw grid in 3D
	start3D(m_cam);
	setLight(S3D, 20, 100, 20);	
	for (int i=-10; i <= 10; i++ ) {
		drawLine3D( i, 0, -10, i, 0, 10, 1,1,1, .1);
		drawLine3D( -10, 0, i, 10, 0, i, 1,1,1, .1);
	}
	drawBox3D ( Vector3DF(0,0,0), Vector3DF(1,1,1), 1,1,1, 0.3);
	end3D();

	draw3D();										// complete 3D rendering to OpenGL
	
	appPostRedisplay();								// Post redisplay since simulation is continuous
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
	float fine = 0.5;

	switch (mouse_down) {
	case AppEnum::BUTTON_LEFT: {

		appPostRedisplay();	// Update display
	} break;

	case AppEnum::BUTTON_MIDDLE: {
		// Adjust target pos		
		m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
		appPostRedisplay();	// Update display
	} break;

	case AppEnum::BUTTON_RIGHT: {

		// Adjust camera orbit 
		Vector3DF angs = m_cam->getAng();
		angs.x += dx * 0.2f * fine;
		angs.y -= dy * 0.2f * fine;
		m_cam->setOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());		
		appPostRedisplay();	// Update display
	} break;
	}
}

void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
	if (guiHandler(button, state, x, y)) return;
	mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag
}

void Sample::mousewheel(int delta)
{
	// Adjust zoom
	float zoomamt = 1.0;
	float dist = m_cam->getOrbitDist();
	float dolly = m_cam->getDolly();
	float zoom = (dist - dolly) * 0.001f;
	dist -= delta * zoom * zoomamt;

	m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), dist, dolly);
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
	if (action==AppEnum::BUTTON_RELEASE) return;

	switch (keycode) {
	case ' ':	m_run = !m_run;	break;
	case 'g':	Restart();	break;
	};
}

void Sample::reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	setview2D(w, h);

	m_cam->setSize( w, h );
	m_cam->setAspect(float(w) / float(h));
	m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
	m_cam->updateMatricies();

	appPostRedisplay();
}

void Sample::startup()
{
	int w = 1900, h = 1000;
	appStart("Volume Raycast", "Volume Raycast", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}





