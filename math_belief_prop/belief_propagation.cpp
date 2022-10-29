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
#include <algorithm>
#include "mersenne.h"
#include "dataptr.h"

/*
#ifdef USE_OPENGL
  #include <GL/glew.h>
#endif
#ifdef USE_CUDA
  #include "common_cuda.h"
#endif
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <vector>
#include <string>

#define BUF_VOL         0    // volume: n^3
#define BUF_G           1    // beliefprop, G(a) vector
#define BUF_H           2    // beliefprop, H(a) vector
#define BUF_F           3    // beliefprop, F(a,b) vector - assume independent of i,j
#define BUF_MU          4    // beliefprop, mu{i,j}(a,b) vector 
#define BUF_MU_NXT      5    // beliefprop, mu'{i,j}(a,b) vector
#define BUF_BELIEF      6    // Belief array
#define BUF_TILE_IDX    7
#define BUF_TILE_IDX_N  8

class BeliefPropagation {
public:
  bool _init();
  bool init(int);
  bool init(int, int, int);
  bool init_F_CSV(std::string &, std::string &);
  float step();

  // volumes
  void    AllocBuffer(int id, Vector3DI res, int chan=1);

  // belief prop
  void    Restart();
  void    AllocBPVec (int id, int cnt);                  // vector alloc  
  void    AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals);  // matrix alloc
  void    AllocBPMap (int id, int nbrs, int vals);

  void    AllocTileIdx (int, int, int);
  void    AllocTileIdxN(int, int );

  uint64_t  getNeighbor(uint64_t j, int nbr);        // 3D spatial neighbor function
  Vector3DI  getVertexPos(uint64_t j);
  uint64_t  getVertex(int x, int y, int z);
  float    getVertexBelief ( uint64_t j );
  float    _getVertexBelief ( uint64_t j, float* bi);
  int      getNumNeighbors(int j)        {return 6;}     
  int      getNumValues(int j)          {return m_num_values;}
  int      getNumVerts()            {return m_num_verts;}
  
  // belief matrix packing
  float   getVal(int id, int a)        {return *(float*) buf[id].getPtr (a);}            // G and H vectors, size B
  void    SetVal(int id, int a, float val)  {*(float*) buf[id].getPtr(a) = val;}
  float   getVal(int id, int n, int j, int a) {return *(float*) buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }  // mu matrix, NxDxB, where D=R^3, N=nbrs=6
  void    SetVal(int id, int n, int j, int a, float val ) { *(float*) buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ) = val; }
  float   getValF(int id, int a, int b, int n)      { return *(float*) buf[id].getPtr ( (b*m_num_values + a)*6 + n ); }  // belief mapping (f), BxB
  void    SetValF(int id, int a, int b, int n, float val ) { *(float*) buf[id].getPtr ( (b*m_num_values + a)*6 + n ) = val; }

  void    SetVali(int id, int i, int32_t val)   { *(int32_t *) buf[id].getPtr (i) = val; }
  int32_t getVali(int id, int i)                { return *(int32_t *) buf[id].getPtr (i); }

  int32_t getVali(int id, int i, int a)                { return *(int32_t *) buf[id].getPtr ( uint64_t(i*m_num_values + a) ); }
  void    SetVali(int id, int i, int a, int32_t val)   { *(int32_t*) buf[id].getPtr ( (i*m_num_values + a) ) = val; }

  float   BeliefProp();
  void    ComputeBelief (int id, int id_vol);
  void    UpdateMU ();

  void    ConstructF ();
  void    ConstructGH ();
  void    ConstructMU ();
  void    NormalizeMU ();

  void    _NormalizeMU ();

  void    ConstructTileIdx();
  
  uint64_t  m_num_verts;    // Xi = 0..X (graph domain)
  uint64_t  m_num_values;    //  B = 0..Bm-1 (value domain)  
  Vector3DI  m_bpres;      // 3D spatial belief prop res

  Vector3DI  m_res;        // volume res

  DataPtr    buf[128];      // data buffers (CPU & GPU)  

  int      mouse_down;  
  bool    m_run_cuda;
  float    m_frame;
  int      m_peak_iter;
  int      m_seed;

  void debugPrint();

  Mersenne  m_rand;

  std::vector< std::string > m_tile_name;

  void filterKeep(uint64_t pos, std::vector<int32_t> &tile_id);
  void filterDiscard(uint64_t pos, std::vector<int32_t> &tile_id);
  int32_t tileName2ID (std::string &tile_name);
  int32_t tileName2ID (char *);

};

//Sample obj;

//---- volume buffers
//
void BeliefPropagation::AllocBuffer (int id, Vector3DI res, int chan)    // volume alloc
{
  uint64_t cnt = res.x*res.y*res.z;
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  buf[id].Resize( chan*sizeof(float), cnt, 0x0, flags );
}

//---- belief prop buffers
//
void BeliefPropagation::AllocBPVec (int id, int cnt)            // vector alloc (h and g)
{
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  buf[id].Resize( sizeof(float), cnt, 0x0, flags );
}

// allocate message matrix mu
void BeliefPropagation::AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals)    // belief matrix alloc (mu)
{
  // NOTE: matrix is stored sparesly. 
  // full matrix: mem=D*D*B, mu{D->D}[B] is full vertex-to-vertex messages over values B, where D=R^3, e.g. D=64^3, B=4, mem=262144^2*4 floats= 1 terabyte
  // sparse mtrx: mem=6*D*B, mu{6->D}[B] since only 6x neigbors are non-zero in 3D. explicitly index those six.
  // final: mem = 6*D*B, e.g. D=64^3, B=4, mem=6*262144*4 floats = 25 megabytes
  uint64_t cnt = nbrs * verts * vals;          
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  buf[id].Resize( sizeof(float), cnt, 0x0, flags );
}

void BeliefPropagation::AllocBPMap (int id, int nbrs, int vals)        // belief value mapping (f)
{
  uint64_t cnt = vals * vals * nbrs;    // size B*B*6
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  buf[id].Resize( sizeof(float), cnt, 0x0, flags );
}

void BeliefPropagation::AllocTileIdx(int id, int nvert, int nval)        // belief value mapping (f)
{
  uint64_t cnt = nvert * nval ;
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  buf[id].Resize( sizeof(int32_t), cnt, 0x0, flags );
}

void BeliefPropagation::AllocTileIdxN(int id, int nval)        // belief value mapping (f)
{
  uint64_t cnt = nval ;
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  buf[id].Resize( sizeof(int32_t), cnt, 0x0, flags );
}

uint64_t BeliefPropagation::getVertex(int x, int y, int z)
{
  return uint64_t(z*m_bpres.y + y)*m_bpres.x + x;
}

// domain index to 3D pos
Vector3DI BeliefPropagation::getVertexPos(uint64_t j)
{
  Vector3DI p;
  p.z = j / (m_bpres.x*m_bpres.y);  j -= p.z * (m_bpres.x*m_bpres.y);
  p.y = j / m_bpres.x;        j -= p.y * m_bpres.x;
  p.x = j;
  return p;
}

// get 3D grid neighbor 
uint64_t BeliefPropagation::getNeighbor( uint64_t j, int nbr )
{
  Vector3DI jp = getVertexPos(j);

  // 3D spatial neighbor function
  switch (nbr) {
  case 0:    return (jp.x < m_bpres.x-1) ?  j+1 : -1;
  case 1:    return (jp.x > 0) ?        j-1 : -1;
  case 2:    return (jp.y < m_bpres.y-1) ?  j+m_bpres.x : -1;
  case 3:    return (jp.y > 0) ?        j-m_bpres.x : -1;
  case 4:    return (jp.z < m_bpres.z-1) ?  j+(m_bpres.x*m_bpres.y) : -1;
  case 5:    return (jp.z > 0) ?        j-(m_bpres.x*m_bpres.y) : -1;
  };
  return -1;
}

void BeliefPropagation::ConstructTileIdx() {
  int i, j;
  AllocTileIdx( BUF_TILE_IDX, m_num_verts, m_num_values );
  AllocTileIdxN( BUF_TILE_IDX_N, m_num_verts );
  for (i=0; i<m_num_verts; i++) {
    SetVali( BUF_TILE_IDX_N, i, m_num_values );
    for (j=0; j<m_num_values; j++) {
      SetVali( BUF_TILE_IDX, i, j, (int32_t)j );
    }
  }
}

void BeliefPropagation::ConstructF ()
{
  int B = m_num_values;

  AllocBPMap ( BUF_F, 6, B );

  // randomly enable interactions among values  
  memset( buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  
  
  
  std::vector<Vector4DF> rules;

  float h=0.5;
  rules.push_back( Vector4DF(0,1,-1,h) );    // red<->grn
  rules.push_back( Vector4DF(1,0,-1,h) );  
  rules.push_back( Vector4DF(1,2,-1,h) );    // grn<->blue
  rules.push_back( Vector4DF(2,1,-1,h) );    
  rules.push_back( Vector4DF(0,3,-1,h) );    // red<->empty
  rules.push_back( Vector4DF(3,0,-1,h) );  
  rules.push_back( Vector4DF(2,3,-1,h) );    // blue<->empty
  rules.push_back( Vector4DF(3,2,-1,h) );  
  rules.push_back( Vector4DF(3,3,-1,1) );    // empty<->empty
  rules.push_back( Vector4DF(2,2,-1,1) );   // blue<->blue
  rules.push_back( Vector4DF(1,1,-1,1) );    // green<->green
  rules.push_back( Vector4DF(0,0,-1,1) );    // red<->red  - red can connect to itself, in any direction (w<0) 

 
  Vector4DF r;
  for (int n=0; n < rules.size(); n++) {
    r = rules[n];
    if ( r.z<0) {        // negative w - indicates a rule for all neighbors
      for (int nbr=0; nbr < 6; nbr++)
        SetValF ( BUF_F, r.x, r.y, nbr, r.w);
    } else {
      SetValF ( BUF_F, r.x, r.y, r.z, r.w );
    }
  }

}

void BeliefPropagation::ConstructGH ()
{
  AllocBPVec ( BUF_G, m_num_values );
  AllocBPVec ( BUF_H, m_num_values );
  
  float weight = 1.0 / m_num_values;
  for (int a=0; a < m_num_values; a++ ) {
    SetVal ( BUF_G, a, weight );
  }
}



void BeliefPropagation::ConstructMU ()
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

        //DEBUG
        //printf(" constructmu [%i][%i][%i] = %f\n", (int)jnbr, (int)j, (int)a, (float)w);

        SetVal( BUF_MU, jnbr, j, a, w );      // randomize MU
      }
    }
}

void BeliefPropagation::NormalizeMU ()
{
  int i, n_a, a;
  float v, sum;

  for (int j=0; j < m_num_verts; j++) {
    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, in);
      sum = 0;

      n_a = getVali( BUF_TILE_IDX_N, j );
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getVali( BUF_TILE_IDX, j, a_idx );
        sum += getVal(BUF_MU, in, j, a );
      }

      if ( sum > 0 ) {
        n_a = getVali( BUF_TILE_IDX_N, j );
        for (int a_idx=0; a_idx<n_a; a_idx++) {
          a = getVali( BUF_TILE_IDX, j, a_idx );

          v = getVal(BUF_MU, in, j, a);
          SetVal( BUF_MU, in, j, a, v / sum);
        }
      }

    }
  }
}

void BeliefPropagation::_NormalizeMU ()
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

float BeliefPropagation::BeliefProp ()
{  
  uint64_t i, j, k;
  float H_ij_a;
  float u_nxt_b, u_prev_b;
  float mu_j, du;
  int jchk = getVertex(3,3,3);

  float rate = .98;

  float max_diff = -1.0;

  uint64_t count=0;

  // for all i->j messages in graph domain
  //
  for ( uint64_t j=0; j < getNumVerts(); j++ ) {

    // 6 neighbors of j in 3D
    //
    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, in);
      if (i == -1) { continue; }

      // compute message from i to j for each a..
      //
      //for (int a=0; a < getNumValues(j); a++) {
      for (a_idx=0; a_idx<a_n; a_idx++) {

        // CHECKPOINT (WIP)!!!
        //!!!
        //a = getVali( BUF_TILE_IDX, 

        // first compute Hij_t
        // initialize Hij(a) = gi(a)/
        //
        H_ij_a = getVal(BUF_G, a);

        for (int kn=0; kn < getNumNeighbors(i); kn++ ) {
          k = getNeighbor(i, kn);

          // Hij(a) = gi(a) * PROD mu{ki}_a
          //
          if (k!=-1 && k!=j) {
            H_ij_a *= getVal(BUF_MU, kn, i, a);
          }

        }

        // exclude mu{ji}_a - message from j to i
        //
        SetVal (BUF_H, a, H_ij_a);
      }

      // now compute mu_ij_t+1 = Fij * hij         
      // b = rows in f{ij}(a,b), also elements of mu(b)/
      //
      for (int b=0; b < getNumValues(j); b++) {
        u_nxt_b = 0;          

        // a = cols in f{ij}(a,b), also elements of h(a)
        //
        for (int a=0; a < getNumValues(j); a++) {
          u_nxt_b += getValF(BUF_F, a, b, in) * getVal(BUF_H, a);
          count++;
        }
        u_prev_b = getVal(BUF_MU, in, j, b);

        du = u_nxt_b - u_prev_b;

        if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }

        SetVal (BUF_MU_NXT, in, j, b, u_prev_b + du );
      }
    }
  }

  printf(">>> %i\n", (int)count);

  return max_diff;
}

// copy BUF_MU_NXT back to BUF_MU
//
void BeliefPropagation::UpdateMU ()
{  
  float* mu_curr = (float*) buf[BUF_MU].getData();
  float* mu_next = (float*) buf[BUF_MU_NXT].getData();

  uint64_t cnt = 6 * m_num_verts * m_num_values;
  memcpy ( mu_curr, mu_next, cnt * sizeof(float) );
}

float BeliefPropagation::_getVertexBelief ( uint64_t j, float* bi )
{
  uint64_t k;
  float sum = 0;
  for (int a=0; a < m_num_values; a++) {
    bi[a] = 1.0;
    for (int kn=0; kn < getNumNeighbors(j); kn++) {
      k = getNeighbor(j, kn);

      // mu{k,j}(a)
      //
      if (k!=-1) { bi[a] *= getVal(BUF_MU, kn, j, a ); }
    }
    sum += bi[a];
  }
  if ( sum > 0 ) {
    for (int a=0; a < m_num_values; a++)  {
      bi[a] /= sum;
    }
  }
  
  return sum;
}

float BeliefPropagation::getVertexBelief ( uint64_t j )
{
  uint64_t k;
  float sum = 0;
  float _bi = 1.0;

  for (int a=0; a < m_num_values; a++) {
    SetVal( BUF_BELIEF, a, 1.0 );

    for (int kn=0; kn < getNumNeighbors(j); kn++) {
      k = getNeighbor(j, kn);
      if (k!=-1) {

        // mu{k,j}(a)
        //
        _bi = getVal( BUF_BELIEF, a) * getVal(BUF_MU, kn, j, a );
        SetVal( BUF_BELIEF, a, _bi );
      }
    }
    sum += getVal( BUF_BELIEF, a );
  }
  if ( sum > 0 ) {
    for (int a=0; a < m_num_values; a++) {
      _bi = getVal( BUF_BELIEF, a ) / sum;
      SetVal( BUF_BELIEF, a, _bi );
    }
  }
  
  return sum;
}

void BeliefPropagation::ComputeBelief (int id, int id_vol)
{
  int k;
  float sum;
  //float bi[16];
  float *bi;
  uint64_t j, i;

  //Vector4DF* dat = (Vector4DF*) buf[id_vol].getData();
  //Vector4DF* vox = dat;

  for ( j=0; j < getNumVerts(); j++ ) {    
    //getVertexBelief (j, bi);    
    getVertexBelief (j);

    /*
    vox->x = bi[0] + bi[3];
    vox->y = bi[1] + bi[3];
    vox->z = bi[2];
    //vox->w = max(bi[0], max(bi[1], max(bi[2], bi[3])));
    vox->w = std::max(bi[0], std::max(bi[1], bi[2]));
    vox++;
    */
  }

  //----------- alternative: select highest b and write only that to volume
  // find highest probability
  /* int j_best = 0;
  int a_best = 0;
  float bi_best = 0;  
  vox = dat;
  for ( j=0; j < getNumVerts(); j++ ) {    
    // compute belief at j      
    if ( vox->w == 0 ) {                // select only from those not yet decided
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
  vox = dat + j_best;            // voxel for highest probability
   vox->x = b.x;
  vox->y = b.y;
  vox->z = b.z;
  vox->w = max(b.x, max(b.y, b.z));
  */

  printf("----\n"); fflush(stdout);
  
}

void BeliefPropagation::Restart()
{
  m_rand.seed ( m_seed++ );  

  ConstructF ();
  ConstructGH ();
  ConstructMU ();
  NormalizeMU ();

  ConstructTileIdx();
}

//----

int _read_line(FILE *fp, std::string &line) {
  int ch=0, count=0;

  while (!feof(fp)) {
    ch = fgetc(fp);
    if (ch == '\n') { break; }
    if (ch == EOF) { break; }
    line += (char)ch;
    count++;
  }
  return count;
}

int _read_name_csv(std::string &fn, std::vector<std::string> &name) {
  int i, idx;
  FILE *fp;
  std::string line, tok, _s;
  std::vector<std::string> toks;

  fp = fopen(fn.c_str(), "r");
  if (!fp) { return -1; }

  while (!feof(fp)) {
    line.clear();
    _read_line(fp, line);

    if (line.size()==0) { continue; }
    if (line[0] == '#') { continue; }
    if (line[0] == ' ') { continue; }

    toks.clear();
    tok.clear();
    for (i=0; i<line.size(); i++) {
      if (line[i]==',') {
        toks.push_back(tok);
        tok.clear();
        continue;
      }
      tok += line[i];
    }
    toks.push_back(tok);

    if (toks.size() != 2) { continue; }

    idx = atoi(toks[0].c_str());
    if (idx <= name.size()) {
      for (i=name.size(); i<=idx; i++) {
        _s.clear();
        name.push_back(_s);
      }
    }
    name[idx] = toks[1];
  }

  fclose(fp);

  return 0;
}


int _read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule) {
  int i;
  float val, _weight;
  FILE *fp;
  std::string line, tok;
  std::vector<std::string> toks;
  std::vector<float> v;

  float _tile_src, _tile_dst;

  fp = fopen(fn.c_str(), "r");
  if (!fp) { return -1; }

  while (!feof(fp)) {
    line.clear();
    _read_line(fp, line);

    if (line.size()==0) { continue; }
    if (line[0] == '#') { continue; }
    if (line[0] == ' ') { continue; }

    toks.clear();
    tok.clear();
    for (i=0; i<line.size(); i++) {
      if (line[i]==',') {
        toks.push_back(tok);
        tok.clear();
        continue;
      }
      tok += line[i];
    }
    toks.push_back(tok);

    if ((toks.size() < 3) || (toks.size() > 4)) { continue; }

    _tile_src = atof(toks[0].c_str());
    _tile_dst = atof(toks[1].c_str());
    _weight = 1;

    if ((toks.size() >= 4) &&
        (toks[3].size() != 0) &&
        (toks[3][0] != 'u')) {
      _weight = atof(toks[3].c_str());
    }

    // direction wild card
    //
    if ((toks[2].size()==0) ||
        (toks[2][0] == '*')) {
      v.clear();
      v.push_back(0.0);
      v.push_back(0.0);
      v.push_back(0.0);
      v.push_back(0.0);
      for (i=0; i<6; i++) {
        v[0] = _tile_src;
        v[1] = _tile_dst;
        v[2] = (float)i;
        v[3] = _weight ;
        rule.push_back(v);
      }
    }

    // explicit entry
    //
    else {
      v.clear();
      v.push_back(_tile_src);
      v.push_back(_tile_dst);
      v.push_back(atof(toks[2].c_str()));
      v.push_back(_weight);
      rule.push_back(v);
    }

  }

  fclose(fp);

  return 0;
}



//----

bool BeliefPropagation::init_F_CSV(std::string &rule_fn, std::string &name_fn)
{
  int i;
  //std::string name_fn = "examples/stair_name.csv";
  //std::string rule_fn = "examples/stair_rule.csv";
  //std::vector< std::string > tile_name;
  std::vector< std::vector<float> > tile_rule;

  _read_name_csv(name_fn, m_tile_name);
  _read_rule_csv(rule_fn, tile_rule);

  //---
  //m_rand.seed ( m_seed++ );  

  int R = 32;
  //int R = 10;
  m_bpres.Set ( R, R, R );
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = m_tile_name.size();

  // F
  //
  int B = m_num_values;
  AllocBPMap ( BUF_F, 6, B );
  memset( buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  
  for (i=0; i<tile_rule.size(); i++) {
    SetValF( BUF_F, tile_rule[i][0], tile_rule[i][1], tile_rule[i][2], tile_rule[i][3] );
  }

  ConstructGH();


  return true;
}

bool BeliefPropagation::init(int R)
{
  int i, j;
  std::string name_fn = "examples/stair_name.csv";
  std::string rule_fn = "examples/stair_rule.csv";
  //std::vector< std::string > tile_name;
  //std::vector< std::vector<float> > tile_rule;

  init_F_CSV(rule_fn, name_fn);

  ConstructTileIdx();

  //---
  m_rand.seed ( m_seed++ );  

  //int R = 32;
  //int R = 10;
  m_bpres.Set ( R, R, R );
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = m_tile_name.size();
  m_res.Set ( R, R, R );

  // F
  //
  /*
  int B = m_num_values;
  AllocBPMap ( BUF_F, 6, B );
  memset( buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  
  for (i=0; i<tile_rule.size(); i++) {
    SetValF( BUF_F, tile_rule[i][0], tile_rule[i][1], tile_rule[i][2], tile_rule[i][3] );
  }
  ConstructGH();
  */


  ConstructMU();
  NormalizeMU ();

  AllocBPVec( BUF_BELIEF, m_num_values );

  // options
  //
  m_frame     = 0;  
  m_run_cuda  = false;

  AllocBuffer ( BUF_VOL, m_res, 4 );


  printf("init done\n"); fflush(stdout);

  return true;
}

bool BeliefPropagation::init(int Rx, int Ry, int Rz)
{
  int i, j;
  std::string name_fn = "examples/stair_name.csv";
  std::string rule_fn = "examples/stair_rule.csv";

  init_F_CSV(rule_fn, name_fn);

  ConstructTileIdx();

  //---
  m_rand.seed ( m_seed++ );  

  m_bpres.Set ( Rx, Ry, Rz );
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = m_tile_name.size();
  m_res.Set ( Rx, Ry, Rz );

  // F
  //
  /*
  int B = m_num_values;
  AllocBPMap ( BUF_F, 6, B );
  memset( buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  
  for (i=0; i<tile_rule.size(); i++) {
    SetValF( BUF_F, tile_rule[i][0], tile_rule[i][1], tile_rule[i][2], tile_rule[i][3] );
  }
  ConstructGH();
  */

  ConstructMU();
  NormalizeMU ();

  AllocBPVec( BUF_BELIEF, m_num_values );

  // options
  //
  m_frame     = 0;  
  m_run_cuda  = false;

  AllocBuffer ( BUF_VOL, m_res, 4 );

  printf("init done\n"); fflush(stdout);

  return true;
}

bool BeliefPropagation::_init()
{
  int i;
  std::string name_fn = "examples/stair_name.csv";
  std::string rule_fn = "examples/stair_rule.csv";
  std::vector< std::string > tile_name;
  std::vector< std::vector<float> > tile_rule;

  _read_name_csv(name_fn, tile_name);
  _read_rule_csv(rule_fn, tile_rule);

  //----
  // debug

  /*
  for (i=0; i<tile_name.size(); i++) {
    printf("%i %s\n", i, tile_name[i].c_str());
  }

  for (i=0; i<tile_rule.size(); i++) {
    printf("%f %f %f %f\n",
      tile_rule[i][0], tile_rule[i][1],
      tile_rule[i][2], tile_rule[i][3]);
  }
  */

  //---
  m_rand.seed ( m_seed++ );  

  int R = 32;
  //int R = 10;
  m_bpres.Set ( R, R, R );
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = tile_name.size();
  m_res.Set ( R, R, R );

  // F
  //
  int B = m_num_values;
  AllocBPMap ( BUF_F, 6, B );
  memset( buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  
  for (i=0; i<tile_rule.size(); i++) {
    SetValF( BUF_F, tile_rule[i][0], tile_rule[i][1], tile_rule[i][2], tile_rule[i][3] );
  }

  ConstructGH();
  ConstructMU();
  NormalizeMU ();

  AllocBPVec( BUF_BELIEF, m_num_values );

  // options
  //
  m_frame     = 0;  
  m_run_cuda  = false;

  AllocBuffer ( BUF_VOL, m_res, 4 );

  printf("init done\n"); fflush(stdout);

  return true;
}

float BeliefPropagation::step()
{
  float max_diff = 0.0;
  //char savename[256] = {'\0'};
  //char msg[256];
  Vector3DF a, b, c;
  Vector3DF p, q, d;

  // advance
  //time update  
  //
  ComputeBelief ( BUF_MU, BUF_VOL );

  printf(">>>\n"); fflush(stdout);

  max_diff = BeliefProp ();    
  UpdateMU();
  NormalizeMU();  

  printf("cp.step.0 (--> %f)\n", max_diff); fflush(stdout);

  return max_diff;
}

//---

void BeliefPropagation::filterKeep(uint64_t pos, std::vector<int32_t> &tile_id) {
  int32_t tile_idx, idx, n, tile_val, tv;

  n = getVali( BUF_TILE_IDX_N, pos );
  for (idx=0; idx<n; idx++) {

    tile_val = getVali( BUF_TILE_IDX, pos, idx );

    for (tile_idx=0; tile_idx<tile_id.size(); tile_idx++) {
      if (tile_id[tile_idx] == tile_val) { break; }
    }
    if (tile_idx < tile_id.size()) { continue; }

    n--;
    tv = getVali( BUF_TILE_IDX, pos, n );
    SetVali( BUF_TILE_IDX, pos, n, tile_val );
    SetVali( BUF_TILE_IDX, pos, idx, tv);

    SetVali( BUF_TILE_IDX_N, pos, n );

    idx--;
  }

}

void BeliefPropagation::filterDiscard(uint64_t pos, std::vector<int32_t> &tile_id) {
  int32_t tile_idx, idx, n, tile_val, tv;

  n = getVali( BUF_TILE_IDX_N, pos );
  for (idx=0; idx<n; idx++) {

    tile_val = getVali( BUF_TILE_IDX, pos, idx );

    for (tile_idx=0; tile_idx<tile_id.size(); tile_idx++) {
      if (tile_id[tile_idx] == tile_val) { break; }
    }
    if (tile_idx==tile_id.size()) { continue; }

    n--;
    tv = getVali( BUF_TILE_IDX, pos, n );
    SetVali( BUF_TILE_IDX, pos, n, tile_val );
    SetVali( BUF_TILE_IDX, pos, idx, tv);

    SetVali( BUF_TILE_IDX_N, pos, n );

    idx--;
  }

}

int32_t BeliefPropagation::tileName2ID (std::string &tile_name) {
  int32_t  i;

  for (i=0; i<m_tile_name.size(); i++) {
    if (tile_name == m_tile_name[i]) { return i; }
  }

  return -1;
}

int32_t BeliefPropagation::tileName2ID (char *cs) {
  int32_t  i;
  std::string tile_name = cs;

  for (i=0; i<m_tile_name.size(); i++) {
    if (tile_name == m_tile_name[i]) { return i; }
  }

  return -1;
}

void BeliefPropagation::debugPrint() {
  int i, n=3, m=7, jnbr=0, a;
  int a_idx, a_idx_n;
  uint64_t u;
  Vector3DI p;
  double v;

  int __a;

  std::vector< std::string > _dp_desc;
  _dp_desc.push_back("-1:0:0");
  _dp_desc.push_back("+1:0:0");
  _dp_desc.push_back("0:-1:0");
  _dp_desc.push_back("0:+1:0");
  _dp_desc.push_back("0:0:-1");
  _dp_desc.push_back("0:0:+1");

  printf("m_res: (%i,%i,%i)\n", m_res.x, m_res.y, m_res.z);
  printf("m_bpres: (%i,%i,%i)\n", m_bpres.x, m_bpres.y, m_bpres.z);
  printf("m_num_verts: %i, m_num_values: %i\n", (int)m_num_verts, (int)m_num_values);

  printf("m_tile_name[%i]:\n", (int)m_tile_name.size());
  for (i=0; i<m_tile_name.size(); i++) {
    if ((i%m)==0) { printf("\n"); }
    v = getVal( BUF_G, i );
    printf(" %s(%2i):%0.4f)", m_tile_name[i].c_str(), i, (float)v);
  }
  printf("\n\n");

  for (u=0; u<m_num_verts; u++) {
    p = getVertexPos(u);

    a_idx_n = getVali( BUF_TILE_IDX_N, u );

    printf("[%i,%i,%i](%i):\n", (int)p.x, (int)p.y, (int)p.z, (int)u);
    for (a_idx=0; a_idx<a_idx_n; a_idx++) {
      a = getVali( BUF_TILE_IDX, (int)u, (int)a_idx );

      __a = tileName2ID( m_tile_name[a] );

      printf("  %s(%2i): ", m_tile_name[a].c_str(), a);
      //printf("  %s(%i,%i): ", m_tile_name[a].c_str(), a, __a);

      for (jnbr=0; jnbr<getNumNeighbors(u); jnbr++) {
        v = getVal( BUF_MU, jnbr, u, a );
        printf(" [%s](%i):%f", _dp_desc[jnbr].c_str(), jnbr, v);
      }
      printf("\n");
    }
    printf("\n");
  }

}

//---
//---
//---

// custom size (basic test)
//
int test0() {
  BeliefPropagation bp;
  bp.init(4,3,1);
  bp.debugPrint();
  return 0;
}

// test filterDiscard
//
int test1() {
  std::vector<int32_t> discard_list;

  discard_list.push_back(35);
  discard_list.push_back(36);
  discard_list.push_back(37);
  discard_list.push_back(38);
  discard_list.push_back(39);
  discard_list.push_back(40);
  discard_list.push_back(41);
  discard_list.push_back(42);
  discard_list.push_back(43);
  discard_list.push_back(44);

  BeliefPropagation bp;
  bp.init(4,3,1);

  bp.filterDiscard(6, discard_list);
  bp.debugPrint();
  return 0;
}

// test filterKeep
//
int test2() {
  std::vector<int32_t> keep_list;

  keep_list.push_back(1);
  keep_list.push_back(2);
  keep_list.push_back(3);
  keep_list.push_back(4);
  keep_list.push_back(5);
  keep_list.push_back(6);

  BeliefPropagation bp;
  bp.init(4,3,1);

  bp.filterKeep(6, keep_list);
  bp.debugPrint();
  return 0;
}

int test3() {
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  bp.init(4,3,1);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"+000") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(3,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(3,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(3,0,0), keep_list);

  //---

  bp.NormalizeMU();  

  bp.debugPrint();

  return 0;
}

// DEBUG MAIN
//
int main(int argc, char **argv) {

  //test0();
  //test1();
  //test2();
  test3();

  return 0;

  BeliefPropagation bp;
  bp.init(32);
  while (true) {
    bp.step();
  }
  printf("hello\n");
}

