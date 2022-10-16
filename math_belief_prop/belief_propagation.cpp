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

#define BUF_VOL      0    // volume: n^3
#define BUF_G      1    // beliefprop, G(a) vector
#define BUF_H      2    // beliefprop, H(a) vector
#define BUF_F      3    // beliefprop, F(a,b) vector - assume independent of i,j
#define BUF_MU      4    // beliefprop, mu{i,j}(a,b) vector 
#define BUF_MU_NXT    5    // beliefprop, mu'{i,j}(a,b) vector
#define BUF_BELIEF    6    // Belief array

class BeliefPropagation {
public:
  virtual bool init();
  virtual void step();

  // volumes
  void    AllocBuffer(int id, Vector3DI res, int chan=1);

  // belief prop
  void    Restart();
  void    AllocBPVec (int id, int cnt);                  // vector alloc  
  void    AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals);  // matrix alloc
  void    AllocBPMap (int id, int nbrs, int vals);
  uint64_t  getNeighbor(uint64_t j, int nbr);        // 3D spatial neighbor function
  Vector3DI  getVertexPos(uint64_t j);
  uint64_t  getVertex(int x, int y, int z);
  float    getVertexBelief ( uint64_t j );
  float    _getVertexBelief ( uint64_t j, float* bi);
  int      getNumNeighbors(int j)        {return 6;}     
  int      getNumValues(int j)          {return m_num_values;}
  int      getNumVerts()            {return m_num_verts;}
  
  // belief matrix packing
  float    getVal(int id, int a)        {return *(float*) buf[id].getPtr (a);}            // G and H vectors, size B
  void    SetVal(int id, int a, float val)  {*(float*) buf[id].getPtr(a) = val;}
  float    getVal(int id, int n, int j, int a) {return *(float*) buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }  // mu matrix, NxDxB, where D=R^3, N=nbrs=6
  void    SetVal(int id, int n, int j, int a, float val ) { *(float*) buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ) = val; }
  float    getValF(int id, int a, int b, int n)      { return *(float*) buf[id].getPtr ( (b*m_num_values + a)*6 + n ); }  // belief mapping (f), BxB
  void    SetValF(int id, int a, int b, int n, float val ) { *(float*) buf[id].getPtr ( (b*m_num_values + a)*6 + n ) = val; }

  float   BeliefProp();
  void    ComputeBelief (int id, int id_vol);
  void    UpdateMU ();

  void    ConstructF ();
  void    ConstructGH ();
  void    ConstructMU ();
  void    NormalizeMU ();
  
  uint64_t  m_num_verts;    // Xi = 0..X (graph domain)
  uint64_t  m_num_values;    //  B = 0..Bm-1 (value domain)  
  Vector3DI  m_bpres;      // 3D spatial belief prop res

  Vector3DI  m_res;        // volume res

  DataPtr    buf[128];      // data buffers (CPU & GPU)  

  int      mouse_down;  
  bool    m_run;
  bool    m_run_cuda;
  bool    m_save;
  float    m_frame;
  int      m_peak_iter;
  int      m_seed;

  Mersenne  m_rand;
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

  // for all i->j messages in graph domain
  //
  // this is a 3D grid/ ---|
  //                       v
  for ( uint64_t j=0; j < getNumVerts(); j++ ) {


    // 6 neighbors of j in 3D --|
    //                          v
    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, in);
      if (i == -1) { continue; }

      // compute message from i to j      
      // for each a..
      //
      for (int a=0; a < getNumValues(j); a++) {

        // first compute Hij_t
        //
        // initialize Hij(a) = gi(a)/
        //
        H_ij_a = getVal(BUF_G, a);

        for (int kn=0; kn < getNumNeighbors(i); kn++ ) {
          k = getNeighbor(i, kn);

          // Hij(a) = gi(a) * PROD mu{ki}_a
          //
          if (k!=-1 && k!=j) {
            H_ij_a *= getVal(BUF_MU, kn, i, a);

            //DEBUG
            //printf("  h_ij_a *= mu[%i][%i][%i] (=%f)\n", kn, (int)i, a, getVal(BUF_MU, kn, i, a) );
          }
        }

        // exclude mu{ji}_a - message from j to i
        //
        //mu_j = getVal(BUF_MU, j, i, a);
        //if (mu_j > 0) H_ij_a /= mu_j;
        //
        SetVal (BUF_H, a, H_ij_a);
      }

      // now compute mu_ij_t+1 = Fij * hij         
      //max_mu = 0;
      //
      //
      // b = rows in f{ij}(a,b), also elements of mu(b)/
      //
      for (int b=0; b < getNumValues(j); b++) {
        u_nxt_b = 0;          

        // a = cols in f{ij}(a,b), also elements of h(a)
        //
        for (int a=0; a < getNumValues(j); a++) {
          u_nxt_b += getValF(BUF_F, a, b, in) * getVal(BUF_H, a);

          //DEBUG
          //printf("  (f[%i][%i][%i] %f)*(h[%i] %f)\n", (int)a, (int)b, (int)in, (float)getValF(BUF_F, a, b, in), (int)a, (float)getVal(BUF_H, a));
        }
        u_prev_b = getVal(BUF_MU, in, j, b);

        // in = neighbor index of i, for mu{i,j}(b)

        du = u_nxt_b - u_prev_b;

        //DEBUG
        //printf("::: du %f (%f, %f)\n", du, u_nxt_b, u_prev_b);

        if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }

        //if ( j < 8 ) dbgprintf ( "%d: %f\n", j, du );
        //SetVal (BUF_MU_NXT, in, j, b, u_prev_b + du*rate );
        //
        SetVal (BUF_MU_NXT, in, j, b, u_prev_b + du );
        
      }
    }
  }

  return max_diff;
}

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
    bi[a] = 1.0;  //getVal(BUF_G, a);
    for (int kn=0; kn < getNumNeighbors(j); kn++) {
      k = getNeighbor(j, kn);
      if (k!=-1) bi[a] *= getVal(BUF_MU, kn, j, a );    // mu{k,j}(a)
    }
    sum += bi[a];
  }
  if ( sum > 0 )
    for (int a=0; a < m_num_values; a++) 
      bi[a] /= sum;
  
  return sum;
}

float BeliefPropagation::getVertexBelief ( uint64_t j )
{
  uint64_t k;
  float sum = 0;
  float _bi = 1.0;

  for (int a=0; a < m_num_values; a++) {
    SetVal( BUF_BELIEF, a, 1.0 );
    //bi[a] = 1.0;  //getVal(BUF_G, a);
    for (int kn=0; kn < getNumNeighbors(j); kn++) {
      k = getNeighbor(j, kn);
      //if (k!=-1) bi[a] *= getVal(BUF_MU, kn, j, a );    // mu{k,j}(a)
      if (k!=-1) {

        // mu{k,j}(a)
        //
        _bi = getVal( BUF_BELIEF, a) * getVal(BUF_MU, kn, j, a );
        SetVal( BUF_BELIEF, a, _bi );
      }
    }
    //sum += bi[a];
    sum += getVal( BUF_BELIEF, a );
  }
  if ( sum > 0 ) {
    for (int a=0; a < m_num_values; a++) {
      _bi = getVal( BUF_BELIEF, a ) / sum;
      SetVal( BUF_BELIEF, a, _bi );
      //bi[a] /= sum;
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

bool BeliefPropagation::init()
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

  //int R = 32;
  int R = 10;
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
  m_run       = true;
  m_save      = false;

  AllocBuffer ( BUF_VOL, m_res, 4 );

  printf("init done\n"); fflush(stdout);

  return true;

  /*
  // G - global individual (independent) tile probability
  // H - messsage passing vector
  //
  AllocBPVec ( BUF_G, m_num_values );
  AllocBPVec ( BUF_H, m_num_values );
  
  float weight = 1.0 / (float)m_num_values;
  for (int a=0; a < m_num_values; a++ ) {
    SetVal ( BUF_G, a, weight );
  }

  return true;
  */

  #ifdef USE_OPENGL
    init2D("arial");
    setText(18,1);
  #endif

  // options
  m_frame = 0;  
  m_run_cuda = false;    // run cuda pathway  
  m_run = true;      // run belief prop
  m_save = true;      // save to disk
  
  //int R = 32;
  R = 32;

  // belief propagation setup
  m_bpres.Set ( R, R, R );    // D = R^3
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = 4;
  
  m_res.Set ( R, R, R );              // volume resolution

  Restart();

  #ifdef USE_CUDA    
    if ( m_run_cuda ) {
      CUcontext ctx; 
      CUdevice dev;
      cuStart ( DEV_FIRST, 0, dev, ctx, 0, true );    // start CUDA
    }
  #endif

  AllocBuffer ( BUF_VOL, m_res, 4 );          // allocate color volume (4 channel)

  return true;

}

void BeliefPropagation::step()
{
  float md= 0.0;
  char savename[256] = {'\0'};
  char msg[256];
  Vector3DF a, b, c;
  Vector3DF p, q, d;

  // advance
  //time update  
  //
  if (m_run) {
    ComputeBelief ( BUF_MU, BUF_VOL );

    printf(">>>\n"); fflush(stdout);

    md = BeliefProp ();    
    UpdateMU();
    NormalizeMU();  

    printf("cp.step.0 (--> %f)\n", md); fflush(stdout);
  }

  dbgprintf ( "Running..\n" );
}

// DEBUG MAIN
int main(int argc, char **argv) {
  printf("hello\n");
}
