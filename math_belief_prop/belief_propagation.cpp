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

#include "belief_propagation.h"


//---- belief prop buffers
// vector alloc (h and g)P
//
void BeliefPropagation::AllocBPVec (int id, int cnt) {
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  m_buf[id].Resize( sizeof(float), cnt, 0x0, flags );

  memset( (void *)(m_buf[id].getPtr(0)), 0, sizeof(float)*cnt);
}

void BeliefPropagation::AllocViz (int id, uint64_t cnt )
{
  uint64_t sz = cnt * sizeof(float);         
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  m_buf[id].Resize( sizeof(float), cnt, 0x0, flags );
  memset( (void *) (m_buf[id].getPtr(0)), 0, sz );
}



void BeliefPropagation::ZeroBPVec (int id ) {
  m_buf[id].FillBuffer ( 0 );
}

// allocate message matrix mu
// belief matrix alloc (mu)
//
void BeliefPropagation::AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals) {
  // NOTE: matrix is stored sparesly. 
  // full matrix: mem=D*D*B, mu{D->D}[B] is full vertex-to-vertex messages over values B, where D=R^3, e.g. D=64^3, B=4, mem=262144^2*4 floats= 1 terabyte
  // sparse mtrx: mem=6*D*B, mu{6->D}[B] since only 6x neigbors are non-zero in 3D. explicitly index those six.
  // final: mem = 6*D*B, e.g. D=64^3, B=4, mem=6*262144*4 floats = 25 megabytes
  uint64_t cnt = nbrs * verts * vals;          
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  m_buf[id].Resize( sizeof(float), cnt, 0x0, flags );

  memset( (void *)(m_buf[id].getPtr(0)), 0, sizeof(float)*cnt);
}


// belief value mapping (f)
//
void BeliefPropagation::AllocBPMap (int id, int nbrs, int vals) {
  uint64_t cnt = vals * vals * nbrs;    // size B*B*6
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;

  m_buf[id].Resize( sizeof(float), cnt, 0x0, flags );

  memset( (void *)(m_buf[id].getPtr(0)), 0, sizeof(float)*cnt);
}

// belief value mapping (f)
//
void BeliefPropagation::AllocTileIdx(int id, int nvert, int nval) {
  uint64_t cnt = nvert * nval ;
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  m_buf[id].Resize( sizeof(int32_t), cnt, 0x0, flags );

  memset( (void *)(m_buf[id].getPtr(0)), 0, sizeof(int32_t)*cnt);
}

// belief value mapping (f)
//
void BeliefPropagation::AllocTileIdxN(int id, int nval) {
  uint64_t cnt = nval ;
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
  m_buf[id].Resize( sizeof(int32_t), cnt, 0x0, flags );

  memset( (void *)(m_buf[id].getPtr(0)), 0, sizeof(int32_t)*cnt);
}

void BeliefPropagation::AllocVeci32(int id, int nval) {
  uint64_t cnt = nval;
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;

  m_buf[id].Resize( sizeof(int32_t), cnt, 0x0, flags );

  memset( (void *)(m_buf[id].getPtr(0)), 0, sizeof(int32_t)*cnt);
}

void BeliefPropagation::AllocVeci32(int id, int nval, int b) {
  uint64_t cnt = nval*b;
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;

  m_buf[id].Resize( sizeof(int32_t), cnt, 0x0, flags );

  memset( (void *)(m_buf[id].getPtr(0)), 0, sizeof(int32_t)*cnt);
}


//---

int64_t BeliefPropagation::getVertex(int x, int y, int z) {
  return int64_t(z*m_bpres.y + y)*m_bpres.x + x;
}

// domain index to 3D pos
//
Vector3DI BeliefPropagation::getVertexPos(int64_t j) {
  Vector3DI p;
  p.z = j / (m_bpres.x*m_bpres.y);  j -= p.z * (m_bpres.x*m_bpres.y);
  p.y = j / m_bpres.x;        j -= p.y * m_bpres.x;
  p.x = j;
  return p;
}

// get 3D grid neighbor 
//
int64_t BeliefPropagation::getNeighbor( uint64_t j, int nbr ) {
  Vector3DI jp = getVertexPos(j);

  // 3D spatial neighbor function
  //
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
  AllocVeci32( BUF_TILE_IDX, m_num_verts * m_num_values );
  AllocVeci32( BUF_TILE_IDX_N, m_num_verts );
  for (i=0; i<m_num_verts; i++) {
    SetVali( BUF_TILE_IDX_N, i, m_num_values );
    for (j=0; j<m_num_values; j++) {
      SetVali( BUF_TILE_IDX, i, j, (int32_t)j );
    }
  }
}

void BeliefPropagation::ConstructConstraintBuffers() {
  int i, j, x;
  int32_t x32;

  AllocVeci32( BUF_CONSIDER, m_num_verts );
  AllocVeci32( BUF_VISITED, m_num_verts );

  AllocVeci32( BUF_NOTE, 2, m_num_verts );

  m_note_n[0] = 0;
  m_note_n[1] = 0;
  m_grid_note_idx = 0;
}


void BeliefPropagation::ConstructF () {
  int B = m_num_values;

  AllocBPMap ( BUF_F, 6, B );

  // randomly enable interactions among values  
  //
  memset( m_buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  

}

void BeliefPropagation::ConstructGH () {
  AllocBPVec ( BUF_G, m_num_values );
  AllocBPVec ( BUF_H, m_num_values );
  
  float weight = 1.0 / m_num_values;
  for (int a=0; a < m_num_values; a++ ) {
    SetVal ( BUF_G, a, weight );
  }
}



void BeliefPropagation::ConstructMU () {
  AllocBPMtx ( BUF_MU, 6, m_num_verts, m_num_values );
  AllocBPMtx ( BUF_MU_NXT, 6, m_num_verts, m_num_values );

  float w;
  float *mu = (float*) m_buf[BUF_MU].getData();
  uint64_t cnt = 6 * m_num_verts * m_num_values;
  memset ( mu, 0, cnt * sizeof(float) );

  int i;
  for (int j=0; j < m_num_verts; j++) {
    for (int jnbr=0; jnbr < getNumNeighbors(j); jnbr++) {
      i = getNeighbor(j, jnbr);
      for (int a=0; a < m_num_values;a++) {
        w = m_rand.randF();

        // randomize MU
        //
        SetVal( BUF_MU, jnbr, j, a, w );
      }
    }
  }
}

void BeliefPropagation::ComputeDiffMUField () {
  int i, n_a, a;
  float v0,v1, d, max_dmu;

  for (int j=0; j < m_num_verts; j++) {
    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, in);
      n_a = getVali( BUF_TILE_IDX_N, j );
      max_dmu = 0;
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getVali( BUF_TILE_IDX, j, a_idx );
        v0 = getVal( BUF_MU, in, j, a );
        v1 = getVal( BUF_MU_NXT, in, j, a );
        d = fabs(v0-v1);
        if ( d > max_dmu) max_dmu = d;
      }
    }
    //printf ( "%f\n", max_dmu );
    SetVal ( BUF_VIZ, j, max_dmu );
  } 
 }

//---

float BeliefPropagation::MaxDiffMU () {
  int i, n_a, a;
  float v0,v1, d, max_diff=-1.0;

  for (int j=0; j < m_num_verts; j++) {
    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, in);
      n_a = getVali( BUF_TILE_IDX_N, j );
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getVali( BUF_TILE_IDX, j, a_idx );
        v0 = getVal( BUF_MU, in, j, a );
        v1 = getVal( BUF_MU_NXT, in, j, a );

        d = fabs(v0-v1);
        if (max_diff < d) { max_diff = d; }
      }
    }
  }

  return max_diff;
}

void BeliefPropagation::NormalizeMU () { NormalizeMU( BUF_MU ); }
void BeliefPropagation::NormalizeMU (int id) {
  int i=0, n_a=0, a=0;
  float v=0, sum=0;

  for (int j=0; j < m_num_verts; j++) {
    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, in);
      sum = 0;

      if (i==-1) {
        n_a = getVali( BUF_TILE_IDX_N, j );
        for (int a_idx=0; a_idx<n_a; a_idx++) {
          a = getVali( BUF_TILE_IDX, j, a_idx );
          SetVal( id, in, j, a, 1.0 );
        }
      }

      n_a = getVali( BUF_TILE_IDX_N, j );
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getVali( BUF_TILE_IDX, j, a_idx );
        sum += getVal( id, in, j, a );
      }

      if ( sum > 0 ) {
        n_a = getVali( BUF_TILE_IDX_N, j );
        for (int a_idx=0; a_idx<n_a; a_idx++) {
          a = getVali( BUF_TILE_IDX, j, a_idx );

          v = getVal( id, in, j, a );
          SetVal( id, in, j, a, v / sum);
        }
      }

    }
  }
}

float BeliefPropagation::BeliefProp () {  
  int64_t anch_cell=0, nei_cell=0;
  int a_idx, a_idx_n;
  int a, b, d;

  int anch_tile_idx, anch_tile_idx_n;
  int nei_tile_idx, nei_tile_idx_n;
  int anch_tile, nei_tile;
  int64_t _neinei_cell;

  int nei_in_idx, anch_in_idx;
  int nei_out_idx, anch_out_idx;
  int nei_to_anch_dir_idx;

  float H_ij_a;
  float u_nxt_b, u_prev_b;
  float mu_j, du;

  float rate = 1.0,
        max_diff=-1.0;

  rate = m_rate;

  // for all `nei`->`anch` messages in graph domain
  //
  for ( anch_cell=0; anch_cell < getNumVerts(); anch_cell++ ) {

    anch_tile_idx_n = getVali( BUF_TILE_IDX_N, anch_cell );

    // 6 neighbors of j in 3D
    //
    for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {
      nei_cell = getNeighbor(anch_cell, anch_in_idx);

      // pathological conditions
      // * cell has none (which is an error) or only 1 tile
      // * cell's neighbor falls off the end
      //
      if (anch_tile_idx_n <= 1) {
        if (anch_tile_idx_n == 1) {
          anch_tile = getVali( BUF_TILE_IDX, anch_cell, 0 );
          SetVal( BUF_MU_NXT, anch_in_idx, anch_cell, anch_tile, 1.0 );
        }
        continue;
      }
      if (nei_cell == -1) {
        SetVal( BUF_MU_NXT, anch_in_idx, anch_cell, anch_tile, 1.0 );
        continue;
      }

      // compute message from `nei` to `anch` for each a..
      // we're skipping some tiles, so zero out H
      //
      for (d=0; d<m_num_values; d++) { SetVal(BUF_H, d, 0); }

      nei_tile_idx_n = getVali( BUF_TILE_IDX_N, nei_cell );

      // pathological condition
      // * neighbor cell has 0 values (error) or only 1
      //
      if (nei_tile_idx_n <= 1) {
        for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
          anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );
          SetVal( BUF_MU_NXT, anch_in_idx, anch_cell, anch_tile, 1.0 );
        }
        continue;
      }

      for (nei_tile_idx=0; nei_tile_idx < nei_tile_idx_n; nei_tile_idx++) {

        nei_tile = getVali( BUF_TILE_IDX, nei_cell, nei_tile_idx );

        // first compute Hij_t
        // initialize Hij(a) = gi(a)
        //
        H_ij_a = getVal(BUF_G, nei_tile);

        for (nei_in_idx=0; nei_in_idx < getNumNeighbors(nei_cell); nei_in_idx++ ) {
          _neinei_cell = getNeighbor(nei_cell, nei_in_idx);

          // Hij(a) = gi(a) * PROD mu{ki}_a
          //
          if ((_neinei_cell != -1) && (_neinei_cell != anch_cell)) {
            H_ij_a *= getVal(BUF_MU, nei_in_idx, nei_cell, nei_tile);
          }

        }

        SetVal (BUF_H, nei_tile, H_ij_a);

      }

      // now compute mu_ij_t+1 = Fij * hij
      // b = rows in f{ij}(a,b), also elements of mu(b)/
      //
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );

        u_nxt_b = 0.0;

        // a = cols in f{ij}(a,b), also elements of h(a)
        //
        for (d=0; d < m_num_values; d++) {

        // experimental
        //for (nei_tile_idx=0; nei_tile_idx<nei_tile_idx_n; nei_tile_idx++) {
        //  d = getVali( BUF_TILE_IDX, nei_cell, nei_tile_idx );
        // experimental

          nei_to_anch_dir_idx = m_dir_inv[anch_in_idx];
          u_nxt_b += getValF(BUF_F, d, anch_tile, nei_to_anch_dir_idx) * getVal(BUF_H, d);
        }
        u_prev_b = getVal(BUF_MU, anch_in_idx, anch_cell, anch_tile);

        du = u_nxt_b - u_prev_b;

        if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }

        SetVal (BUF_MU_NXT, anch_in_idx, anch_cell, anch_tile, u_prev_b + du*rate );
      }
    }
  }

  return max_diff;
}

// copy BUF_MU_NXT back to BUF_MU
//
void BeliefPropagation::UpdateMU () {
  float* mu_curr = (float*) m_buf[BUF_MU].getData();
  float* mu_next = (float*) m_buf[BUF_MU_NXT].getData();

  uint64_t cnt = 6 * m_num_verts * m_num_values;
  memcpy ( mu_curr, mu_next, cnt * sizeof(float) );
}

//----

void BeliefPropagation::cellUpdateBelief(int64_t anch_cell) {
  int64_t nei_cell=0;
  int32_t anch_tile=0, anch_tile_idx=0, anch_tile_idx_n=0;
  int dir_idx;
  float sum=0.0,
        _eps = m_eps_zero;
  float _b_i_t_a = 0.0;

  sum = 0.0;

  anch_tile_idx_n = getVali( BUF_TILE_IDX_N, anch_cell );
  for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
    anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );

    SetVal( BUF_BELIEF, anch_tile, 1.0 );

    _b_i_t_a = getVal( BUF_G, anch_tile );

    for (dir_idx=0; dir_idx < getNumNeighbors(anch_cell); dir_idx++) {
      nei_cell = getNeighbor(anch_cell, dir_idx);
      if (nei_cell < 0) { continue; }

      _b_i_t_a *= getVal( BUF_MU, dir_idx, anch_cell, anch_tile );
    }
    SetVal(BUF_BELIEF, anch_tile, _b_i_t_a);
    sum += getVal( BUF_BELIEF, anch_tile);
  }

  if (sum > _eps) {
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );
      _b_i_t_a = getVal( BUF_BELIEF, anch_tile ) / sum;
      SetVal( BUF_BELIEF, anch_tile, _b_i_t_a );
    }
  }

}

int BeliefPropagation::chooseMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief) {
  int64_t anch_cell=0;
  int32_t anch_tile_idx, anch_tile_idx_n, anch_tile;
  int count=0;

  float _max_belief = -1.0, f, p;
  int64_t _max_cell = -1;
  int32_t _max_tile = -1,
          _max_tile_idx = -1;

  //float _eps = (1.0/(1024.0*1024.0));
  float _eps = m_eps_zero;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {
    cellUpdateBelief(anch_cell);

    anch_tile_idx_n = getVali( BUF_TILE_IDX_N, anch_cell );
    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) { continue; }

    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );

      f = getVal( BUF_BELIEF, anch_tile );

      if (f > (_max_belief-_eps)) {

        if (f > _max_belief) {
          _max_cell = anch_cell;
          _max_tile = anch_tile;
          _max_tile_idx = anch_tile_idx;
          _max_belief = f;
          count=1;
        }

        // randomize 'equal' choices
        //
        else {
          count++;
          p = m_rand.randF();
          if ( p < (1.0/(float)count) ) {
            _max_cell = anch_cell;
            _max_tile = anch_tile;
            _max_tile_idx = anch_tile_idx;
            _max_belief = f;
          }
        }

      }

    }

  }

  if (max_cell) { *max_cell = _max_cell; }
  if (max_tile) { *max_tile = _max_tile; }
  if (max_tile_idx)  { *max_tile_idx = _max_tile_idx; }
  if (max_belief) { *max_belief = _max_belief; }

  return count;
}

int BeliefPropagation::chooseMaxEntropy(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_entropy) {
  int64_t anch_cell=0;
  int32_t anch_tile_idx, anch_tile_idx_n, anch_tile;
  int count=0;

  float _max_entropy= -1.0, p, g, _entropy, g_sum, g_cdf;
  int64_t _max_cell = -1;
  int32_t _max_tile = -1,
          _max_tile_idx = -1;

  //float _eps = (1.0/(1024.0*1024.0));
  float _eps = m_eps_zero;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {

    anch_tile_idx_n = getVali( BUF_TILE_IDX_N, anch_cell );
    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) { continue; }

    g_sum = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );
      g = getVal( BUF_G, anch_tile );
      g_sum += g;
    }

    if (g_sum < _eps) { continue; }

    _entropy = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );
      g = getVal( BUF_G, anch_tile ) / g_sum;
      if (g > _eps) {
        _entropy += -g * logf(g);
      }
    }

    if (_entropy > (_max_entropy - _eps)) {

      if (_entropy > _max_entropy) {
        _max_entropy  = _entropy;
        _max_cell     = anch_cell;
        //_max_tile     = anch_tile;
        //_max_tile_idx = anch_tile_idx;
        count=1;
      }

      // randomize 'equal' choices
      //
      else {
        count++;
        p = m_rand.randF();
        if ( p < (1.0/(float)count) ) {
          _max_entropy  = _entropy;
          _max_cell     = anch_cell;
          //_max_tile     = anch_tile;
          //_max_tile_idx = anch_tile_idx;
        }
      }

    }

  }

  if (_max_cell<0) { return 0; }

  _max_tile = getVali( BUF_TILE_IDX, _max_cell, 0 );
  _max_tile_idx = 0;

  // now we choose a particular tile from the tile position
  // (both tile ID and tile index)
  //
  anch_cell = _max_cell;
  anch_tile_idx_n = getVali( BUF_TILE_IDX_N, anch_cell );
  g_sum = 0.0;
  for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
    anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );
    g_sum += getVal( BUF_G, anch_tile);
  }

  if (g_sum > _eps) {

    p = m_rand.randF();

    g_cdf = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );
      g_cdf += getVal( BUF_G, anch_tile ) / g_sum;

      if (p < g_cdf) {
        _max_tile     = anch_tile;
        _max_tile_idx = anch_tile_idx;
        break;
      }
    }

  }

  if (max_cell)     { *max_cell     = _max_cell; }
  if (max_tile)     { *max_tile     = _max_tile; }
  if (max_tile_idx) { *max_tile_idx = _max_tile_idx; }
  if (max_entropy)  { *max_entropy  = _max_entropy; }

  return count;
}

//----

float BeliefPropagation::getVertexBelief ( uint64_t j ) {
  int64_t k;
  int a, kn;
  float sum = 0.0;
  float _bi = 1.0;

  int64_t tile_idx_n,
          tile_idx,
          tile_val;

  for (a=0; a < m_num_values; a++) {
    SetVal( BUF_BELIEF, a, 0.0 );
  }

  tile_idx_n= getVali( BUF_TILE_IDX_N, j );
  for (tile_idx=0; tile_idx<tile_idx_n; tile_idx++) {
    tile_val = getVali( BUF_TILE_IDX, j, tile_idx );

    _bi = 1.0;
    for (kn=0; kn<getNumNeighbors(j); kn++) {
      k = getNeighbor(j, kn);
      if (k==-1) { continue; }

      _bi *= getVal(BUF_MU, kn, j, tile_val);
    }
    SetVal(BUF_BELIEF, tile_val, _bi);

    sum += _bi;
  }

  if (sum > m_eps_zero) {
    for (tile_idx=0; tile_idx<tile_idx_n; tile_idx++) {
      tile_val = getVali( BUF_TILE_IDX, j, tile_idx );

      _bi = getVal( BUF_BELIEF, tile_val ) / sum;
      SetVal( BUF_BELIEF, tile_val, _bi );
    }

  }

  return sum;
}

float BeliefPropagation::_getVertexBelief ( uint64_t j ) {
  int64_t k;
  int a, kn;
  float sum = 0;
  float _bi = 1.0;

  for (a=0; a < m_num_values; a++) {
    SetVal( BUF_BELIEF, a, 1.0 );

    for (kn=0; kn < getNumNeighbors(j); kn++) {
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
    for (a=0; a < m_num_values; a++) {
      _bi = getVal( BUF_BELIEF, a ) / sum;
      SetVal( BUF_BELIEF, a, _bi );
    }
  }
  
  return sum;
}

void BeliefPropagation::Restart()
{
  m_rand.seed ( m_seed++ );  

  ConstructF ();
  ConstructGH ();
  ConstructMU ();
  NormalizeMU ();

  ConstructTileIdx();
  ConstructConstraintBuffers();

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

// constraint file format:
//
// <x>,<y>,<z>,<tile_id>
//
int _read_constraint_csv(std::string &fn, std::vector< std::vector<int32_t> > &admissible_tile) {
  int i;
  float val, _weight;
  FILE *fp;
  std::string line, tok;
  std::vector<std::string> toks;
  std::vector<float> v;

  float _tile_src, _tile_dst;
  int x, y, z, tileid;

  std::vector< int32_t > v32;

  fp = fopen(fn.c_str(), "r");
  if (!fp) { return -1; }

  while (!feof(fp)) {
    line.clear();
    _read_line(fp, line);

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

    x = atoi(toks[0].c_str());
    y = atoi(toks[1].c_str());
    z = atoi(toks[2].c_str());
    tileid = atoi(toks[3].c_str());

    v32.clear();
    v32.push_back(x);
    v32.push_back(y);
    v32.push_back(z);
    v32.push_back(tileid);

    admissible_tile.push_back(v32);    

  }
  fclose (fp);

  return 0;
}

//----

int BeliefPropagation::filter_constraint(std::vector< std::vector< int32_t > > &constraint_list) {
  int i;
  int32_t tile_id, x, y, z, n;
  int64_t pos;

  for (pos=0; pos<m_num_verts; pos++) {
    SetVali( BUF_TILE_IDX_N, pos, 0 );
  }

  for (i=0; i<constraint_list.size(); i++) {

    if ( constraint_list[i].size() != 4 ) {
        printf ( "error: constraints don't have 4.\n");
        exit(-1);
    }

    x = constraint_list[i][0];
    y = constraint_list[i][1];
    z = constraint_list[i][2];

    tile_id = constraint_list[i][3];

    pos = getVertex(x,y,z);
    if ((pos < 0) || (pos >= m_num_verts)) { continue; }

    if ( x < 0 || x >= m_res.x || y <0 || y >= m_res.y || z<0 || z >= m_res.z) {
        printf ( "error: constraint out of range: %d,%d,%d\n", x,y,z);
        exit(-1);
    }

    n = getVali( BUF_TILE_IDX_N, pos );
    SetVali( BUF_TILE_IDX, pos, n,  tile_id );
    n++;
    SetVali( BUF_TILE_IDX_N, pos, n );

  }

  return 0;
}

int BeliefPropagation::init_F_CSV(std::string &rule_fn, std::string &name_fn) {
  int i, ret;
  int b, maxb=-1, B;

  std::vector< std::vector<float> > tile_rule;

  ret = _read_name_csv(name_fn, m_tile_name);
  if (ret < 0) { return ret; }
  ret = _read_rule_csv(rule_fn, tile_rule);
  if (ret < 0) { return ret; }

  // use tile_rule to determine maximum number of tiles
  //
  for (i=0; i<tile_rule.size(); i++) {
    b = (int)tile_rule[i][0];
    if (b>maxb) { maxb = b; }
    b = (int)tile_rule[i][1];
    if (b>maxb) { maxb = b; }
  }
  m_num_values = maxb+1;

  printf(">>>> %i %i (%i)\n", (int)m_num_values, (int)(maxb+1), maxb);

  //---

  // F
  //
  B = m_num_values;
  AllocBPMap ( BUF_F, 6, B );
  memset( m_buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  

  for (i=0; i<tile_rule.size(); i++) {
    SetValF( BUF_F, tile_rule[i][0], tile_rule[i][1], tile_rule[i][2], tile_rule[i][3] );
  }

  ConstructGH();

  return 0;
}

void BeliefPropagation::init_dir_desc() {
  m_dir_desc.push_back("+1:0:0");
  m_dir_desc.push_back("-1:0:0");
  m_dir_desc.push_back("0:+1:0");
  m_dir_desc.push_back("0:-1:0");
  m_dir_desc.push_back("0:0:+1");
  m_dir_desc.push_back("0:0:-1");
}


//DEBUG
//
int BeliefPropagation::init(int R) {
  std::string name_fn = "examples/stair_name.csv";
  std::string rule_fn = "examples/stair_rule.csv";
  return init_CSV(R, name_fn, rule_fn);
}
int BeliefPropagation::init(int Rx, int Ry, int Rz) {
  std::string name_fn = "examples/stair_name.csv";
  std::string rule_fn = "examples/stair_rule.csv";
  return init_CSV(Rx, Ry, Rz, name_fn, rule_fn);
}
//
//DEBUG

//----

int BeliefPropagation::init_CSV(int R, std::string &name_fn, std::string &rule_fn) {
  int i, j, ret;

  m_rate = 0.98;

  //std::string name_fn = "examples/stair_name.csv";
  //std::string rule_fn = "examples/stair_rule.csv";
  //std::vector< std::string > tile_name;
  //std::vector< std::vector<float> > tile_rule;

  init_dir_desc();

  m_dir_inv[0] = 1;
  m_dir_inv[1] = 0;
  m_dir_inv[2] = 3;
  m_dir_inv[3] = 2;
  m_dir_inv[4] = 5;
  m_dir_inv[5] = 4;

  ret = init_F_CSV(rule_fn, name_fn);
  if (ret<0) { return ret; }

  //---

  m_rand.seed ( m_seed++ );  

  m_bpres.Set ( R, R, R );
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = m_tile_name.size();
  m_res.Set ( R, R, R );

  ConstructTileIdx();
  ConstructConstraintBuffers();

  ConstructMU();
  NormalizeMU ();

  AllocBPVec( BUF_BELIEF, m_num_values );

  AllocViz ( BUF_VIZ, m_num_verts );

  // options
  //  
  m_run_cuda  = false;

  return 0;
}

int BeliefPropagation::init_CSV(int Rx, int Ry, int Rz, std::string &name_fn, std::string &rule_fn) {
  int i, j, ret = 0;

  m_rate = 0.98;

  init_dir_desc();

  m_dir_inv[0] = 1;
  m_dir_inv[1] = 0;
  m_dir_inv[2] = 3;
  m_dir_inv[3] = 2;
  m_dir_inv[4] = 5;
  m_dir_inv[5] = 4;

  ret = init_F_CSV(rule_fn, name_fn);
  if (ret<0) { return ret; }

  //---

  m_rand.seed ( m_seed++ );  

  m_bpres.Set ( Rx, Ry, Rz );
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = m_tile_name.size();
  m_res.Set ( Rx, Ry, Rz );

  ConstructTileIdx();
  ConstructConstraintBuffers();

  ConstructMU();
  NormalizeMU ();

  AllocBPVec( BUF_BELIEF, m_num_values );

  AllocViz ( BUF_VIZ, m_num_verts );

  // options
  //
  m_run_cuda  = false;

  return 0;
}

bool BeliefPropagation::_init() {
  int i;
  std::string name_fn = "examples/stair_name.csv";
  std::string rule_fn = "examples/stair_rule.csv";
  std::vector< std::string > tile_name;
  std::vector< std::vector<float> > tile_rule;

  _read_name_csv(name_fn, tile_name);
  _read_rule_csv(rule_fn, tile_rule);

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
  memset( m_buf[BUF_F].getData(), 0, 6*B*B*sizeof(float) );  
  for (i=0; i<tile_rule.size(); i++) {
    SetValF( BUF_F, tile_rule[i][0], tile_rule[i][1], tile_rule[i][2], tile_rule[i][3] );
  }

  ConstructGH();
  ConstructMU();
  NormalizeMU ();

  AllocBPVec( BUF_BELIEF, m_num_values );

  // options
  //  
  m_run_cuda  = false;

  printf("init done\n"); fflush(stdout);

  return true;
}

//---

int BeliefPropagation::wfc() {

    int ret = 1;

    wfc_start();

    for (int64_t it = 0; it < m_num_verts; it++) {
        ret = wfc_step (it );

        if ( ret==0 ) break;

        if ( ret < 0) { 
          switch (ret) {          
          case -1: printf ( "wfc chooseMaxBelief error.\n" ); break;
          case -2: printf ( "wfc tileIdxCollapse error.\n" ); break;
          case -3: printf ( "wfc cellConstraintPropagate error.\n" ); break;
          };          
        } 
    }
    return ret;
}

int BeliefPropagation::wfc_start() {

  CullBoundary();

  return 1;
}

int BeliefPropagation::wfc_step(int64_t it) {
  
  int ret;
  int64_t cell=-1;
  int32_t tile=-1, tile_idx=-1;
  float entropy=-1.0, d = -1.0;

  int64_t step_iter=0;

  ret = chooseMaxEntropy( &cell, &tile, &tile_idx, &entropy);
  if (ret < 0) { return -1; }
  if (ret==0) { return 0; }

  printf("wfc[%i]: cell:%i, tile:%i, tile_idx:%i, entropy:%f (ret:%i)\n",
    (int)it, (int)cell, (int)tile, (int)tile_idx, (float)entropy, (int)ret);

  ret = tileIdxCollapse( cell, tile_idx );
  if (ret < 0) { return -2; }

  m_note_n[ m_grid_note_idx ] = 0;
  m_note_n[ 1-m_grid_note_idx ] = 0;

  cellFillAccessed(cell, m_grid_note_idx);
  unfillAccessed(m_grid_note_idx);
  //ret = cellConstraintPropagate(cell);
  ret = cellConstraintPropagate();
  if (ret < 0) { return -3; }

  return 1;
}


int BeliefPropagation::start () {
  CullBoundary();
  return 1;
}

int BeliefPropagation::single_realize_cb (int64_t it, void (*cb)(void *)) {
  int ret;
  int64_t cell=-1;
  int32_t tile=-1, tile_idx=-1;
  float belief=-1.0, d = -1.0;
  
  int64_t step_iter=0,
          max_step_iter = m_max_iteration;

  float _eps = m_eps_converge;
  
  // after we've propagated constraints, BUF_MU
  // needs to be renormalized
  //
  NormalizeMU();

  // iterate bp until converged
  //
  for (step_iter=0; step_iter<max_step_iter; step_iter++) {
    d = step();

    if (cb && ((step_iter % m_step_cb) == 0)) {
      m_state_info_d = d;
      m_state_info_iter = step_iter;
      cb(NULL);
    }

    /*
    if ((step_iter>0) && ((step_iter%10)==0)) {
      printf("  [%i/%i] step_iter %i (d:%f)\n", (int)it, (int)m_num_verts, (int)step_iter, d); fflush(stdout);
      if (m_verbose > 2) { gp_state_print(); }
    }
    */

    if (fabs(d) < _eps) { break; }
  }

  // choose the cell and propagate choice
  //
  ret = chooseMaxBelief( &cell, &tile, &tile_idx, &belief );
  if (ret < 0) { return -1; }

  // (success) end condition, all cell positions have exactly
  // one tile in them.
  //
  if (ret==0 ) { return 0; }

  ret = tileIdxCollapse( cell, tile_idx );
  if (ret < 0) { return -2; }

  m_note_n[ m_grid_note_idx ] = 0;
  m_note_n[ 1-m_grid_note_idx ] = 0;

  cellFillAccessed(cell, m_grid_note_idx);
  unfillAccessed(m_grid_note_idx);

  ret = cellConstraintPropagate();
  if (ret < 0) { return -3; }

  // non-error but still processing
  //
  return 1;
}

int BeliefPropagation::single_realize (int64_t it) {
  int ret;
  int64_t cell=-1;
  int32_t tile=-1, tile_idx=-1;
  float belief=-1.0, d = -1.0;
  
  int64_t step_iter=0,
          max_step_iter = m_max_iteration;

  float _eps = m_eps_converge;
  
  // after we've propagated constraints, BUF_MU
  // needs to be renormalized
  //
  NormalizeMU();

  ComputeDiffMUField ();

  // iterate bp until converged
  //
  for (step_iter=0; step_iter<max_step_iter; step_iter++) {
    d = step();

    if (m_verbose > 0) {
      if ((step_iter>0) && ((step_iter%10)==0)) {
        printf("  [%i/%i] step_iter %i (d:%f)\n", (int)it, (int)m_num_verts, (int)step_iter, d); fflush(stdout);
        if (m_verbose > 2) { gp_state_print(); }
      }
    }
    if (fabs(d) < _eps) { break; }
  }

  // choose the cell and propagate choice
  //
  ret = chooseMaxBelief( &cell, &tile, &tile_idx, &belief );
  if (ret < 0) { return -1; }
  if (ret==0 ) { return 0; }      // was break

  ret = tileIdxCollapse( cell, tile_idx );
  if (ret < 0) { return -2; }

  m_note_n[ m_grid_note_idx ] = 0;
  m_note_n[ 1-m_grid_note_idx ] = 0;

  cellFillAccessed(cell, m_grid_note_idx);
  unfillAccessed(m_grid_note_idx);

  ret = cellConstraintPropagate();
  if (ret < 0) { return -3; }

  return 1;
}


void BeliefPropagation::gp_state_print() {
  int64_t anch_cell,
          anch_tile,
          anch_tile_idx,
          anch_tile_idx_n;
  Vector3DI pos;
  float max_belief = 0.0, b=0.0;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {
    cellUpdateBelief(anch_cell);

    pos = getVertexPos(anch_cell);

    cellUpdateBelief(anch_cell);

    anch_tile_idx_n = getVali( BUF_TILE_IDX_N, anch_cell );
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );

      b = getVal( BUF_BELIEF, anch_tile );
      if (anch_tile_idx==0) { max_belief = b; }
      if (b>max_belief)     { max_belief = b; }
    }
    printf("#gp: %i %i %i %f\n", (int)pos.x, (int)pos.y, (int)pos.z, (float)max_belief);
  }
}

int BeliefPropagation::realize() {
  int ret = 1;

  start();

  // process all verts
  //
  for (int64_t it=0; it<m_num_verts; it++) {

    ret = single_realize( it );
    if (ret==0) { break; }

    // check for errors
    //
    if ( ret < 0) {
      switch (ret) {
      case -1: printf ( "chooseMaxBelief error.\n" ); break;
      case -2: printf ( "tileIdxCollapse error.\n" ); break;
      case -3: printf ( "cellConstraintPropagate error.\n" ); break;
      };
    }
  }

  // ret == 0 - success and done
  // ret  < 0 - error
  //
  return ret;
}

float BeliefPropagation::step() {
  float max_diff = -1.0;

  // run main bp, store in BUF_MU_NXT
  //
  BeliefProp();

  // renormalize BUF_MU_NXT
  //
  NormalizeMU( BUF_MU_NXT );

  // calculate the difference between
  // BUF_MU_NXT and BUF_MU
  //
  max_diff = MaxDiffMU();

  // BUF_MU <- BUF_MU_NXT
  //
  UpdateMU();

  return max_diff;
}

//--------------------------------//
//  _          _                  //
// | |__   ___| |_ __   ___ _ __  //
// | '_ \ / _ \ | '_ \ / _ \ '__| //
// | | | |  __/ | |_) |  __/ |    //
// |_| |_|\___|_| .__/ \___|_|    //
//              |_|               //
//--------------------------------//

// Keep tile in the array of tile_id at cell position `pos` and discard
// the rest
//
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

// Discard tile entreis at cell positoin `pos`
//
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

// Iniefficiant scan to recover tile ID from tile name
//
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
  return tileName2ID(tile_name);
}

// print out state of BUF_NOTE, BUF_VISITED and BUF_CONSIDER
//
void BeliefPropagation::debugPrintC() {
  int i, n, fold = 20, m;

  printf("NOTE[%i][%i]", (int)m_grid_note_idx, (int)m_note_n[m_grid_note_idx]);
  for (m=0; m<2; m++) {
    n = m_note_n[m];
    for (i=0; i<n; i++) {
      if ((i%fold)==0) { printf("\n"); }
      printf(" %i", (int) getValNote( BUF_NOTE, m, i));
    }
    printf("\n");
  }

  n = m_num_verts;
  printf("VISITED[%i]\n", (int)m_num_verts);
  for (i=0; i<n; i++) {
    if ((i>0) && ((i%fold)==0)) { printf("\n"); }
    printf(" %i", (int)getVali( BUF_VISITED, i ));
  }
  printf("\n");

  n = m_num_verts;
  printf("CONSIDER[%i]\n", (int)m_num_verts);
  for (i=0; i<n; i++) {
    if ((i>0) && ((i%fold)==0)) { printf("\n"); }
    printf(" %i", (int)getVali( BUF_CONSIDER, i ));
  }
  printf("\n");

}

void BeliefPropagation::debugPrintS() {
  int i, j, dir_idx;

  for (dir_idx=0; dir_idx<6; dir_idx++) {
    printf("%s(%i)\n", m_dir_desc[dir_idx].c_str(), dir_idx);
    for (i=0; i<m_num_values; i++) {
      printf(" %s(%i)", m_tile_name[i].c_str(), i);
    }
    printf("\n");

    for (i=0; i<m_num_values; i++) {
      printf("%s(%i):", m_tile_name[i].c_str(), i);
      for (j=0; j<m_num_values; j++) {
        printf(" %0.1f", getValF( BUF_F, i, j, dir_idx));
      }
      printf("\n");
    }
    printf("---\n");
  }
}

void BeliefPropagation::debugPrint() {
  int i=0, n=3, m=7, jnbr=0, a=0;
  int a_idx=0, a_idx_n=0;
  uint64_t u=0;
  Vector3DI p;
  double v=0.0;
  float _vf = 0.0;

  int __a = 0;

  int64_t max_cell=-1;
  int32_t max_tile=-1, max_tile_idx=-1;
  float max_belief=-1.0;
  int count=-1;

  std::vector< std::string > _dp_desc;

  _dp_desc.push_back("+1:0:0");
  _dp_desc.push_back("-1:0:0");
  _dp_desc.push_back("0:+1:0");
  _dp_desc.push_back("0:-1:0");
  _dp_desc.push_back("0:0:+1");
  _dp_desc.push_back("0:0:-1");

  printf("m_res: (%i,%i,%i)\n", m_res.x, m_res.y, m_res.z);
  printf("m_bpres: (%i,%i,%i)\n", m_bpres.x, m_bpres.y, m_bpres.z);
  printf("m_num_verts: %i, m_num_values: %i\n", (int)m_num_verts, (int)m_num_values);
  printf("m_eps_converge: %f, m_eps_zero: %f\n",
      (float)m_eps_converge, (float)m_eps_zero);

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

    cellUpdateBelief(u);

    printf("[%i,%i,%i](%i):\n", (int)p.x, (int)p.y, (int)p.z, (int)u);
    for (a_idx=0; a_idx<a_idx_n; a_idx++) {
      a = getVali( BUF_TILE_IDX, (int)u, (int)a_idx );

      __a = tileName2ID( m_tile_name[a] );

      printf("  %s(%2i): ", m_tile_name[a].c_str(), a);
      //printf("  %s(%i,%i): ", m_tile_name[a].c_str(), a, __a);

      for (jnbr=0; jnbr<getNumNeighbors(u); jnbr++) {
        v = getVal( BUF_MU, jnbr, u, a );
        _vf = (float)v;
        printf(" [%s]", (char *)_dp_desc[jnbr].c_str());
        printf("(%i)", (int)jnbr);
        printf(":%f", (float)_vf);
        //printf(" [%s](%i):%f", (char *)_dp_desc[jnbr].c_str(), (int)jnbr, _vf);
      }

      printf(" {b:%f}", getVal( BUF_BELIEF, a ));

      printf("\n");
    }
    printf("\n");
  }

  count = chooseMaxBelief(&max_cell, &max_tile, &max_tile_idx, &max_belief);

  if (max_tile >= 0) {
    printf("max_belief: %i %s(%i) [%i] (%f) (%i)\n",
        (int)max_cell, m_tile_name[max_tile].c_str(), (int)max_tile,
        (int)max_tile_idx,
        (float)max_belief, (int)count);
  }
  else {
    printf("max_belief: %i -(%i) [%i] (%f) (%i)\n",
        (int)max_cell, (int)max_tile, (int)max_tile_idx, (float)max_belief, (int)count);
  }

}

//----------------------------------
//----------------------------------

//--------------------------------------------------------------//
//                      _             _       _                  // 
//   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_               //
//  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __|              //
// | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_               //
//  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|              //
//                                                              //
//                                          _   _               //
//  _ __  _ __ ___  _ __   __ _  __ _  __ _| |_(_) ___  _ __    //
// | '_ \| '__/ _ \| '_ \ / _` |/ _` |/ _` | __| |/ _ \| '_ \   //
// | |_) | | | (_) | |_) | (_| | (_| | (_| | |_| | (_) | | | |  //
// | .__/|_|  \___/| .__/ \__,_|\__, |\__,_|\__|_|\___/|_| |_|  //
// |_|             |_|          |___/                           //
//                                                              //
//--------------------------------------------------------------//

int BeliefPropagation::tileIdxCollapse(uint64_t pos, int32_t tile_idx) {
  int32_t idx, n, tile_val, tv;

  n = getVali( BUF_TILE_IDX_N, pos );
  if (tile_idx >= n) { return -1; }

  tile_val = getVali( BUF_TILE_IDX, pos, tile_idx );
  tv = getVali( BUF_TILE_IDX, pos, 0 );
  SetVali( BUF_TILE_IDX, pos, 0, tile_val );
  SetVali( BUF_TILE_IDX, pos, tile_idx, tv );
  SetVali( BUF_TILE_IDX_N, pos, 1 );

  return 0;
}


int BeliefPropagation::CullBoundary() {
  int i;
  int64_t x, y, z, vtx;
  Vector3DI vp;

  for (y=0; y<m_res.y; y++) {
    for (z=0; z<m_res.z; z++) {

      //printf("cb.yz %i,%i,%i\n", 0, (int)y, (int)z);

      vtx = getVertex(0, y, z);
      SetValNote( BUF_NOTE, m_grid_note_idx, m_note_n[m_grid_note_idx], vtx );
      m_note_n[m_grid_note_idx]++;

      if ((m_res.x-1) != 0) {

        //printf("cb.yz %i,%i,%i\n", (int)(m_res.x-1), (int)y, (int)z);

        vtx = getVertex(m_res.x-1, y, z);
        SetValNote( BUF_NOTE, m_grid_note_idx, m_note_n[m_grid_note_idx], vtx );
        m_note_n[m_grid_note_idx]++;
      }

    }
  }

  for (x=1; x<(m_res.x-1); x++) {
    for (z=0; z<m_res.z; z++) {

      //printf("cb.xz %i,%i,%i\n", (int)x, 0, (int)z);

      vtx = getVertex(x, 0, z);
      SetValNote( BUF_NOTE, m_grid_note_idx, m_note_n[m_grid_note_idx], vtx );
      m_note_n[m_grid_note_idx]++;

      if ((m_res.y-1) != 0) {

        //printf("cb.xz %i,%i,%i\n", (int)x, (int)(m_res.y-1), (int)z);

        vtx = getVertex(x, m_res.y-1, z);
        SetValNote( BUF_NOTE, m_grid_note_idx, m_note_n[m_grid_note_idx], vtx );
        m_note_n[m_grid_note_idx]++;
      }

    }
  }

  for (x=1; x<(m_res.x-1); x++) {
    for (y=1; y<(m_res.y-1); y++) {

      //printf("cb.xy %i,%i,%i\n", (int)x, (int)y, 0);

      vtx = getVertex(x, y, 0);
      SetValNote( BUF_NOTE, m_grid_note_idx, m_note_n[m_grid_note_idx], vtx );
      m_note_n[m_grid_note_idx]++;

      if ((m_res.z-1) != 0) {

        //printf("cb.xy %i,%i,%i\n", (int)x, (int)y, (int)(m_res.z-1));

        vtx = getVertex(x, y, m_res.z-1);
        SetValNote( BUF_NOTE, m_grid_note_idx, m_note_n[m_grid_note_idx], vtx );
        m_note_n[m_grid_note_idx]++;
      }

    }
  }

  cellConstraintPropagate();

  return 0;
}

int BeliefPropagation::_CullBoundary() {
  int64_t anch_cell;
  int64_t anch_in_idx, nei_cell;
  float fval,
        _eps = m_eps_zero;
        //_eps = (1.0/(1024.0*1024.0));

  int boundary_tile = 0;
  int anch_tile_idx, anch_tile, anch_tile_n, tval;

  int count = 0;

  for ( anch_cell=0; anch_cell < getNumVerts(); anch_cell++ ) {
    anch_tile_n = getVali( BUF_TILE_IDX_N, anch_cell );
    for (anch_tile_idx=0; anch_tile_idx<anch_tile_n; anch_tile_idx++) {

      anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );

      for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {
        nei_cell = getNeighbor(anch_cell, anch_in_idx);
        if (nei_cell != -1) { continue; }

        fval = getValF( BUF_F, anch_tile, boundary_tile, anch_in_idx);
        if (fval > _eps) { continue; }

        anch_tile_n--;
        tval = getVali( BUF_TILE_IDX, anch_cell, anch_tile_n );
        SetVali( BUF_TILE_IDX, anch_cell, anch_tile_n, anch_tile);
        SetVali( BUF_TILE_IDX, anch_cell, anch_tile_idx, tval);

        SetVali( BUF_TILE_IDX_N, anch_cell, anch_tile_n );

        count++;

        anch_tile_idx--;
        break;
      }

    }


  }

  return count;
}


// To speed up the 'collapse' propagation, two
// auxiliary data structures are stored, one a copy
// of the grid x dim that holds a 'note' about whether
// it's been accessed or not, and a list of verticies
// to process.
//
// This is an alternative to a 'map' by allowing set
// inclusion tests to be done by inspecting the 'note'
// so we know which vertices are already present in
// the 'consider' array.
// The 'consider' array has a list of vertices (cell
// positions) that need to be inspected to determine
// if any tiles should be removed.
//
void BeliefPropagation::cellFillAccessed(uint64_t vtx, int32_t note_idx) {
  int64_t i, nei_vtx;
  int32_t note;

  for (i=0; i<getNumNeighbors(vtx); i++) {
    nei_vtx  = getNeighbor(vtx, i);
    if (nei_vtx<0) { continue; }
    if (getVali( BUF_VISITED, nei_vtx ) != 0) { continue; }

    SetValNote( BUF_NOTE, note_idx, m_note_n[note_idx], nei_vtx );
    SetVali( BUF_VISITED, nei_vtx , 1 );
    m_note_n[note_idx]++;

  }

}

int BeliefPropagation::getTilesAtVertex ( int64_t vtx ) {

  int a = 0;
  int n_a = getVali( BUF_TILE_IDX_N, vtx );

  ZeroBPVec ( BUF_BELIEF );   // clear belief vec

  float p = 1.0 / n_a;  
  
  for (int a_idx=0; a_idx<n_a; a_idx++) {
    a = getVali( BUF_TILE_IDX, vtx, a_idx );
    SetVal ( BUF_BELIEF, a, p );
  }

  return n_a;
}

/*void BeliefPropagation::UpdateAllVertsFromNotes ()
{
  int64_t i, vtx;

  for (i=0; i < m_note_n[ m_grid_note_idx ]; i++) {
    vtx = getValNote( BUF_NOTE, m_grid_note_idx, i );
    
    //SetVal( BUF_MU, 0, vtx, tile_val, 1.0 );    
  }
}*/

// unwind/remove all 'filled' cells
//
void BeliefPropagation::unfillAccessed(int32_t note_idx) {
  int64_t i, vtx;

  for (i=0; i<m_note_n[note_idx]; i++) {
    vtx = getValNote( BUF_NOTE, note_idx, i );
    SetVali( BUF_VISITED, vtx, 0 );
  }
}

int BeliefPropagation::sanityAccessed() {
  int64_t i;

  for (i=0; i<m_num_verts; i++) {
    if (getVali( BUF_VISITED, i)!=0) { return -1; }
  }
  return 0;
}


int BeliefPropagation::removeTileIdx(int64_t anch_cell, int32_t anch_tile_idx) {
  int tile, anch_tile_n, anch_tile, tval;

  anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );

  anch_tile_n = getVali( BUF_TILE_IDX_N, anch_cell );
  anch_tile_n--;
  if (anch_tile_n==0) { return -1; }

  tval = getVali( BUF_TILE_IDX, anch_cell, anch_tile_n );
  SetVali( BUF_TILE_IDX, anch_cell, anch_tile_n, anch_tile);
  SetVali( BUF_TILE_IDX, anch_cell, anch_tile_idx, tval);

  SetVali( BUF_TILE_IDX_N, anch_cell, anch_tile_n );

  return 0;
}

// This propagates constraints based on a 'wave front' approach,
// removing tiles from the BUF_TILE_IDX as necessary.
// This fills two structures, the BUF_NOTE and the BUFF_VISITED in
// addition to altering the BUF_TILE_IDX and BUF_TILE_IDX_N.
//
// For every vertex in the BUF_NOTE array, test to see if any
// tile needs to be removed.
// If so, add it to the other plane of the BUF_NOTE for the next
// round of processing.
// To make sure duplicate entries aren't added, the tile entry
// in BUF_VISITED is set and not added to BUF_NOTE if the entry is already set.
//
// Once the round of constraint propagation has been run, the current
// plane of BUF_NOTE is reset (m_note_n[plane]=0) and the BUF_VISITED
// vector is unwound.
// A note about unwinding the BUF_VISITED, this is done by walking
// the current BUF_NOTE plane as this holds all verticies that were
// touched, alleviating the need to touch every entry of BUF_VISITED.
//
// There are two major tests to see if a tile can be removed from the
// TILE_IDX list of choices:
//
// * if it connects outward to an out-of-bound area, cull it
// * if it does not have a valid connection to a neighboring cell, cull it
//
// In pseudo-code:
//
// while (current plane of BUF_NOTE non-empty) {
//   for (anch_cell in current plane of BUF_NOTE) {
//     for (anch_tile in anch_cell position) {
//       if (anch_tile @ anch_cell has connection that reachesout of bounds) {
//         cull it
//         if (neighbor vertex not in BUF_VISITED) {
//           add neighbor positions of anch_cell to next plane of BUF_NOTE
//           add vertex to BUF_VISITED
//         }
//       }
//       if (anch_tile @ anch_cell does not have at least one valid connection to neighboring tiles) {
//         cull it
//         if (neighbor vertex not in BUF_VISITED) {
//           add neighbor positions of anch_cell to next plane of BUF_NOTE
//           add vertex to BUF_VISITED
//         }
//       }
//     }
//   }
//   unwind visited by walking current plane of BUF_NOTE and setting entries back to 0
//   set current BUF_NOTE plane length to 0
//   update BUF_NOTE current plane to the next plane
// }
//
//
int BeliefPropagation::cellConstraintPropagate() {
  int still_culling=1, i;

  int64_t note_idx, anch_cell, nei_cell;
  int64_t nei_n_tile, nei_a_idx, nei_a_val;

  int64_t anch_n_tile, anch_b_idx, anch_b_val;
  int anch_has_valid_conn = 0;

  int boundary_tile = 0, tile_valid = 0;
  int gn_idx = 0;

  float _eps = m_eps_zero;

  while (still_culling) {

    for (note_idx=0; note_idx<m_note_n[m_grid_note_idx]; note_idx++) {
      anch_cell = getValNote( BUF_NOTE, m_grid_note_idx, note_idx );

      anch_n_tile = getVali( BUF_TILE_IDX_N, anch_cell );
      for (anch_b_idx=0; anch_b_idx < anch_n_tile; anch_b_idx++) {

        // Test if anchor tile has connection that falls out of bounds.
        // if so, remove tile from BUF_TILE_IDX and add unvisited
        // neighbors to BUF_NOTE and BUF_CONSIDER for later processing.
        //
        tile_valid = 1;
        anch_b_val = getVali( BUF_TILE_IDX, anch_cell, anch_b_idx );

        for (i=0; i<getNumNeighbors(anch_cell); i++) {
          nei_cell = getNeighbor(anch_cell, i);
          if ((nei_cell<0) &&
              (getValF( BUF_F, anch_b_val, boundary_tile, i ) < _eps)) {

            if (anch_n_tile==1) {

              if (m_verbose > 1) {
                printf("# BeliefPropagation::cellConstraintPropagate: ERROR, cell %i slated to rmove last remaining tile (tile %s(%i) conflicts with out of bounds neighbor %s(%i) dir %s(%d))\n",
                    (int)anch_cell,
                    m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                    m_tile_name[boundary_tile].c_str(), (int)boundary_tile,
                    m_dir_desc[i].c_str(), (int)i);
              }

              return -1;
            }

            tile_valid = 0;

            removeTileIdx(anch_cell, anch_b_idx);
            cellFillAccessed(anch_cell, 1-m_grid_note_idx);

            anch_b_idx--;
            anch_n_tile--;

            break;
          }
        }

        if (!tile_valid) { continue; }

        // Test for at least one valid neighbor from the anchor point.
        // That is, for each anchor cell and tile value, make sure
        // there is at least one "admissible" tile in the appropriate
        // direction by checking the BUF_F table.
        //
        for (i=0; i<getNumNeighbors(anch_cell); i++) {
          nei_cell = getNeighbor(anch_cell, i);

          if (nei_cell<0) { continue; }

          anch_has_valid_conn = 0;

          nei_n_tile = getVali( BUF_TILE_IDX_N, nei_cell );
          for (nei_a_idx=0; nei_a_idx < nei_n_tile; nei_a_idx++) {
            nei_a_val = getVali( BUF_TILE_IDX, nei_cell, nei_a_idx );

            if (getValF( BUF_F, anch_b_val, nei_a_val, i ) > _eps) {
              anch_has_valid_conn = 1;
              break;
            }
          }

          if (!anch_has_valid_conn) {
            if (anch_n_tile==1) {

              if (m_verbose > 1) {
                printf("# BeliefPropagation::cellConstraintPropagate: ERROR, cell %i slated to rmove last remaining tile (tile %s(%i) conflicts with neighbor cell %i, tile %s(%i) dir %s(%d))\n",
                    (int)anch_cell,
                    m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                    (int)nei_cell,
                    m_tile_name[nei_a_val].c_str(), (int)nei_a_val,
                    m_dir_desc[i].c_str(), (int)i);
              }


              return -1;
            }

            tile_valid = 0;

            removeTileIdx(anch_cell, anch_b_idx);
            cellFillAccessed(anch_cell, 1-m_grid_note_idx);
            anch_b_idx--;
            anch_n_tile--;

            break;
          }

        }

        if (!tile_valid) { continue; }
      }
    }

    unfillAccessed(1-m_grid_note_idx);

    if (m_note_n[m_grid_note_idx] == 0) { still_culling = 0; }

    m_note_n[m_grid_note_idx] = 0;
    m_grid_note_idx = 1-m_grid_note_idx;
  }

  return 0;
}

