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
#include <time.h>

#include <vector>
#include <string>

#include "belief_propagation.h"


void BeliefPropagation::setConverge ( bp_opt_t* oparg, float c ) {

  oparg->eps_converge = c;

  oparg->eps_converge_beg = c;
  oparg->eps_converge_end = c;

}

int BeliefPropagation::default_opts () {

  op.seed = 17;

  st.enabled = 1;
  st.post = 0;
  st.upper_step = 0;
  st.avg_step = 0;
  st.second_moment_step = 0;
  st.eps_curr = 0.0;
  st.max_dmu = 0.0;
  st.ave_mu = 0.0;
  st.ave_dmu = 0.0;
  st.num_culled = 0;
  st.num_collapsed = 0;
  st.num_chosen = 0;

  st.iter_resolved = 0;
  st.total_resolved=0;
  st.elapsed_time = 0.0;
  st.constraints = 0;

  st.instr = 0;

  st.occupancy_mean = 0.0;
  st.occupancy_mode = 0.0;
  st.occupancy_second_moment = 0.0;

  st.cluster_mean = 0.0;
  st.cluster_mode = 0.0;
  st.cluster_second_moment = 0.0;

  op.eps_converge = (1.0/(1024.0));

  setConverge ( &op, op.eps_converge );

  //op.eps_zero = (1.0/(1024.0*1024.0*1024.0*1024.0));
  op.eps_zero = (1.0/(1024.0*1024.0));

  op.cur_run = 0;
  op.max_run = 0;



   // iterations
   //
  op.cur_iter = 0;

  // will be set to # vertices
  //
  op.max_iter = 0;


  // steps
  //
  op.cur_step = 0;
  op.max_step = 1024;

  op.step_cb = 1;
  op.state_info_d = -1;
  op.state_info_iter = 0;

  op.step_rate = 0.98;

  op.use_svd = 0;
  op.use_checkerboard = 0;

  op.index_heap_size = 0;

  op.viz_opt = VIZ_NONE;

  op.alg_cell_opt = ALG_CELL_MIN_ENTROPY;
  op.alg_tile_opt = ALG_TILE_MAX_BELIEF;
  op.alg_run_opt = ALG_RUN_VANILLA;
  op.alg_accel = ALG_ACCEL_NONE;

  op.use_cuda = false;

  op.tileset_stride_x = 0;
  op.tileset_stride_y = 0;
  op.tileset_margin = 0;
  op.tileset_spacing = 0;
  op.tileset_width = 0;
  op.tileset_height = 0;
  op.tiled_reverse_y = 0;

  return 0;
}


// AllocBuf -- new allocation function
//
// supports multi-dimensional data of any type
// total elements = cntx * cnty * cntz
// cntx will be sequential in memory
//
void BeliefPropagation::AllocBuf (int id, char dt, uint64_t resx, uint64_t resy, uint64_t resz ) {

  char buf_dt;
  uint64_t type_sz = 0;

  // get type size
  switch (dt) {
  case 'i': type_sz = sizeof(int32_t);  buf_dt = DT_UINT;   break;
  case 'l': type_sz = sizeof(int64_t);  buf_dt = DT_UINT64; break;
  case 'f': type_sz = sizeof(float);    buf_dt = DT_FLOAT;  break;
  default:
    printf ("ERROR: Type not available.\n" );
    exit(-4);
  };
  int flags = (op.use_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU);

  // resize buffer
  //
  uint64_t total_cnt = resx * resy * resz;
  m_buf[id].Resize( type_sz, total_cnt, 0x0, flags );

  // set usage by dimension
  //
  m_buf[id].SetUsage ( buf_dt, flags, resx, resy, resz );

  ZeroBuf ( id );
}

void BeliefPropagation::ZeroBuf ( int id ) {

  m_buf[id].FillBuffer ( 0 );
}


//---

int64_t BeliefPropagation::getVertex(int x, int y, int z) {

  return int64_t(z*m_bpres.y + y)*m_bpres.x + x;
}

// domain index to 3D pos
//
Vector3DI BeliefPropagation::getVertexPos(int64_t j) {

  Vector3DI p;

  p.z = j / (m_bpres.x*m_bpres.y);
  j -= p.z * (m_bpres.x*m_bpres.y);

  p.y = j / m_bpres.x;
  j -= p.y * m_bpres.x;

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




// get 3D grid neighbor
//
int64_t BeliefPropagation::getNeighbor( uint64_t j, Vector3DI jp, int nbr ) {

  //-- using lookup
  /* int32_t ndx = (nbr<2) ? jp.x : (nbr<4) ? jp.y : jp.z;
  int64_t n = nbr_lookup[nbr][ ndx ];    
  n = (n==NOUT) ? -1 : j + n;
  return n; */

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


void BeliefPropagation::ConstructStaticBufs () {

  //-- Construct F
  //
  int B;
  B = m_num_values;
  AllocBuf ( BUF_F, 'f', B, B, 6 );

  //-- Construct G
  //
  AllocBuf ( BUF_G, 'f', m_num_values );
  float weight = 1.0 / m_num_values;
  for (int a=0; a < m_num_values; a++ ) {
    SetValF ( BUF_G, a, weight );
  }

  //-- Construct H
  //
  AllocBuf ( BUF_H, 'f', m_num_values );

}

void BeliefPropagation::ConstructDynamicBufs () {

  //-- Construct MU
  //
  AllocBuf ( BUF_MU,     'f', 6, m_num_values, m_num_verts );
  AllocBuf ( BUF_MU_NXT, 'f', 6, m_num_values, m_num_verts );

  //-- Construct TILE_IDX
  //
  AllocBuf ( BUF_TILE_IDX,   'i', m_num_values, m_num_verts );
  AllocBuf ( BUF_TILE_IDX_N, 'i', m_num_verts );

  // initialize to all tiles per vertex
  //
  for (int i=0; i<m_num_verts; i++) {

    // vtx i has num_vals possible number of tiles
    //
    SetValI ( BUF_TILE_IDX_N, (m_num_values), i );

    for (int b=0; b<m_num_values; b++) {

      // initially put all tile ids
      // in order at vtx i
      //
      SetValI ( BUF_TILE_IDX, b, b, i );
    }
  }

  //-- Construct visited
  //
  AllocBuf ( BUF_VISITED, 'l', m_num_verts );

  //-- Construct note
  //
  AllocBuf ( BUF_NOTE,    'l', m_num_verts, 2 );
  m_note_n[0] = 0;
  m_note_n[1] = 0;
  m_note_plane = 0;

  //-- Construct Residual BP buffers
  //
  AllocBuf ( BUF_RESIDUE_HEAP,          'f', 6 * m_num_verts * m_num_values );
  AllocBuf ( BUF_RESIDUE_HEAP_CELL_BP,  'l', 6 * m_num_verts * m_num_values );
  AllocBuf ( BUF_RESIDUE_CELL_HEAP,     'l', 6 * m_num_verts * m_num_values );

  RandomizeMU ();

}

void BeliefPropagation::ConstructTempBufs () {

  //-- Construct belief
  //
  AllocBuf ( BUF_BELIEF, 'f', m_num_values );

  //-- Construct viz
  //
  AllocBuf ( BUF_VIZ, 'f', m_num_verts );

  //-- Construct TILE buf
  //
  AllocBuf ( BUF_TILES, 'i', m_num_verts );
  
  //-- Construct B buf
  //
  AllocBuf ( BUF_B, 'f', m_num_verts );

  //-- Construct C (constraint count) buf
  //
  AllocBuf ( BUF_C, 'i', m_num_verts );

}

void BeliefPropagation::ConstructSVDBufs () {

  int B = m_num_values;

  AllocBuf ( BUF_SVD_U,   'f', B, B, 6 );
  AllocBuf ( BUF_SVD_Vt,  'f', B, B, 6 );
  AllocBuf ( BUF_SVD_VEC, 'f', B );
}


void BeliefPropagation::RandomizeMU () {
  int i;
  Vector3DI jp;

  float *mu = (float*) m_buf[BUF_MU].getData();
  uint64_t cnt = 6 * m_num_verts * m_num_values;
  memset ( mu, 0, cnt * sizeof(float) );

  for (int j=0; j < m_num_verts; j++) {
    jp = getVertexPos(j);

    for (int jnbr=0; jnbr < getNumNeighbors(j); jnbr++) {
      i = getNeighbor(j, jp, jnbr);

      for (int a=0; a < m_num_values; a++) {

        float w = m_rand.randF();

        // randomize MU
        //
        SetValF( BUF_MU, (w), jnbr, a, j );
      }
    }
  }
}

void BeliefPropagation::ComputeDiffMUField () {

  int i, n_a, a;
  float v0, v1, d, max_diff;
  Vector3DI jp;

  //int vtx = 528;

  for (int j=0; j < m_num_verts; j++) {
    jp = getVertexPos(j);

    max_diff = 0;

    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, jp, in);
      n_a = getValI( BUF_TILE_IDX_N, j );

      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getValI( BUF_TILE_IDX, a_idx, j );
        v0 = getValF( BUF_MU, in, a, j );
        v1 = getValF( BUF_MU_NXT, in, a, j );

        d = fabs(v0-v1);
        if (d > max_diff) { max_diff = d; }
      }
    }

    SetValF ( BUF_VIZ, max_diff, j );
  }

 }

int BeliefPropagation::getMaxBeliefTile ( uint64_t j ) {

  int tile_idx_n, tile_idx, tile_val;
  float b, maxb;
  int maxtv;

  // list of tile beliefs for single vertex
  //
  getVertexBelief ( j );

  // walk the tile vals to get max belief
  //
  b = 0;
  maxb = 0;
  maxtv = 0;
  tile_idx_n= getValI( BUF_TILE_IDX_N, j );

  if ( tile_idx_n > 1 ) {

    for (tile_idx=0; tile_idx<tile_idx_n; tile_idx++) {

      tile_val = getValI( BUF_TILE_IDX, tile_idx, j );

      b = getValF (BUF_BELIEF, tile_val );

      if ( b > maxb) {
        maxb = b;
        maxtv = tile_val;
      }
    }
  }

  return maxtv;
}

// ComputeBeliefField
// - fills BUF_TILES with maxbelief tiles (always)
// - fills BUF_VIZ with maxbelief values (if VIZ_BELIEF)
//
void BeliefPropagation::ComputeBeliefField () {

  int tile_idx_n, tile_idx, tile_val;

  float global_maxb;
  float b, maxb, sum;
  int maxt;

  global_maxb = 0;

  for (int j=0; j < m_num_verts; j++) {

    // list of tile beliefs for single vertex
    //
    getVertexBelief ( j );

    // walk the tile vals to get max belief
    //
    b = 0;
    maxb = 0;
    maxt = 0;

    tile_idx_n= getValI( BUF_TILE_IDX_N, j );

    if ( tile_idx_n == 1 ) {

      // only one tile
      //
      maxb = 0.0;
      maxt = getValI( BUF_TILE_IDX, 0, j );
      sum = 0;

    } else {

      // search for max belief tile
      //
      sum = 0;
      for (tile_idx=0; tile_idx < tile_idx_n; tile_idx++) {
        tile_val = getValI( BUF_TILE_IDX, tile_idx, j );

        b = getValF (BUF_BELIEF, tile_val );

        sum += (b < 0.0001) ? 0 : b * log(b);

        if ( b > maxb) {
          maxb = b;
          maxt = tile_val;
        }
      }

    }

    if ( maxb > global_maxb ) global_maxb = maxb;

    // set max belief for this vertex
    //
    SetValF( BUF_B, maxb, j );

    SetValI( BUF_TILES, maxt , j );
  }

  // global max belief
  st.max_belief = global_maxb;

}


//---

float BeliefPropagation::MaxDiffMU ()  {

  int i, n_a, a;
  float v0, v1, d;
  Vector3DI jp;

  float global_maxdiff = 0;
  float vert_maxdiff;

  int mu_cnt = 0;
  st.ave_mu = 0;
  st.ave_dmu = 0;

  for (int j=0; j < m_num_verts; j++) {
    jp = getVertexPos(j);

    vert_maxdiff = 0;

    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, jp, in);
      n_a = getValI ( BUF_TILE_IDX_N, j );

      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getValI ( BUF_TILE_IDX, a_idx, j );
        v0 = getValF ( BUF_MU, in, a, j );
        v1 = getValF ( BUF_MU_NXT, in, a, j );

        d = fabs(v0-v1);
        if (d > vert_maxdiff) { vert_maxdiff = d; }

        if (st.enabled) {
            st.ave_mu += v1;
            st.ave_dmu += d;
            mu_cnt++;
        } 
      }
    }

    if ( op.alg_accel==ALG_ACCEL_WAVE) {
      // store the dmu in BUF_MU_NXT temporarily 
      //  (will be overwriten on next bp step)
      SetValF ( BUF_MU_NXT, vert_maxdiff, 0, 0, j );
    }

    if ( vert_maxdiff > global_maxdiff ) {global_maxdiff = vert_maxdiff;}
    //printf ( "max overall: %f\n", max_overall );
  }

  st.ave_mu /= mu_cnt;
  st.ave_dmu /= mu_cnt;

  return global_maxdiff;
}

void BeliefPropagation::InitializeDMU (int buf_id) {

     for (int j=0; j < m_num_verts; j++) {

         SetValF ( BUF_MU_NXT, 1.0, 0, 0, j );

     }
}


float BeliefPropagation::MaxDiffMUCellTile (float *max_diff, int64_t *max_cell, int64_t *max_tile_idx, int64_t *max_dir_idx) {
  int i, n_a, a;
  float v0,v1, d, _max_diff=-1.0;
  int64_t _max_cell=-1,
          _max_tile_idx=-1,
          _max_dir_idx=-1;

  for (int j=0; j < m_num_verts; j++) {
    for (int in=0; in < getNumNeighbors(j); in++) {
      i = getNeighbor(j, in);
      n_a = getValI ( BUF_TILE_IDX_N, j );
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getValI ( BUF_TILE_IDX, a_idx, j );
        v0 = getValF ( BUF_MU, in, a, j );
        v1 = getValF ( BUF_MU_NXT, in, a, j );

        d = fabs(v0-v1);
        if (_max_diff < d) {
          _max_diff = d;
          _max_cell = j;
          _max_tile_idx = a_idx;
          _max_dir_idx = in;
        }
      }
    }
  }

  if (max_diff) { *max_diff = _max_diff; }
  if (max_cell) { *max_cell = _max_cell; }
  if (max_tile_idx) { *max_tile_idx = _max_tile_idx; }
  if (max_dir_idx) { *max_dir_idx = _max_dir_idx; }

  return _max_diff;
}

void BeliefPropagation::NormalizeMU () { NormalizeMU( BUF_MU ); }

void BeliefPropagation::NormalizeMU (int id) {
  int i=0, n_a=0, a=0;
  float v=0, sum=0;
  Vector3DI jp;

  for (int j=0; j < m_num_verts; j++) {
    jp = getVertexPos(j);

    for (int in=0; in < getNumNeighbors(j); in++) {

      i = getNeighbor(j, jp, in);

      // write 1 at boundaries
      if (i==-1) {
        n_a = getValI ( BUF_TILE_IDX_N, j );
        for (int a_idx=0; a_idx<n_a; a_idx++) {
          a = getValI ( BUF_TILE_IDX, a_idx, j );
          SetValF ( id, 1.0, in, a, j );
        }
      }

      // sum all MU values
      sum = 0;

      n_a = getValI ( BUF_TILE_IDX_N, j );
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getValI ( BUF_TILE_IDX, a_idx, j );
        sum += getValF( id, in, a, j );
      }

      // normalize each MU
      if ( sum > 0 ) {
        n_a = getValI ( BUF_TILE_IDX_N, j );
        for (int a_idx=0; a_idx<n_a; a_idx++) {
          a = getValI ( BUF_TILE_IDX, a_idx, j );

          v = getValF( id, in, a, j );
          SetValF( id, (v / sum), in, a, j );
        }
      }

    }
  }
}

void BeliefPropagation::NormalizeMU_cell_residue (int buf_id, int64_t cell) {
  int i=0, n_a=0, a=0;
  float v=0, sum=0,
        mu_cur_val,
        mu_nxt_val;
  Vector3DI cell_p;

  cell_p = getVertexPos(cell);
  for (int in=0; in < getNumNeighbors(cell); in++) {
    i = getNeighbor(cell, cell_p, in);
    sum = 0;

    if (i==-1) {
      n_a = getValI ( BUF_TILE_IDX_N, cell );
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getValI ( BUF_TILE_IDX, a_idx, cell );
        SetValF( buf_id, 1.0, in, a, cell );

        mu_cur_val = getValF( BUF_MU,      in, a, cell );
        mu_nxt_val = getValF( BUF_MU_NXT,  in, a, cell );
        indexHeap_update_mu_pos( in, cell, a, fabs(mu_cur_val - mu_nxt_val) );

      }
    }

    n_a = getValI( BUF_TILE_IDX_N, cell );
    for (int a_idx=0; a_idx<n_a; a_idx++) {
      a = getValI( BUF_TILE_IDX, a_idx, cell );
      sum += getValF ( buf_id, in, a, cell );
    }

    if ( sum > 0 ) {
      n_a = getValI( BUF_TILE_IDX_N, cell );
      for (int a_idx=0; a_idx<n_a; a_idx++) {
        a = getValI( BUF_TILE_IDX, a_idx, cell );

        v = getValF( buf_id, in, a, cell );
        SetValF( buf_id, (v / sum), in, a, cell );

        mu_cur_val = getValF( BUF_MU,      in, a, cell );
        mu_nxt_val = getValF( BUF_MU_NXT,  in, a, cell );
        indexHeap_update_mu_pos( in, cell, a, fabs(mu_cur_val - mu_nxt_val) );

      }
    }

  }

}

void BeliefPropagation::SetVis (int vopt) {

  op.viz_opt = vopt;

  std::string msg;
  switch ( vopt ) {
  case VIZ_MU:            msg = "VIZ_MU"; break;
  case VIZ_DMU:           msg = "VIZ_DMU"; break;
  case VIZ_BELIEF:        msg = "VIZ_BELIEF"; break;
  case VIZ_CONSTRAINT:    msg = "VIZ_CONSTRAINT"; break;
  case VIZ_TILECOUNT:     msg = "VIZ_TILECOUNT"; break;
  case VIZ_ENTROPY:       msg = "VIZ_ENTROPY"; break;
  case VIZ_CHANGE:        msg = "VIZ_CHANGE"; break;
  case VIZ_RESPICK:       msg = "VIZ_RESPICK"; break;
  };
  printf ( "started: %s\n", msg.c_str() );
}

// WFC  cnt = getTilesAtVertex( j );
// DMU  dmu =  scalar * std::max(0.0f, std::min(1.0f, pow ( src.getVal ( BUF_DMU, j ), 0.1f ) ));
//
Vector4DF BeliefPropagation::getVisSample ( int64_t v ) {

  float f, c, b;
  Vector4DF s;

  float vexp = 0.3;
  float vscale = 0.1;

  float vmax = op.eps_converge * 10.0;

  switch (op.viz_opt) {
  case VIZ_MU:
    f = getValF ( BUF_VIZ, v );
    s = Vector4DF(f,f,f,f);
    break;
  case VIZ_DMU:

    // dmu written into viz by ComputeDiffMU
    //
    f = getValF ( BUF_VIZ, v );

    c = 0.1 + std::max(0.0f, std::min(1.0f, pow ( f * vscale / vmax, vexp ) ));
    
    if ( f <= op.eps_converge ) s = Vector4DF(0,c,0,c);
    else                        s = Vector4DF(c,c,c,c);

    break;

  case VIZ_BELIEF:    
    // get maxbelief value
    f = getValF ( BUF_B, v );
    f = vscale * std::max(0.0f, std::min(1.0f, pow ( f, vexp ) ) );
    s = Vector4DF(f,f,f,f);
    break;
  
  case VIZ_TILECOUNT:
    // 1. compute maxbelief and outputs json
    // 2. visualizes 1/TILE_NDX_N as alpha (eg. 1=opaque=fully resolved)
    // 3. visualizes green-red as maxbelief # constraints/cell (eg. 0=green, 6=all faces of cell)

    f = getValF ( BUF_VIZ, v );        // 1/tilecount

    c = getValI ( BUF_C, v ) / 6.0f;   // constraints
    b = getValF ( BUF_B, v );          // belief

    s = Vector4DF( c, 1-c, 0, f*f );

    float beps = 0.1;

    if ( f==1 ) s = Vector4DF(1,1,1,1);              // white = resolved to 1 tile
    if ( b >= st.max_belief - beps ) s = Vector4DF(1,0,1,1);  // purple = current max belief vertex

    break;
  }

  return s;
}



float BeliefPropagation::BeliefProp_cell_residue (int64_t anch_cell) {
  int64_t nei_cell=0;
  int d;

  int anch_tile_idx=0, anch_tile_idx_n=0;
  int nei_tile_idx=0, nei_tile_idx_n=0;
  int anch_tile, nei_tile=0;
  int64_t _neinei_cell=0;

  int nei_in_idx=0, anch_in_idx=0;
  int nei_to_anch_dir_idx=0;

  float H_ij_a=0;
  float u_nxt_b=0, u_prev_b=0;
  float du=0;

  float rate = 1.0,
        max_diff=-1.0;

  float mu_cur_val=0,
        mu_nxt_val=0;


  rate = op.step_rate;

  anch_tile_idx_n = getValI ( BUF_TILE_IDX_N, anch_cell );

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
        anch_tile = getValI( BUF_TILE_IDX, 0, anch_cell );
        SetValF ( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell  );

        mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell );
        mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell );

        indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );

      }
      continue;
    }
    if (nei_cell == -1) {

      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        SetValF ( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell  );

        mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell );
        mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell );

        indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );

      }

      continue;
    }

    // compute message from `nei` to `anch` for each a..
    // we're skipping some tiles, so zero out H
    //
    for (d=0; d<m_num_values; d++) { SetValF (BUF_H, 0, d); }

    nei_tile_idx_n = getValI ( BUF_TILE_IDX_N, nei_cell );

    // pathological condition
    // * neighbor cell has 0 values (error) or only 1
    //
    if (nei_tile_idx_n <= 1) {
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        SetValF ( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell  );

        mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell );
        mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell );

        indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );

      }
      continue;
    }

    int numbrs = getNumNeighbors(nei_cell);
    int nei_in_ignore = getOppositeDir( anch_in_idx );

    for (nei_tile_idx=0; nei_tile_idx < nei_tile_idx_n; nei_tile_idx++) {

      nei_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      // first compute Hij_t
      // initialize Hij(a) = gi(a)
      //
      H_ij_a = getValF (BUF_G, nei_tile);

      float *currMu = (float*) getPtr (BUF_MU, 0, nei_tile, nei_cell );

      for (nei_in_idx=0; nei_in_idx < getNumNeighbors(nei_cell); nei_in_idx++ ) {
        _neinei_cell = getNeighbor(nei_cell, nei_in_idx);
        if (nei_in_idx != nei_in_ignore) { H_ij_a *= *currMu; }

        currMu++;
      }

      SetValF (BUF_H, H_ij_a, nei_tile );

    }

    // now compute mu_ij_t+1 = Fij * hij
    // b = rows in f{ij}(a,b), also elements of mu(b)/
    //
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      u_nxt_b = 0.0;
      nei_to_anch_dir_idx = m_dir_inv[anch_in_idx];

      float* currH = (float*) getPtr (BUF_H);
      float* currF = (float*) getPtr (BUF_F, 0, anch_tile, nei_to_anch_dir_idx);    // <B, B, 6>

      for (d=0; d < m_num_values; d++) {
        u_nxt_b += (*currF) * (*currH);

        currF++;
        currH++;
      }

      u_prev_b = getValF (BUF_MU, anch_in_idx, anch_tile, anch_cell );

      du = u_nxt_b - u_prev_b;

      if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }

      SetValF (BUF_MU_NXT, (u_prev_b + du*rate), anch_in_idx, anch_tile, anch_cell );

      mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell );
      mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell );

      indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );

    }
  }

  return max_diff;
}

void BeliefPropagation::TransferBoundaryMU (int src_id, int dst_id) {
  Vector3DI jp;
  int64_t j;
  float v;
  // 0=x+
  // 1=x-
  // 2=y+
  // 3=y-
  // 4=z+
  // 5=z-

  // Set MU values on all boundary planes
  // to 1.0 in the direction of out-of-bounds

  for (int tile=0; tile < m_num_values; tile++) {

    // X plane
    for (jp.z=0; jp.z < m_bpres.z; jp.z++) {
      for (jp.y=0; jp.y < m_bpres.y; jp.y++) {
        jp.x = 0; j = getVertex(jp.x, jp.y, jp.z);

        v = getValF ( src_id, 1, tile, j );
        SetValF ( dst_id, v, 1, tile, j );

        jp.x = m_bpres.x-1; j = getVertex(jp.x, jp.y, jp.z);

        v = getValF ( src_id, 0, tile, j );
        SetValF ( dst_id, v, 0, tile, j );
      }
    }

    // Y plane
    for (jp.z=0; jp.z < m_bpres.z; jp.z++) {
      for (jp.x=0; jp.x < m_bpres.x; jp.x++) {
        jp.y = 0; j = getVertex(jp.x, jp.y, jp.z);

        v = getValF ( src_id, 3, tile, j );
        SetValF ( dst_id, v, 3, tile, j );

        jp.y = m_bpres.y-1; j = getVertex(jp.x, jp.y, jp.z);

        v = getValF ( src_id, 2, tile, j );
        SetValF ( dst_id, v, 2, tile, j );
      }
    }


    // Z plane
    for (jp.y=0; jp.y < m_bpres.y; jp.y++) {
      for (jp.x=0; jp.x < m_bpres.x; jp.x++) {
        jp.z = 0; j = getVertex(jp.x, jp.y, jp.z);

        v = getValF ( src_id, 5, tile, j );
        SetValF ( dst_id, v, 5, tile, j );

        jp.z = m_bpres.z-1; j = getVertex(jp.x, jp.y, jp.z);

        v = getValF ( src_id, 4, tile, j );
        SetValF ( dst_id, v, 4, tile, j );
      }
    }
  }
}

void BeliefPropagation::WriteBoundaryMUbuf(int buf_id=BUF_MU) {
  Vector3DI jp;
  int64_t j;

  // 0=x+
  // 1=x-
  // 2=y+
  // 3=y-
  // 4=z+
  // 5=z-

  // Set MU values on all boundary planes
  // to 1.0 in the direction of out-of-bounds

  for (int tile=0; tile < m_num_values; tile++) {

    // X plane
    for (jp.z=0; jp.z < m_bpres.z; jp.z++) {
      for (jp.y=0; jp.y < m_bpres.y; jp.y++) {
        jp.x = 0;
        j = getVertex(jp.x, jp.y, jp.z);
        SetValF ( buf_id, 1.0f, 1, tile, j );
        jp.x = m_bpres.x-1;
        j = getVertex(jp.x, jp.y, jp.z);
        SetValF ( buf_id, 1.0f, 0, tile, j );
      }
    }

    // Y plane
    for (jp.z=0; jp.z < m_bpres.z; jp.z++) {
      for (jp.x=0; jp.x < m_bpres.x; jp.x++) {
        jp.y = 0;
        j = getVertex(jp.x, jp.y, jp.z);
        SetValF ( buf_id, 1.0f, 3, tile, j );
        jp.y = m_bpres.y-1;
        j = getVertex(jp.x, jp.y, jp.z);
        SetValF ( buf_id, 1.0f, 2, tile, j );
      }
    }


    // Z plane
    for (jp.y=0; jp.y < m_bpres.y; jp.y++) {
      for (jp.x=0; jp.x < m_bpres.x; jp.x++) {
        jp.z = 0;
        j = getVertex(jp.x, jp.y, jp.z);
        SetValF ( buf_id, 1.0f, 5, tile, j );
        jp.z = m_bpres.z-1;
        j = getVertex(jp.x, jp.y, jp.z);
        SetValF ( buf_id, 1.0f, 4, tile, j );
      }
    }
  }
}


float BeliefPropagation::BeliefProp_svd () {
  int64_t anch_cell=0, nei_cell=0;
  int d, r, c;

  int anch_tile_idx, anch_tile_idx_n;
  int nei_tile_idx, nei_tile_idx_n;
  int anch_tile, nei_tile;

  int nei_in_idx, anch_in_idx;

  float H_ij_a;
  float u_nxt_b, u_prev_b;
  float du;
  float mu_val;

  float rate = 1.0,
        max_diff=-1.0;

  int   odd_even_cell = -1;

  rate = op.step_rate;

  Vector3DI jp;

  // for all `nei`->`anch` messages in graph domain
  //
  for ( anch_cell=0; anch_cell < getNumVerts(); anch_cell++ ) {

    anch_tile_idx_n = getValI ( BUF_TILE_IDX_N, anch_cell );
    jp = getVertexPos(anch_cell);

    if (op.use_checkerboard) {
      odd_even_cell = (jp.x + jp.y + jp.z)%2;
      if (odd_even_cell==0) {
        for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {
          for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
            anch_tile = getValI ( BUF_TILE_IDX, anch_tile_idx, anch_cell );
            mu_val = getValF( BUF_MU, anch_in_idx, anch_tile, anch_cell );
            SetValF ( BUF_MU_NXT, (mu_val), anch_in_idx, anch_tile, anch_cell );
          }
        }
        continue;
      }
    }

    // 6 neighbors of j in 3D
    //
    for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {
      nei_cell = getNeighbor(anch_cell, jp, anch_in_idx);

      // pathological conditions
      // * cell has none (which is an error) or only 1 tile
      // * cell's neighbor falls off the end
      //
      if (anch_tile_idx_n <= 1) {
        if (anch_tile_idx_n == 1) {
          anch_tile = getValI ( BUF_TILE_IDX, anch_cell );
          SetValF( BUF_MU_NXT, (1.0), anch_in_idx, anch_tile, anch_cell );
        }
        continue;
      }
      if (nei_cell == -1) {
        for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
          anch_tile = getValI ( BUF_TILE_IDX, anch_tile_idx, anch_cell );
          SetValF( BUF_MU_NXT, (1.0), anch_in_idx, anch_tile, anch_cell );
        }
        continue;
      }

      // compute message from `nei` to `anch` for each a..
      // we're skipping some tiles, so zero out H
      //
      for (d=0; d<m_num_values; d++) { SetValF (BUF_H, 0.0, d ); }

      nei_tile_idx_n = getValI ( BUF_TILE_IDX_N, nei_cell );

      // pathological condition
      // * neighbor cell has 0 values (error) or only 1
      //
      if (nei_tile_idx_n <= 1) {
        for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
          anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
          SetValF( BUF_MU_NXT, (1.0), anch_in_idx, anch_tile, anch_cell );
        }
        continue;
      }
      //Vector3DI jp = getVertexPos(nei_cell);
      int numbrs = getNumNeighbors(nei_cell);

      // cache direction in which to ignore anch_cell
      int nei_in_ignore = getOppositeDir( anch_in_idx );

      // process all tiles for current nei_cell
      for (nei_tile_idx=0; nei_tile_idx < nei_tile_idx_n; nei_tile_idx++) {

        nei_tile = getValI( BUF_TILE_IDX, nei_tile_idx, nei_cell );

        // first compute Hij_t
        // initialize Hij(a) = gi(a)
        //
        H_ij_a = getValF (BUF_G, nei_tile);

        // starting MU for nei_cell and tile
        float* currMu = (float*) getPtr (BUF_MU, 0, nei_tile, nei_cell);

        // inner loop over neighbors of nei_cell
        for (nei_in_idx=0; nei_in_idx < numbrs; nei_in_idx++ ) {

            // compute: Hij(a) = gi(a) * PROD mu{ki}_a

            // Optimized:
            // - Eliminated boundary check using WriteBoundaryMU. getNeighbor was only used to check if _neinei_cell=-1
            // - Use direction instead of cell pos to eliminate anchor cell
            // - Optimized MUPTR: reorganized MU with 'nbr' as linear mem variable.
            if (nei_in_idx != nei_in_ignore) {
                H_ij_a *= *currMu;
            }
            currMu++;

        }

        SetValF (BUF_H, H_ij_a, nei_tile );
      }


      //--- ***************
      //--- ***************
      //--- ***************

      for (d=0; d < m_num_values; d++) { SetValF( BUF_SVD_VEC, 0.0, d); }

#ifdef BPC_SVD_NAIVE
      for (r=0; r<m_svd_nsv[anch_in_idx]; r++) {
        u_nxt_b = 0.0;
        for (c=0; c<m_num_values; c++) {
          u_nxt_b += getValF( BUF_SVD_Vt, r, c, anch_in_idx ) * getVal( BUF_H, c );
        }
        SetVal( BUF_SVD_VEC, r, u_nxt_b );
      }

      for (r=0; r < m_num_values; r++) {
        u_nxt_b = 0.0;
        for (c=0; c < m_svd_nsv[anch_in_idx]; c++) {
          u_nxt_b += getValF( BUF_SVD_U, r, c, anch_in_idx ) * getVal( BUF_SVD_VEC, c );
        }

        u_prev_b = getVal(BUF_MU, anch_in_idx, anch_cell, r);
        du = u_nxt_b - u_prev_b;
        if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }
        SetVal (BUF_MU_NXT, anch_in_idx, anch_cell, r, u_prev_b + du*rate );
      }
#else
      for (r=0; r < m_svd_nsv[anch_in_idx]; r++) {
        u_nxt_b = 0.0;

        //for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        //  anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );
        //  u_nxt_b += getValF( BUF_SVD_Vt, r, anch_tile, anch_in_idx ) * getVal( BUF_H, anch_tile );
        //}

        for (c=0; c < m_num_values; c++) {
          u_nxt_b += getValF( BUF_SVD_Vt, r, c, anch_in_idx ) * getValF( BUF_H, c );
        }

        SetValF ( BUF_SVD_VEC, u_nxt_b , r );
      }

      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

        u_nxt_b = 0.0;
        for (c=0; c < m_svd_nsv[anch_in_idx]; c++) {
          u_nxt_b += getValF( BUF_SVD_U, anch_tile, c, anch_in_idx ) * getValF( BUF_SVD_VEC, c );
        }

        u_prev_b = getValF (BUF_MU, anch_in_idx,  anch_tile, anch_cell );
        du = u_nxt_b - u_prev_b;
        if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }
        SetValF (BUF_MU_NXT, (u_prev_b + du*rate), anch_in_idx, anch_tile, anch_cell );
      }
#endif

      /*
      // now compute mu_ij_t+1 = Fij * hij
      // b = rows in f{ij}(a,b), also elements of mu(b)/
      //
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getVali( BUF_TILE_IDX, anch_cell, anch_tile_idx );

        u_nxt_b = 0.0;
        nei_to_anch_dir_idx = m_dir_inv[anch_in_idx];

        // a = cols in f{ij}(a,b), also elements of h(a)
        //

        #ifdef OPT_FH
          // Optimized: F and H access using pointers
          float* currH = getPtr(BUF_H, 0);
          float* currF = getPtrF(BUF_F, 0, anch_tile, nei_to_anch_dir_idx);

          for (d=0; d < m_num_values; d++) {
            u_nxt_b += (*currF) * (*currH);

            currF++;
            currH++;
          }
        #else
          // Non-optimized
          for (d=0; d < m_num_values; d++) {
            // in orig code nei_to_arch_dir_idx computed here even though it is ok outside this loop
            nei_to_anch_dir_idx = m_dir_inv[anch_in_idx];
            u_nxt_b += getValF(BUF_F, anch_tile, nei_to_anch_dir_idx, d) * getVal(BUF_H, d);
          }
        #endif

        u_prev_b = getVal(BUF_MU, anch_in_idx, anch_cell, anch_tile);
        du = u_nxt_b - u_prev_b;
        if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }
        SetVal (BUF_MU_NXT, anch_in_idx, anch_cell, anch_tile, u_prev_b + du*rate );

      }
      */

      //--- ***************
      //--- ***************
      //--- ***************

    }


  }

  return max_diff;
}


// WARNING!
// There might be a bug here.
//
// ./bpc -R ./assets/stair_rule.csv -N ./assets/stair_name.csv -I 100000000 -D 10 -G 4 -V 1 -S 124  -e 0.00025
// ./bpc -R ./assets/stair_rule.csv -N ./assets/stair_name.csv -I 100000000 -D 10 -G 4 -V 1 -S 124  -e 0.00025 -E
//
// The svd option for the above fails to converge whereas the non-svd one converges.
// These should be exactly equivalent, so there's something bad going on.
//
//
float BeliefPropagation::BeliefProp_cell_residue_svd (int64_t anch_cell) {

  int64_t nei_cell=0;
  int d, r, c;

  int anch_tile_idx, anch_tile_idx_n;
  int nei_tile_idx, nei_tile_idx_n;
  int anch_tile, nei_tile;

  int nei_in_idx, anch_in_idx;

  float H_ij_a;
  float u_nxt_b, u_prev_b;
  float du;
  float mu_val;

  float rate = 1.0,
        max_diff=-1.0,
        mu_cur_val,
        mu_nxt_val;

  int   odd_even_cell = -1;

  rate = op.step_rate;

  Vector3DI jp;

  anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );
  jp = getVertexPos(anch_cell);

  // 6 neighbors of j in 3D
  //
  for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {
    nei_cell = getNeighbor(anch_cell, jp, anch_in_idx);

    // pathological conditions
    // * cell has none (which is an error) or only 1 tile
    // * cell's neighbor falls off the end
    //
    if (anch_tile_idx_n <= 1) {
      if (anch_tile_idx_n == 1) {
        anch_tile = getValI( BUF_TILE_IDX, anch_cell, 0 );
        SetValF( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell );

        mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell);
        mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell);
        indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );
      }
      continue;
    }
    if (nei_cell == -1) {
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        SetValF( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell );

        mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell);
        mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell);
        indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );
      }
      continue;
    }

    // compute message from `nei` to `anch` for each a..
    // we're skipping some tiles, so zero out H
    //
    for (d=0; d<m_num_values; d++) { SetValF(BUF_H, 0.0, d); }

    nei_tile_idx_n = getValI( BUF_TILE_IDX_N, nei_cell );

    // pathological condition
    // * neighbor cell has 0 values (error) or only 1
    //
    if (nei_tile_idx_n <= 1) {
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        SetValF( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell );

        mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell);
        mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell);
        indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );
      }
      continue;
    }

    //Vector3DI jp = getVertexPos(nei_cell);
    //
    int numbrs = getNumNeighbors(nei_cell);

    // cache direction in which to ignore anch_cell
    //
    int nei_in_ignore = getOppositeDir( anch_in_idx );

    // process all tiles for current nei_cell
    //
    for (nei_tile_idx=0; nei_tile_idx < nei_tile_idx_n; nei_tile_idx++) {

      nei_tile = getValI( BUF_TILE_IDX, nei_tile_idx, nei_cell );

      // first compute Hij_t
      // initialize Hij(a) = gi(a)
      //
      H_ij_a = getValF(BUF_G, nei_tile);

      // starting MU for nei_cell and tile
      //
      float* currMu = (float*) getPtr (BUF_MU, 0, nei_tile, nei_cell );

      // inner loop over neighbors of nei_cell
      //
      for (nei_in_idx=0; nei_in_idx < numbrs; nei_in_idx++ ) {

        // compute: Hij(a) = gi(a) * PROD mu{ki}_a

        // Optimized:
        // - Eliminated boundary check using WriteBoundaryMU. getNeighbor was only used to check if _neinei_cell=-1
        // - Use direction instead of cell pos to eliminate anchor cell

        // - Optimized MUPTR: reorganized MU with 'nbr' as linear mem variable.
        if (nei_in_idx != nei_in_ignore) {
          H_ij_a *= *currMu;
        }
        currMu++;

      }

      SetValF (BUF_H, (H_ij_a), nei_tile );
    }


    //--- ***************
    //--- ***************
    //--- ***************

    for (d=0; d < m_num_values; d++) { SetValF( BUF_SVD_VEC, 0.0, d); }

    for (r=0; r < m_svd_nsv[anch_in_idx]; r++) {
      u_nxt_b = 0.0;

      for (c=0; c < m_num_values; c++) {
        u_nxt_b += getValF( BUF_SVD_Vt, r, c, anch_in_idx ) * getValF( BUF_H, c );
      }

      SetValF( BUF_SVD_VEC, r, u_nxt_b );
    }

    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      u_nxt_b = 0.0;
      for (c=0; c < m_svd_nsv[anch_in_idx]; c++) {
        u_nxt_b += getValF( BUF_SVD_U, anch_tile, c, anch_in_idx ) * getValF( BUF_SVD_VEC, c );
      }

      u_prev_b = getValF (BUF_MU, anch_in_idx, anch_tile, anch_cell );
      du = u_nxt_b - u_prev_b;
      if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }
      SetValF (BUF_MU_NXT, (u_prev_b + du*rate), anch_in_idx, anch_tile, anch_cell );

      mu_cur_val = getValF( BUF_MU,      anch_in_idx, anch_tile, anch_cell );
      mu_nxt_val = getValF( BUF_MU_NXT,  anch_in_idx, anch_tile, anch_cell );
      indexHeap_update_mu_pos( anch_in_idx, anch_cell, anch_tile, fabs(mu_cur_val - mu_nxt_val) );
    }

  }

  return max_diff;
}

float BeliefPropagation::BeliefProp () {

  int64_t anch_cell=0, nei_cell=0;
  int d;

  int anch_tile_idx, anch_tile_idx_n;
  int nei_tile_idx, nei_tile_idx_n;
  int anch_tile, nei_tile;

  int nei_in_idx, anch_in_idx;
  int nei_to_anch_dir_idx;

  float H_ij_a;
  float u_nxt_b, u_prev_b;
  float du;

  float rate = 1.0,
        max_diff=-1.0;

  float dmu;

  float mu_val;
  int   odd_even_cell = -1;
  int   eval;

  rate = op.step_rate;

  Vector3DI jp;

  // for all `nei`->`anch` messages in graph domain
  //
  for ( anch_cell=0; anch_cell < getNumVerts(); anch_cell++ ) {


    // if already decided, continue
    anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );    

    if ( anch_tile_idx_n==1 ) { continue; }


    jp = getVertexPos(anch_cell);

    if ( op.alg_accel==ALG_ACCEL_WAVE) {
        //----- WAVEFRONT BP
        // vertex dmu was temporarily stored in mu_nxt from MaxDiffMU of last step
        float conv_frac = 2.0;
        eval = 0;
        for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {
          nei_cell = getNeighbor(anch_cell, jp, anch_in_idx);
          if (nei_cell==-1) {
              eval++;
          } else {
              dmu = getValF( BUF_MU_NXT, 0, 0, nei_cell );
              if ( dmu >= op.eps_converge * conv_frac ) eval++;
            }
        }
        dmu = getValF( BUF_MU_NXT, 0, 0, anch_cell ); 
        if ( eval==0 && dmu < op.eps_converge * conv_frac ) { continue; }
        //------
    }

    if (op.use_checkerboard) {
      odd_even_cell = (jp.x + jp.y + jp.z)%2;
      if (odd_even_cell==0) {
        for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {

          for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
            anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

            mu_val = getValF( BUF_MU, anch_in_idx, anch_tile, anch_cell );

            SetValF ( BUF_MU_NXT, (mu_val), anch_in_idx, anch_tile, anch_cell );
          }
        }
        continue;
      }
    }


    // 6 neighbors of j in 3D
    //
    for (anch_in_idx=0; anch_in_idx < getNumNeighbors(anch_cell); anch_in_idx++) {
      nei_cell = getNeighbor(anch_cell, jp, anch_in_idx);

      // pathological conditions
      // * cell has none (which is an error) or only 1 tile
      // * cell's neighbor falls off the end
      //
      if (anch_tile_idx_n <= 1) {
        if (anch_tile_idx_n == 1) {
          anch_tile = getValI( BUF_TILE_IDX, anch_cell );
          SetValF ( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell);
        }
        continue;
      }
      if (nei_cell == -1) {
        for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
          anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
          SetValF ( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell);
        }
        continue;
      }

      // compute message from `nei` to `anch` for each a..
      // we're skipping some tiles, so zero out H
      //
      for (d=0; d<m_num_values; d++) { SetValF (BUF_H, 0.0, d); }

      nei_tile_idx_n = getValI( BUF_TILE_IDX_N, nei_cell );

      // pathological condition
      // * neighbor cell has 0 values (error) or only 1
      //
      if (nei_tile_idx_n <= 1) {
        for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
          anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
          SetValF ( BUF_MU_NXT, 1.0, anch_in_idx, anch_tile, anch_cell);
        }
        continue;
      }
      //Vector3DI jp = getVertexPos(nei_cell);
      int numbrs = getNumNeighbors(nei_cell);

      // cache direction in which to ignore anch_cell
      int nei_in_ignore = getOppositeDir( anch_in_idx );

      // process all tiles for current nei_cell
      for (nei_tile_idx=0; nei_tile_idx < nei_tile_idx_n; nei_tile_idx++) {

        nei_tile = getValI ( BUF_TILE_IDX, nei_tile_idx, nei_cell );

        // first compute Hij_t
        // initialize Hij(a) = gi(a)
        //
        H_ij_a = getValF( BUF_G, nei_tile );

        // starting MU for nei_cell and tile
        float* currMu = (float*) getPtr (BUF_MU, 0, nei_tile, nei_cell );

        // inner loop over neighbors of nei_cell
        for (nei_in_idx=0; nei_in_idx < numbrs; nei_in_idx++ ) {

          // compute: Hij(a) = gi(a) * PROD mu{ki}_a

          #ifdef OPT_MUBOUND
            // Optimized:
            // - Eliminated boundary check using WriteBoundaryMU. getNeighbor was only used to check if _neinei_cell=-1
            // - Use direction instead of cell pos to eliminate anchor cell

            #ifdef OPT_MUPTR
              // - Optimized MUPTR: reorganized MU with 'nbr' as linear mem variable.
              if (nei_in_idx != nei_in_ignore) {
                H_ij_a *= *currMu;
              }
              currMu++;
            #else
              if (nei_in_idx != nei_in_ignore) {
                H_ij_a *= getValF(BUF_MU, nei_in_idx, nei_tile, nei_cell );
              }
            #endif

          #else
            // Non-Optimized Bound:
            // - Original bounds check /w getNeighbor
            _neinei_cell = getNeighbor(nei_cell, jp, nei_in_idx);

            #ifdef OPT_MUPTR
              // - Optimized MUPTR: reorganized MU with 'nbr' as linear mem variable.
              if ((_neinei_cell != -1) && (_neinei_cell != anch_cell)) {
                H_ij_a *= *currMu;
              }
              currMu++;
            #else
              if ((_neinei_cell != -1) && (_neinei_cell != anch_cell)) {
                H_ij_a *= getValF ( BUF_MU, nei_in_idx, nei_tile, nei_cell );
              }
            #endif
          #endif
        }

        SetValF (BUF_H, H_ij_a, nei_tile );
      }


      // now compute mu_ij_t+1 = Fij * hij
      // b = rows in f{ij}(a,b), also elements of mu(b)/
      //
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

        u_nxt_b = 0.0;
        nei_to_anch_dir_idx = m_dir_inv[anch_in_idx];

        // a = cols in f{ij}(a,b), also elements of h(a)
        //

        #ifdef OPT_FH
          // Optimized: F and H access using pointers
          float* currH = (float*) getPtr (BUF_H);
          float* currF = (float*) getPtr (BUF_F, 0, anch_tile, nei_to_anch_dir_idx);    // <B, B, 6>

          for (d=0; d < m_num_values; d++) {
            u_nxt_b += (*currF) * (*currH);

            currF++;
            currH++;
          }
        #else
          // Non-optimized
          for (d=0; d < m_num_values; d++) {
            // in orig code nei_to_arch_dir_idx computed here even though it is ok outside this loop
            nei_to_anch_dir_idx = m_dir_inv[anch_in_idx];
            u_nxt_b += getValF(BUF_F, d, anch_tile, nei_to_anch_dir_idx ) * getVal(BUF_H, d);
          }
        #endif

        u_prev_b = getValF (BUF_MU, anch_in_idx, anch_tile, anch_cell );

        du = u_nxt_b - u_prev_b;

        if (max_diff < fabs(du)) { max_diff = (float)fabs(du); }

        SetValF (BUF_MU_NXT, (u_prev_b + du*rate), anch_in_idx, anch_tile, anch_cell  );
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
        _eps = op.eps_zero;
  float _b_i_t_a = 0.0;

  sum = 0.0;

  Vector3DI jp = getVertexPos(anch_cell);

  anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );

  for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
    anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

    SetValF( BUF_BELIEF, 1.0, anch_tile );

    _b_i_t_a = getValF( BUF_G, anch_tile );

    for (dir_idx=0; dir_idx < getNumNeighbors(anch_cell); dir_idx++) {
      nei_cell = getNeighbor(anch_cell, dir_idx);
      if (nei_cell < 0) { continue; }

      _b_i_t_a *= getValF ( BUF_MU, dir_idx, anch_tile, anch_cell );
    }
    SetValF (BUF_BELIEF, _b_i_t_a, anch_tile );

    sum += getValF( BUF_BELIEF, anch_tile);
  }

  if (sum > _eps) {
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      _b_i_t_a = getValF( BUF_BELIEF, anch_tile ) / sum;
      SetValF( BUF_BELIEF, _b_i_t_a, anch_tile );
    }
  }
  else {
    _b_i_t_a = 1.0 / (float) anch_tile_idx_n;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      SetValF( BUF_BELIEF, _b_i_t_a, anch_tile );
    }
  }

}

int BeliefPropagation::_pick_tile(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief) {
  return _pick_tile_max_belief(anch_cell, max_cell, max_tile, max_tile_idx, max_belief);
}

int BeliefPropagation::_pick_tile_max_belief(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief) {
  float f, max_p=-1.0;
  int64_t anch_tile_idx,
          anch_tile_idx_n,
          anch_tile ;

  anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );
  if (anch_tile_idx_n==0) { return -1; }
  //if (anch_tile_idx_n==1) { continue; }

  for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
    anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

    f = getValF( BUF_BELIEF, anch_tile );

    if (op.verbose >= VB_INTRASTEP ) {
      printf("##### f: %f, max_p %f, anch_cell %i, anch_tile %i, anch_tile_idx %i\n",
          f, max_p, (int) anch_cell, (int)anch_tile, (int)anch_tile_idx);
    }

    if ( max_p < f ) {
      max_p = f;
      *max_cell = anch_cell;
      *max_tile = anch_tile;
      *max_tile_idx = anch_tile_idx;
      *max_belief = f;
    }
  }

  return 0;

}

int BeliefPropagation::_pick_tile_min_belief(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief) {
  float f, min_p=-1.0;
  int64_t anch_tile_idx,
          anch_tile_idx_n,
          anch_tile ;

  anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );
  if (anch_tile_idx_n==0) { return -1; }
  //if (anch_tile_idx_n==1) { continue; }

  for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
    anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

    f = getValF( BUF_BELIEF, anch_tile );

    if (op.verbose >= VB_INTRASTEP) {
      printf("##### f: %f, min_p%f, anch_cell %i, anch_tile %i, anch_tile_idx %i\n",
          f, min_p, (int)anch_cell, (int)anch_tile, (int)anch_tile_idx);
    }

    if ( min_p > f ) {
      min_p = f;
      *max_cell = anch_cell;
      *max_tile = anch_tile;
      *max_tile_idx = anch_tile_idx;
      *max_belief = f;
    }
  }

  return 0;

}

int BeliefPropagation::_pick_tile_pdf(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief) {
  float p, f, sum_p;
  int64_t anch_tile_idx,
          anch_tile_idx_n,
          anch_tile ;

  anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );
  if (anch_tile_idx_n==0) { return -1; }
  //if (anch_tile_idx_n==1) { continue; }

  p = m_rand.randF();

  sum_p = 0.0;
  for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
    anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

    f = getValF ( BUF_BELIEF, anch_tile );
    sum_p += f;
    if ( (sum_p > p) ||
         (anch_tile_idx == (anch_tile_idx_n-1)) ) {
      *max_cell = anch_cell;
      *max_tile = anch_tile;
      *max_tile_idx = anch_tile_idx;
      *max_belief = f;
      break;
    }
  }

  return 0;

}


int BeliefPropagation::chooseMinEntropyMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief) {
  int64_t anch_cell=0;
  int32_t anch_tile_idx, anch_tile_idx_n, anch_tile;
  int count=0;

  float _max_belief = -1.0,
        f,
        p;

  int64_t _max_cell = -1;
  int32_t _max_tile = -1,
          _max_tile_idx = -1;

  float _eps = op.eps_zero;

  float _min_entropy = -1.0,
        _entropy_sum = 0.0;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {
    cellUpdateBelief(anch_cell);

    anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );
    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) { continue; }

    _entropy_sum = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      f = getValF ( BUF_BELIEF, anch_tile );
      if (f > _eps) {
        _entropy_sum += -f*logf(f);
      }

    }

    //DEBUG
    //
    if (op.verbose >= VB_DEBUG) {
      printf("anch_cell: %i, _entropy_sum: %f, n: %i\n", (int)anch_cell, (float)_entropy_sum, (int)anch_tile_idx_n);
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        f = getValF ( BUF_BELIEF, anch_tile );
        printf(" (%i)%f", (int)anch_tile, f);
      }
      printf("\n");
    }

    // if it's the first time we're doing an entropy calculation,
    // set initial value
    //
    if ( _min_entropy < 0 ) {
      _min_entropy = _entropy_sum;

      _pick_tile_max_belief( anch_cell, &_max_cell, &_max_tile, &_max_tile_idx, &_max_belief );
      count=1;

      //DEBUG
      if (op.verbose >= VB_DEBUG ) {
        printf("  ## (i.0) picked cell:%i, tile:%i, tile_idx:%i, belief:%f, count:%i\n",
            (int)_max_cell, (int)_max_tile, (int)_max_tile_idx, (float)_max_belief, (int)count);
      }

      continue;
    }

    // if we trigger a new minimum entropy...
    //
    if ( _entropy_sum < (_min_entropy + _eps) ) {

      if (op.verbose >= VB_DEBUG ) {
        printf("  !! picking cell %i (entropy_sum %f < _min_entropy %f + %f)\n", (int)anch_cell,
            _entropy_sum, _min_entropy, _eps);
      }

      p = m_rand.randF();

      // if it's strictly less than the minimum entropy we've observed,
      // reset count and choose a tile to fix
      //
      if ( _entropy_sum < (_min_entropy - _eps) ) {
        _pick_tile_max_belief( anch_cell, &_max_cell, &_max_tile, &_max_tile_idx, &_max_belief );
        count=1;

        //DEBUG
        if (op.verbose >= VB_DEBUG ) {
          printf("  ## (a.0) picked cell:%i, tile:%i, tile_idx:%i, belief:%f, count:%i\n",
              (int)_max_cell, (int)_max_tile, (int)_max_tile_idx, (float)_max_belief, (int)count);
        }
      }

      // else we've seen the same minimum entropy, decide whether we want
      // to keep the entry we've already chosen or redraw from the current
      // cell
      //
      else {
        count++;
        p = m_rand.randF();
        if ( p < (1.0/(float)count) ) {
          _pick_tile_max_belief( anch_cell, &_max_cell, &_max_tile, &_max_tile_idx, &_max_belief );

          if (op.verbose >= VB_DEBUG ) {
            //DEBUG
            printf("  ## (b.0) picked cell:%i, tile:%i, tile_idx:%i, belief:%f, count:%i\n",
                (int)_max_cell, (int)_max_tile, (int)_max_tile_idx, (float)_max_belief, (int)count);
          }

        }
      }

      _min_entropy = _entropy_sum;
    }

  }

  if (max_cell)     { *max_cell     = _max_cell; }
  if (max_tile)     { *max_tile     = _max_tile; }
  if (max_tile_idx) { *max_tile_idx = _max_tile_idx; }
  if (max_belief)   { *max_belief   = _max_belief; }

  return count;
}

// wip
//
int BeliefPropagation::chooseMinEntropyMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief) {
  int64_t anch_cell=0;
  int32_t anch_tile_idx, anch_tile_idx_n, anch_tile;
  int count=0;

  float _min_belief = -1.0,
        f,
        p;

  int64_t _min_cell = -1;
  int32_t _min_tile = -1,
          _min_tile_idx = -1;

  float _eps = op.eps_zero;

  float _min_entropy = -1.0,
        _entropy_sum = 0.0;

  Vector3DI vp;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {
    cellUpdateBelief(anch_cell);

    anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );
    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) { continue; }

    _entropy_sum = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      f = getValF( BUF_BELIEF, anch_tile );
      if (f > _eps) {
        _entropy_sum += -f*logf(f);
      }

    }

    //DEBUG
    if (op.verbose >= VB_DEBUG ) {
      printf("anch_cell: %i, _entropy_sum: %f, n: %i\n", (int)anch_cell, (float)_entropy_sum, (int)anch_tile_idx_n);
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        f = getValF( BUF_BELIEF, anch_tile );
        printf(" (%i)%f", (int)anch_tile, f);
      }
      printf("\n");
    }

    // if it's the first time we're doing an entropy calculation,
    // set initial value
    //
    if ( _min_entropy < 0 ) {
      _min_entropy = _entropy_sum;

      _pick_tile_min_belief( anch_cell, &_min_cell, &_min_tile, &_min_tile_idx, &_min_belief );
      count=1;

      //DEBUG
      if (op.verbose >= VB_DEBUG ) {
        vp = getVertexPos(_min_cell);
        printf("  ## (i.1) picked cell:[%i,%i,%i](%i), tile:%i, tile_idx:%i, belief:%f, count:%i\n",
            (int)vp.x, (int)vp.y, (int)vp.z,
            (int)_min_cell, (int)_min_tile, (int)_min_tile_idx, (float)_min_belief, (int)count);
      }

      continue;
    }

    // if we trigger a new minimum entropy...
    //
    if ( _entropy_sum < (_min_entropy + _eps) ) {

      if (op.verbose >= VB_DEBUG ) {
        printf("  !! picking cell %i (entropy_sum %f < _min_entropy %f + %f)\n", (int)anch_cell,
            _entropy_sum, _min_entropy, _eps);
      }

      p = m_rand.randF();

      // if it's strictly less than the minimum entropy we've observed,
      // reset count and choose a tile to fix
      //
      if ( _entropy_sum < (_min_entropy - _eps) ) {
        _pick_tile_min_belief( anch_cell, &_min_cell, &_min_tile, &_min_tile_idx, &_min_belief );
        count=1;

        //DEBUG
        if (op.verbose >= VB_INTRASTEP ) {
          printf("  ## (a.1) picked cell:[%i,%i,%i](%i), tile:%i, tile_idx:%i, belief:%f, count:%i\n",
              (int)vp.x, (int)vp.y, (int)vp.z,
              (int)_min_cell, (int)_min_tile, (int)_min_tile_idx, (float)_min_belief, (int)count);
        }
      }

      // else we've seen the same minimum entropy, so decide whether we want
      // to keep the entry we've already chosen or redraw from the current
      // cell
      //
      else {
        count++;
        p = m_rand.randF();
        if ( p < (1.0/(float)count) ) {
          _pick_tile_min_belief( anch_cell, &_min_cell, &_min_tile, &_min_tile_idx, &_min_belief );

          if (op.verbose >= VB_INTRASTEP ) {
            //DEBUG
            printf("  ## (b.1) picked cell:[%i,%i,%i](%i), tile:%i, tile_idx:%i, belief:%f, count:%i\n",
                (int)vp.x, (int)vp.y, (int)vp.z,
                (int)_min_cell, (int)_min_tile, (int)_min_tile_idx, (float)_min_belief, (int)count);
          }

        }
      }

      _min_entropy = _entropy_sum;
    }

  }

  if (min_cell)     { *min_cell     = _min_cell; }
  if (min_tile)     { *min_tile     = _min_tile; }
  if (min_tile_idx) { *min_tile_idx = _min_tile_idx; }
  if (min_belief)   { *min_belief   = _min_belief; }

  if (op.verbose >= VB_INTRASTEP ) {
    printf("?? count:%i\n", (int)count);
  }

  return count;
}

int BeliefPropagation::chooseMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief) {
  int64_t anch_cell=0,
          nei_cell=0;
  int32_t anch_tile_idx, anch_tile_idx_n, anch_tile;
  int count=0;

  float _max_belief = -1.0, f, p;
  int64_t _max_cell = -1;
  int32_t _max_tile = -1,
          _max_tile_idx = -1;

  //float _eps = (1.0/(1024.0*1024.0));
  float _eps = op.eps_zero;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {
    cellUpdateBelief(anch_cell);

    anch_tile_idx_n = getValI ( BUF_TILE_IDX_N, anch_cell );
    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) { continue; }

    //DEBUG
    if (op.verbose >= VB_DEBUG ) {
      printf("anch_cell: %i, n: %i\n", (int)anch_cell, (int)anch_tile_idx_n);
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        f = getValF( BUF_BELIEF, anch_tile );
        printf(" (%i)%f", (int)anch_tile, f);
      }
      printf("\n");
    }


    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      f = getValF( BUF_BELIEF, anch_tile );

      if (f > (_max_belief-_eps)) {

        if (f > _max_belief) {
          _max_cell = anch_cell;
          _max_tile = anch_tile;
          _max_tile_idx = anch_tile_idx;
          _max_belief = f;
          count=1;

          if (op.verbose >= VB_DEBUG ) {
            //DEBUG
            printf("  ## (a.2) picked cell:%i, tile:%i, tile_idx:%i, belief:%f, count:%i\n",
                (int)_max_cell, (int)_max_tile, (int)_max_tile_idx, (float)_max_belief, (int)count);
          }


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

            if (op.verbose >= VB_INTRASTEP ) {
              //DEBUG
              printf("  ## (b.2) picked cell:%i, tile:%i, tile_idx:%i, belief:%f, count:%i\n",
                  (int)_max_cell, (int)_max_tile, (int)_max_tile_idx, (float)_max_belief, (int)count);
            }


          }
        }

      }

      //nei_cell = getNeighbor(anch_cell, jp, dir_idx);
      //nei_cell = getNeighbor(anch_cell, dir_idx);
      //if (nei_cell < 0) { continue; }


    }

  }

  if (op.verbose >= VB_INTRASTEP ) {
    printf("## chooseMaxBelief: choosing cell:%i, tile:%i, tile_idx:%i, max_belief:%f, count:%i\n",
        (int)_max_cell, (int)_max_tile, (int)_max_tile_idx, (float)_max_belief, (int)count);
  }

  if (max_cell) { *max_cell = _max_cell; }
  if (max_tile) { *max_tile = _max_tile; }
  if (max_tile_idx)  { *max_tile_idx = _max_tile_idx; }
  if (max_belief) { *max_belief = _max_belief; }

  return count;
}

int BeliefPropagation::chooseMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief) {
  int64_t anch_cell=0;
  int32_t anch_tile_idx, anch_tile_idx_n, anch_tile;
  int count=0;

  float _min_belief = -1.0, f, p;
  int64_t _min_cell = -1;
  int32_t _min_tile = -1,
          _min_tile_idx = -1;

  //float _eps = (1.0/(1024.0*1024.0));
  float _eps = op.eps_zero;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {
    cellUpdateBelief(anch_cell);

    anch_tile_idx_n = getValI ( BUF_TILE_IDX_N, anch_cell );
    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) { continue; }

    //DEBUG
    if (op.verbose >= VB_DEBUG ) {
      printf("anch_cell: %i, n: %i\n", (int)anch_cell, (int)anch_tile_idx_n);
      for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
        anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
        f = getValF( BUF_BELIEF, anch_tile );
        printf(" (%i)%f", (int)anch_tile, f);
      }
      printf("\n");
    }


    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      f = getValF( BUF_BELIEF, anch_tile );

      if ( (_min_belief < 0.0) ||
           (f < (_min_belief+_eps)) ) {

        if ( (_min_belief < 0.0) ||
             (f < (_min_belief - _eps)) ) {
          _min_cell = anch_cell;
          _min_tile = anch_tile;
          _min_tile_idx = anch_tile_idx;
          _min_belief = f;
          count=1;

          if (op.verbose >= VB_DEBUG ) {
            //DEBUG
            printf("  ## (a.3) picked cell:%i, tile:%i, tile_idx:%i, belief:%f, count:%i\n",
                (int)_min_cell, (int)_min_tile, (int)_min_tile_idx, (float)_min_belief, (int)count);
          }


        }

        // randomize 'equal' choices
        //
        else {
          count++;
          p = m_rand.randF();
          if ( p < (1.0/(float)count) ) {
            _min_cell = anch_cell;
            _min_tile = anch_tile;
            _min_tile_idx = anch_tile_idx;
            _min_belief = f;

            if (op.verbose >= VB_INTRASTEP ) {
              //DEBUG
              printf("  ## (b.3) picked cell:%i, tile:%i, tile_idx:%i, belief:%f, count:%i\n",
                  (int)_min_cell, (int)_min_tile, (int)_min_tile_idx, (float)_min_belief, (int)count);
            }


          }
        }

      }

    }

  }

  if (min_cell) { *min_cell = _min_cell; }
  if (min_tile) { *min_tile = _min_tile; }
  if (min_tile_idx)  { *min_tile_idx = _min_tile_idx; }
  if (min_belief) { *min_belief = _min_belief; }

  return count;
}

int BeliefPropagation::chooseMinEntropy(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_entropy) {

  int64_t anch_cell=0;
  int32_t anch_tile_idx, anch_tile_idx_n, anch_tile;
  int count=0;

  float p, g, _entropy, g_sum, g_cdf;

  float _min_entropy= -1.0;
  int64_t _min_cell = -1;
  int32_t _min_tile = -1,
          _min_tile_idx = -1;
  float _eps = op.eps_zero;

  for (anch_cell=0; anch_cell < m_num_verts; anch_cell++) {


    anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );

    //DEBUG
    //printf("chooseMinEntropy: anch_cell:%i (tile_idx_n %i)\n",
    //    (int)anch_cell,(int)anch_tile_idx_n);

    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) {

      //DEBUG
      //printf("chooseMinEntropy: skip anch_cell:%i (tile_idx_n %i)\n",
      //    (int)anch_cell,(int)anch_tile_idx_n);


      continue;
    }

    g_sum = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
      g = getValF( BUF_G, anch_tile );
      g_sum += g;
    }

    if (g_sum < _eps) {

      //DEBUG
      //printf("chooseMinEntropy: skipping anch_chell:%i (%f < %f)\n",
      //    (int)anch_cell, (float)g_sum, (float)_eps);

      continue;
    }

    _entropy = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
      g = getValF( BUF_G, anch_tile ) / g_sum;
      if (g > _eps) {
        _entropy += -g * logf(g);
      }
    }

    //DEBUG
    //printf("chooseMinEntropy: (cell:%i) cur entropy: %f (g_sum:%f) (_eps:%0.12f)\n",
    //    (int)anch_cell, (float)_entropy, (float)g_sum, _eps);


    // initialize min entropy
    //
    if (count==0) {

      //DEBUG
      //printf("chooseMinEntropy: (cell:%i) init _min_entropy = %f\n",
      //    (int)anch_cell, (float)_entropy);

      _min_cell     = anch_cell;
      _min_entropy = _entropy;
      count=1;
      continue;
    }

    if (_entropy < (_min_entropy + _eps)) {

      //DEBUG
      //printf("chooseMinEntropy: cp.0\n");


      if (_entropy < _min_entropy) {

        //DEBUG
        //printf("chooseMinEntropy: cell:%i min_entropy:%f (was %f)\n",
        //    (int)anch_cell, _entropy, _min_entropy);

        _min_entropy  = _entropy;
        _min_cell     = anch_cell;
        count=1;
      }

      // randomize 'equal' choices
      //
      else {

        count++;
        p = m_rand.randF();

        //DEBUG
        //printf("chooseMinEntropy: cp.1 count now %i, p %f, (1/count=%f)\n",
        //    (int)count, (float)p, (float)(1.0/(float)count));


        if ( p < (1.0/(float)count) ) {

          //DEBUG
          //printf("chooseMinEntropy: cell:%i min_entropy:%f (was %f) (choosing alternate min with same entropy)\n",
          //    (int)anch_cell, _entropy, _min_entropy);

          _min_entropy  = _entropy;
          _min_cell     = anch_cell;
        }
      }

    }

  }

  if (_min_cell<0) {

    //DEBUG
    //printf("chooseMinEntropy: _min_cell < 0 (%i), done\n", (int)_min_cell);

    return 0;
  }

  _min_tile = getValI( BUF_TILE_IDX, _min_cell, 0 );
  _min_tile_idx = 0;

  // now we choose a particular tile from the tile position
  // (both tile ID and tile index)
  //
  anch_cell = _min_cell;
  anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );
  g_sum = 0.0;
  for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
    anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
    g_sum += getValF( BUF_G, anch_tile);
  }

  if (g_sum > _eps) {

    p = m_rand.randF();

    g_cdf = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
      g_cdf += getValF( BUF_G, anch_tile ) / g_sum;

      if (p < g_cdf) {
        _min_tile     = anch_tile;
        _min_tile_idx = anch_tile_idx;

        //DEBUG
        //printf("wfc: chooseMinEntropy: choosing cell:%i tile:%i tile_idx:%i entropy:%f\n",
        //    (int)_min_cell, (int)_min_tile, (int)_min_tile_idx, (float)_min_entropy);

        break;
      }
    }

  }

  // ? just pick the first one?
  //
  else {


    _min_tile = getValI( BUF_TILE_IDX, 0, anch_cell );
    _min_tile_idx = 0;

    //DEBUG
    //printf("wfc: chooseMinEntropy: ????? choosing cell:%i tile:%i tile_idx:%i entropy:%f\n",
    //   (int)_min_cell, (int)_min_tile, (int)_min_tile_idx, (float)_min_entropy);


  }

  if (min_cell)     { *min_cell     = _min_cell; }
  if (min_tile)     { *min_tile     = _min_tile; }
  if (min_tile_idx) { *min_tile_idx = _min_tile_idx; }
  if (min_entropy)  { *min_entropy  = _min_entropy; }

  //DEBUG
  //printf("wfc: count: %i\n", count);

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
    SetValF( BUF_BELIEF, 0.0, a );
  }
  Vector3DI jp = getVertexPos(j);

  tile_idx_n= getValI ( BUF_TILE_IDX_N, j );
  for (tile_idx=0; tile_idx<tile_idx_n; tile_idx++) {
    tile_val = getValI ( BUF_TILE_IDX, tile_idx, j );

    _bi = 1.0;
    for (kn=0; kn<getNumNeighbors(j); kn++) {
      k = getNeighbor(j, jp, kn);
      if (k==-1) { continue; }

      _bi *= getValF(BUF_MU, kn, tile_val, j);
    }
    SetValF(BUF_BELIEF, _bi, tile_val );

    sum += _bi;
  }

  if (sum > op.eps_zero) {
    for (tile_idx=0; tile_idx<tile_idx_n; tile_idx++) {
      tile_val = getValI( BUF_TILE_IDX, tile_idx, j );

      _bi = getValF ( BUF_BELIEF, tile_val ) / sum;
      SetValF( BUF_BELIEF, _bi, tile_val );
    }
  }

  return sum;
}

float BeliefPropagation::_getVertexBelief ( uint64_t j ) {
  int64_t k;
  int a, kn;
  float sum = 0;
  float _bi = 1.0;

  Vector3DI jp = getVertexPos(j);

  for (a=0; a < m_num_values; a++) {
    SetValF( BUF_BELIEF, 1.0, a );

    for (kn=0; kn < getNumNeighbors(j); kn++) {
      k = getNeighbor(j, jp, kn);
      if (k!=-1) {

        // mu{k,j}(a)
        //
        _bi = getValF( BUF_BELIEF, a) * getValF(BUF_MU, kn, a, j );
        SetValF( BUF_BELIEF, _bi, a );
      }
    }
    sum += getValF( BUF_BELIEF, a );
  }
  if ( sum > 0 ) {
    for (a=0; a < m_num_values; a++) {
      _bi = getValF( BUF_BELIEF, a ) / sum;
      SetValF( BUF_BELIEF, _bi, a  );
    }
  }

  return sum;
}


int BeliefPropagation::start () {

  int ret=0;

  m_rand.seed ( op.seed );

  int v = m_rand.randI();  // first random # (used as spot check)

  if (op.verbose >= VB_STEP ) {
    printf ("  Started. grid %d,%d,%d, seed = %d\n", op.X, op.Y, op.Z, op.seed );
  }

  op.seed++;

  // reset stats (must do first)
  ResetStats ();

  // rebuild dynamic bufs
  //
  ConstructDynamicBufs ();

  RandomizeMU ();

  // debugInspect (Vector3DI(1,1,0), 0 );

  // cull boundary
  //
  ret = CullBoundary();

  // requires tileidx filled (in DynamicBufs)
  //
  NormalizeMU ();

  if ( op.alg_accel==ALG_ACCEL_WAVE) {
    InitializeDMU ();
  }

  // caller should remove constrained tiles
  // right after this func
  //
  return ret;
}

void BeliefPropagation::ResetStats () {

    op.cur_iter = 0;
    st.elapsed_time = 0;
    st.iter_resolved = 0;
    st.total_resolved = 0;
    st.post = 1;

    st.time_boundary = 0;
    st.time_normalize = 0;
    st.time_bp = 0;
    st.time_viz = 0;
    st.time_maxdiff = 0;
    st.time_updatemu = 0;
}


//----

int BeliefPropagation::filter_constraint(std::vector< std::vector< int32_t > > &constraint_list) {
  int i;
  int32_t tile_id, x, y, z, n;
  int64_t pos;

  for (pos=0; pos<m_num_verts; pos++) {
    SetValI( BUF_TILE_IDX_N, 0, pos );
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

    n = getValI( BUF_TILE_IDX_N, pos );

    SetValI( BUF_TILE_IDX, n, tile_id, pos );
    n++;
    SetValI( BUF_TILE_IDX_N, n, pos );

  }

  return 0;
}


void BeliefPropagation::reset () {

    // reset clears all memory buffers
    // allowing a full rebuild of bp from init.

    // erase all buffers
    for (int n=0; n < BUF_MAX; n++)
        m_buf[ n ].Clear ();

    // reset stats
    ResetStats();

    // do not erase ops as they
    // may be needed for re-init
}

void BeliefPropagation::init_dir_desc() {

  m_dir_desc.push_back("+1:0:0");
  m_dir_desc.push_back("-1:0:0");
  m_dir_desc.push_back("0:+1:0");
  m_dir_desc.push_back("0:-1:0");
  m_dir_desc.push_back("0:0:+1");
  m_dir_desc.push_back("0:0:-1");

}

int BeliefPropagation::init(

  int Rx, int Ry, int Rz,
  std::vector< std::string  >           tile_name_list,
  std::vector< float >                  tile_weight_list,
  std::vector< std::vector < float > >  tile_rule_list ) {

  int i, ret=0,
      b, maxb=-1;

  init_dir_desc();

  m_dir_inv[0] = 1;
  m_dir_inv[1] = 0;
  m_dir_inv[2] = 3;
  m_dir_inv[3] = 2;
  m_dir_inv[4] = 5;
  m_dir_inv[5] = 4;


  //---

  m_tile_name.clear();
  for (i=0; i<tile_name_list.size(); i++) {
    m_tile_name.push_back( tile_name_list[i] );
  }

  // use tile_rule_list to determine maximum number of tiles
  //
  for (i=0; i<tile_rule_list.size(); i++) {
    b = (int)tile_rule_list[i][0];
    if (b>maxb) { maxb = b; }
    b = (int)tile_rule_list[i][1];
    if (b>maxb) { maxb = b; }
  }
  m_num_values = maxb+1;

  //---

  //-- Construct static buffers & populate them
  //
  ConstructStaticBufs ();

  // populate F - tile rules
  for (int i=0; i < tile_rule_list.size(); i++) {
    SetValF( BUF_F, (tile_rule_list[i][3]), tile_rule_list[i][0], tile_rule_list[i][1], tile_rule_list[i][2] );
  }
  // populate G - tile weights
  for (i=0; i<m_num_values; i++) {
    if (i < tile_weight_list.size()) {
      SetValF( BUF_G, tile_weight_list[i], i );
    }
  }

  // Prepare problem size
  //

  m_bpres.Set ( Rx, Ry, Rz );
  m_num_verts = m_bpres.x * m_bpres.y * m_bpres.z;
  m_num_values = m_tile_name.size();
  m_res.Set ( Rx, Ry, Rz );


  // Neighbor lookups
  if ( m_bpres.x > 512 || m_bpres.y > 512 || m_bpres.z > 512 ) {
      printf ( "ERROR: Neighbor lookup limit. Must also modify NOUT.\n" );
      exit(-17);
  }
  for (int nbr=0; nbr < 6; nbr++) {
    for (int i=0; i < std::max(m_bpres.x, std::max(m_bpres.y, m_bpres.z)); i++) {
        switch (nbr) {
        case 0: nbr_lookup[nbr][i] = (i < m_bpres.x-1) ?     1 : NOUT;                      break;
        case 1: nbr_lookup[nbr][i] = (i > 0) ?              -1 : NOUT;                      break;
        case 2: nbr_lookup[nbr][i] = (i < m_bpres.y-1) ?    +m_bpres.x : NOUT;              break;
        case 3: nbr_lookup[nbr][i] = (i > 0) ?              -m_bpres.x : NOUT;              break;
        case 4: nbr_lookup[nbr][i] = (i < m_bpres.z-1) ?    +(m_bpres.x*m_bpres.y) : NOUT;  break;
        case 5: nbr_lookup[nbr][i] = (i > 0) ?              -(m_bpres.x*m_bpres.y) : NOUT;  break;
        };
    }
  }



  //-- Construct temp buffers
  //
  ConstructTempBufs ();

  //-- Reset dynamic buffers
  // MU, MU_NXT, TILE_IDX
  //

  // rebuild dynamic bufs
  //
  ConstructDynamicBufs ();
  RandomizeMU ();

  // options
  //
  op.use_cuda  = false;

  if (op.use_svd) {

    // m_num_values x m_num_values is an upper bound
    // ont he matrix size. The dimensions used will
    // be m_num_values x d and d x m_num_values for
    // U and V respectivley.
    //
    ConstructSVDBufs ();

    init_SVD();
  }

  // set max iterations
  //
  op.cur_iter = 0;
  op.max_iter = m_num_verts;

  return 0;
}


//----

// assumes BUF_F already initailized
//
int BeliefPropagation::init_SVD(void) {
  int i, j, n;
  int r, c;
  int idir=0;

  float _eps = 1.0/(1024.0);
  float s=0.0;

  Eigen::MatrixXf M(m_num_values, m_num_values), S, U, V;
  Eigen::JacobiSVD<Eigen::MatrixXf> svd;

  n = m_num_values;
  for (idir=0; idir<6; idir++) {

    for (i=0; i<n; i++) {
      for (j=0; j<n; j++) {
        M(i,j) = getValF( BUF_F, i, j, idir );
      }
    }

    svd.compute(M, Eigen::ComputeThinV | Eigen::ComputeThinU );

    S = svd.singularValues();
    U = svd.matrixU();
    V = svd.matrixV();

    m_svd_nsv[idir] = 0;
    for (i=0; i<S.rows(); i++) {

      if ( fabs(S(i,0)) < _eps ) { break; }
      m_svd_nsv[idir]++;
    }


    // U * (S * V^t)
    //
    for (r=0; r<n; r++) {
      for (c=0; c<m_svd_nsv[idir]; c++) {
        SetValF( BUF_SVD_U, r, c, idir, U(r,c));
      }
    }

    for (r=0; r<m_svd_nsv[idir]; r++) {
      for (c=0; c<n; c++) {
        SetValF( BUF_SVD_Vt, r, c, idir, S(r,0)*V(c,r));
      }
    }

    // CHECK
    /*
    for (r=0; r<m_num_values; r++) {
      for (c=0; c<m_num_values; c++) {
        s = 0.0;
        for (k=0; k<m_num_values; k++) {
          s += getValF( BUF_SVD_U, r, k, idir ) * getValF( BUF_SVD_Vt, k, c, idir );
        }

        printf("[%i,%i,%i]: %f %f (%f) %c\n",
            r,c, idir,
            getValF( BUF_F, r, c, idir ),
            s,
            abs(getValF( BUF_F, r, c, idir ) - s),
            (abs(getValF( BUF_F, r, c, idir ) - s) < _eps) ? ' ' : '!');

      }
    }
    */

  }

  return 0;
}

//---

int BeliefPropagation::wfc() {

  int ret = 1;
  int64_t it;

  ret = wfc_start();
  if (ret < 0) {
    printf("ERROR: start failed (%i)\n", ret);
    return ret;
  }

  //DEBUG
  //printf("wfc: wfc_start got %i\n", (int)ret);


  for (it = 0; it < m_num_verts; it++) {

    //DEBUG
    //printf("wfc: it[%i/%i]\n", (int)it, (int)m_num_verts);

    ret = wfc_step ( it );

    //printf("wfc: it[%i/%i] wfc_step got %i\n", (int)it, (int)m_num_verts, (int)ret);

    if ( ret==0 ) { break; }

    if ( ret < 0) {
      switch (ret) {
      case -1: printf ( "wfc chooseMaxBelief error.\n" ); break;
      case -2: printf ( "wfc tileIdxCollapse error.\n" ); break;
      case -3: printf ( "wfc cellConstraintPropagate error.\n" ); break;
      };
      break;
    }
  }

  //printf("wfc: done (%i)\n", (int)ret);

  return ret;
}

int BeliefPropagation::wfc_start() {
  int ret=0;

  //DEBUG
  //printf("wfc_start: seed %i\n", (int)op.seed);

  m_rand.seed ( op.seed );

  ret = CullBoundary();
  return ret;
}

int BeliefPropagation::wfc_step(int64_t it) {

  int ret;
  int64_t cell=-1;
  int32_t tile=-1, tile_idx=-1;
  float entropy=-1.0, d = -1.0;

  int64_t step_iter=0;

  ret = chooseMinEntropy( &cell, &tile, &tile_idx, &entropy);
  if (ret < 0) { return -1; }
  if (ret==0) { return 0; }

  if (op.verbose >= VB_STEP ) {
    printf("wfc[%i]: cell:%i, tile:%i, tile_idx:%i, entropy:%f (ret:%i)\n",
      (int)it, (int)cell, (int)tile, (int)tile_idx, (float)entropy, (int)ret);
  }

  ret = tileIdxCollapse( cell, tile_idx );
  if (ret < 0) { return -2; }

  m_note_n[ m_note_plane ] = 0;
  m_note_n[ 1 - m_note_plane ] = 0;

  cellFillVisited(cell, m_note_plane );
  unfillVisited( m_note_plane );

  //ret = cellConstraintPropagate(cell);
  //int resolved;

  ret = cellConstraintPropagate();
  if (ret < 0) { return -3; }

  return 1;
}

//-------

//  0 - success
// -1 - error
//
int BeliefPropagation::RealizePre(void) {

  int64_t cell=-1;
  int32_t tile=-1, tile_idx=-1, n_idx=-1;
  float belief=-1.0, d = -1.0;

  // reset steps
  op.cur_step = 0;

  float _eps = getLinearEps();

  if (op.alg_run_opt == ALG_RUN_VANILLA) {

    // after we've propagated constraints, BUF_MU
    // needs to be renormalized
    //
    WriteBoundaryMUbuf(BUF_MU);
    NormalizeMU(BUF_MU);

    if (op.verbose >= VB_INTRASTEP ) {
      printf("# RealizePre %f (%i/%i) {%f:%f}\n",
          (float) _eps, (int) op.cur_iter, (int) op.max_iter,
          (float) op.eps_converge_beg, (float) op.eps_converge_end);
    }

  }

  else if (op.alg_run_opt == ALG_RUN_RESIDUAL) {

    // after we've propagated constraints, BUF_MU
    // needs to be renormalized
    //
    WriteBoundaryMUbuf(BUF_MU);
    WriteBoundaryMUbuf(BUF_MU_NXT);

    NormalizeMU(BUF_MU);
    NormalizeMU(BUF_MU_NXT);

    // first do a whole sweep, updating MU_NXT and keeping
    // the values there. An initial pass has to be done (step(1))
    // to make sure boundary conditions are populated and transferred
    // over correcly.
    //
    // populate the indexHeap: priority queue with maximum
    // difference of MU and MU_NXT as heap key in additon
    // to keeping the "mu index" (position in MU buffer)
    //
    // From this point forward, updates to BUF_MU or BUF_MU_NXT
    // will need a corresponding bookeeping call to indexXHeap
    // to keep track of the maximum difference between
    // the two buffers and corresponding cell index information.
    //
    d = step(1);
    d = step(0);
    indexHeap_init();

  }

  return 0;
}

//  0 - success and finish
//  1 - continuation
// -1 - error
//
int BeliefPropagation::RealizePost(void) {

  int resolved = 0;
  int post_ret=0;
  int ret;
  int64_t cell=-1;
  int32_t tile=-1, tile_idx=-1, n_idx=-1;
  float belief=-1.0, d = -1.0;

  Vector3DI vp;

  // choose the cell and propagate choice
  //
  switch (op.alg_cell_opt) {
    case ALG_CELL_ANY:
      ret = chooseMaxBelief( &cell, &tile, &tile_idx, &belief );
      break;
    case ALG_CELL_MIN_ENTROPY:
      ret = chooseMinEntropyMaxBelief( &cell, &tile, &tile_idx, &belief );
      break;
    default:
      return -1;
      break;
  }

  // result of choose..
  //
  if ( ret >= 1 ) {

    // 1 = continue condition.
    // display chosen cell
    //
    if (op.verbose >= VB_INTRASTEP ) {
      vp = getVertexPos(cell);
      n_idx = getValI ( BUF_TILE_IDX_N, cell );
      printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i, belief:%f (tile_idx:%i / %i) [rp]\n",
          (int)op.cur_iter,
          (int)cell,
          (int)vp.x, (int)vp.y, (int)vp.z,
          (int)tile, (float)belief, (int)tile_idx, (int)n_idx);
    }

    // assume continue
    //
    post_ret = 1;

    // reset iter resolved and advance total
    //
    st.iter_resolved = 1;
    st.total_resolved++;

    // collapse
    //
    ret = tileIdxCollapse( cell, tile_idx );
    if (ret >=0 ) {
      m_note_n[ m_note_plane ] = 0;
      m_note_n[ 1 - m_note_plane  ] = 0;

      cellFillVisited ( cell, m_note_plane );
      unfillVisited( m_note_plane  );

      // propagate constraints to remove neighbor tiles,
      // and count number resolved (only 1 tile val remain)
      //
      ret = cellConstraintPropagate();

      // constraint prop failed
      //
      if (ret < 0) { post_ret = -3; }

    }

    // collapse failed
    //
    else { post_ret = -2; }

  }

  // 0 = success condition.
  // -1 = failure of MaxBelief
  //
  else if ( ret == 0 ) { post_ret = 0; }
  else if ( ret < 0)   { post_ret = -1; }

  // iteration or overall complete
  //
  if (post_ret >= 0) {

    if (st.enabled) {
      // check constraints
      st.constraints = CheckConstraints ();
    }
    st.eps_curr = getLinearEps();
    st.post = post_ret;

    // print iter stats
    //
    if (post_ret==1 && op.verbose >= VB_STEP ) {
      printf ("%s", getStatMessage().c_str() );
    }
    // print run completion
    if (post_ret==0 && op.verbose >= VB_RUN ) {
      printf ("%s", getStatMessage().c_str() );
    }
    
    op.cur_iter++;
  }

  return post_ret;
}

std::string BeliefPropagation::getStatMessage () {

  char msg[1024] = {0};

  snprintf ( msg, 1024, 
             "  %s: %d/%d, Iter: %d, %4.1fmsec, constr:%d, resolved: %d/%d/%d (%4.1f%%), steps %d/%d, max.dmu %f, eps %f, av.mu %1.5f, av.dmu %1.8f\n",
              ((st.post==1) ? "RUN" :
                ((st.constraints==0) ? "RUN_SUCCESS" : "RUN_FAIL")),
              op.cur_run, op.max_run, op.cur_iter, 
              st.elapsed_time, (int)st.constraints, 
              st.iter_resolved, (int)st.total_resolved, op.max_iter, 100.0*float(st.total_resolved)/op.max_iter, 
              op.cur_step, op.max_step, 
              st.max_dmu, st.eps_curr, st.ave_mu, st.ave_dmu );
  
  // b:%f, n:%f, bp:%f, v:%f, md:%f, u:%f
  // st.time_boundary, st.time_normalize, st.time_bp, st.time_viz, st.time_maxdiff, st.time_updatemu );

  return msg;
}

std::string BeliefPropagation::getStatCSV (int mode) {

  char msg[1024] = {0};
  int status;
  status = (st.post==1) ? 0 : (st.constraints==0) ? 1 : -1;

  snprintf ( msg, 1024, 
      "%d, %d, %d, %4.1f, %d, %d, %d, %d, %4.1f%%, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",  
      op.cur_run, status, op.cur_iter, st.elapsed_time,
      (int)st.constraints, st.iter_resolved, st.total_resolved, op.max_iter, 100.0*float(st.total_resolved)/op.max_iter, 
      op.cur_step, op.max_step, st.max_dmu, st.eps_curr,
      st.ave_mu, st.ave_dmu,
      st.time_boundary, st.time_normalize, st.time_bp, st.time_viz, st.time_maxdiff, st.time_updatemu );                   

  return msg;
}


float BeliefPropagation::getLinearEps () {

  // linear interpolation epsilon
  //
  float _eps = op.eps_converge;

  _eps = op.eps_converge_beg + ((op.eps_converge_end - op.eps_converge_beg) * float(op.cur_iter) / float(op.max_iter) );

  return _eps;
}


// 0 - converged
// 1 - max iter reached
//
int BeliefPropagation::RealizeIter (void) {
  int ret = 1;
  float d = -1.0;

  float _eps = getLinearEps();

  // iterate bp until converged
  //
  for (op.cur_step=0; op.cur_step < op.max_step; op.cur_step++) {

    d = step(MU_COPY);

    if (fabs(d) < _eps) {
      ret = 0;
      break;
    }
  }

  return ret;
}

// 0 - aconverged
// 1 - not converged yet
//
int BeliefPropagation::RealizeStep(void) {

  float d;

  int64_t mu_idx, cell;
  float f_residue;
  int32_t idir, tile;

  // start timing
  //
  clock_t m_t1 = clock();

  // get linear interpolation eps
  //
  float _eps = getLinearEps();

  // assume continue
  //
  int ret = 1;

  //---

  if (op.alg_run_opt == ALG_RUN_VANILLA) {
    d = step(MU_COPY);
    if (fabs(d) < _eps) { ret = 0; }
  }

  //---

  else if (op.alg_run_opt == ALG_RUN_RESIDUAL) {

    mu_idx = indexHeap_peek_mu_pos( &idir, &cell, &tile, &f_residue);

    if (f_residue < _eps) { ret = 0; }
    else {
      if (op.verbose >= VB_INTRASTEP ) {
        printf("  [it:%i,step:%i] updating mu[%i,%i,%i](%i) residue:%f\n",
            (int) op.cur_iter, (int) op.cur_step,
            (int) idir, (int)cell, (int)tile, (int)mu_idx, (float)f_residue);
      }

      step_residue( idir, cell, tile );
    }

  }

  //--- unknown algorithm

  else { ret = -1; }

  // complete timing
  //
  clock_t m_t2 = clock();
  st.elapsed_time += ((((double) m_t2-m_t1) / CLOCKS_PER_SEC) * 1000);

  if ( ret==1 ) {
    if (op.cur_step >= op.max_step )  { ret = -2; }
    else                              { op.cur_step++; }
  }

  return ret;
}

// Example of a full realization, running until completion
//
int BeliefPropagation::Realize(void) {

  int ret=-1;

  ret = start();
  if (ret<0) { return ret; }

  op.max_iter = m_num_verts;

  for (op.cur_iter = 0; op.cur_iter < op.max_iter; op.cur_iter++) {

    ret = RealizePre();
    if (ret < 0) { break; }

    ret = 1;
    while (ret>0) {
      ret = RealizeStep();
    }

    ret = RealizePost();
    if (ret <= 0) { break; }

  }

  return ret;
}

int BeliefPropagation::CheckConstraints () {

  // compute and store the max belief tile for each vertex
  //
  ComputeBeliefField ();

  // compute the unresolved constraints at each vertex
  //
  int cnt = 0;

  for (int64_t vtx = 0; vtx < m_num_verts; vtx++) {
    cnt += CheckConstraints ( vtx );
  }

  return cnt;
}


int BeliefPropagation::ComputeTilecountField () {

    int sum = 0;
    int i;

    for (int64_t vtx = 0; vtx < m_num_verts; vtx++) {

        i = getValI ( BUF_TILE_IDX_N, vtx );

        sum += i;

        SetValF ( BUF_VIZ, 1.0 / i, vtx );        
    }

    return sum;
}



int BeliefPropagation::CheckConstraints ( int64_t vtx ) {

  int a, b, c, cnt;
  float rule;
  int64_t pnbr, f;
  Vector3DI pi;

  // tile value at p
  //
  a = getValI ( BUF_TILES, vtx );

  cnt = 0;
  for (int nbr=0; nbr < 6; nbr++) {

    // tile value at neighbor of p
    //
    pnbr = getNeighbor(vtx, nbr);
    if ( pnbr != -1) {

      b = getValI( BUF_TILES, pnbr);

       // check rule for b->a
       //
      rule = getValF ( BUF_F, a, b, nbr );

      // rule: weight 0 = disallowed
      //
      if (rule==0) {
        cnt++;
      }

    } else {
      b = 0;
    }

  }

  SetValI ( BUF_C, cnt, vtx );

  return cnt;
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

    anch_tile_idx_n = getValI ( BUF_TILE_IDX_N, anch_cell );
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI ( BUF_TILE_IDX, anch_tile_idx, anch_cell );

      b = getValF( BUF_BELIEF, anch_tile );
      if (anch_tile_idx==0) { max_belief = b; }
      if (b>max_belief)     { max_belief = b; }
    }
    printf("#gp: %i %i %i %f\n", (int)pos.x, (int)pos.y, (int)pos.z, (float)max_belief);
  }
}


float BeliefPropagation::step (int update_mu) {
  
  float max_diff = -1.0;
  int m_calc_residue = 1;

  clock_t t1, t2;

  // initial boundary condiitions
  //
  #ifdef OPT_MUBOUND
    if (st.instr) t1 = clock();
    WriteBoundaryMUbuf(BUF_MU);      
    //WriteBoundaryMUbuf(BUF_MU_NXT);  //-- not necessary i believe, BeliefProp will overwrite buf_mu_nxt
    if (st.instr) {st.time_boundary += clock()-t1;}

    if (st.instr) t1 = clock();
    NormalizeMU( BUF_MU );
    if (st.instr) {st.time_normalize += clock()-t1;}
  #endif


  // run main bp, store in BUF_MU_NXT
  //
  if (st.instr) t1 = clock();

  if (op.use_svd) { BeliefProp_svd(); }
  else            { BeliefProp(); }

  if (st.instr) {st.time_bp += clock()-t1;}

  // renormalize BUF_MU_NXT
  //
  NormalizeMU( BUF_MU_NXT );

  // visualize before updateMU
  //
  if (st.instr) t1 = clock();
  if ( op.viz_opt == VIZ_DMU ) {
    ComputeDiffMUField ();
  }

  if ( op.viz_opt == VIZ_BELIEF ) {    
    ComputeBeliefField ();
  }
  if ( op.viz_opt == VIZ_CONSTRAINT || op.viz_opt == VIZ_TILECOUNT ) {
    
    // ComputesBeliefField first. see CheckConstraints
    CheckConstraints ();   
    ComputeTilecountField ();
  }
  if (st.instr) {st.time_viz += clock()-t1;}


  // calculate the difference between
  // BUF_MU_NXT and BUF_MU
  //
  if (st.instr) t1 = clock();
  max_diff = MaxDiffMU();
  if (st.instr) {st.time_maxdiff += clock()-t1;}

  st.max_dmu = max_diff;

  if (update_mu) {

    // BUF_MU <- BUF_MU_NXT
    //
    if (st.instr) t1 = clock();
    UpdateMU();
    if (st.instr) {st.time_updatemu += clock()-t1;}
  }

  return max_diff;
}

// !!!WIP!!!!
//
// idir, cell, tile should be the value to update in the MU buf.
// That is:
//
//   MU[idir][cell][tile] = MU_NXT[idir][cell][tile]
//
// Once that's done, update all neighboring values that would be affected by this change,
// updating MU_NXT with the new values but also the indexHeap structure to do the
// boookeeping to make sure the maximum |MU-MU_NXT| can be fetched.
//
//
float BeliefPropagation::step_residue(int32_t idir, int64_t cell, int32_t tile) {
  int64_t nei_cell=-1;
  int64_t dir_idx=-1;
  float mu_new = -1.0,
        mu_cur_val,
        mu_nxt_val;
  int32_t _t;
  int64_t n_tile_idx, tile_idx;

  // update BUF_MU with new value
  //
  mu_new = getValF ( BUF_MU_NXT, idir, tile, cell );
  SetValF( BUF_MU, mu_new, idir, tile, cell );

  // the single update to cell:tile requires a
  // renormalization, which can alter all values
  // in the cell, requiring an update to the
  // priority queue.
  //

  NormalizeMU_cell_residue( BUF_MU, cell );

  // run bp on neighbors potentially affected by the updated current
  // cell.
  //
  for (dir_idx=0; dir_idx < getNumNeighbors(cell); dir_idx++) {

    nei_cell = getNeighbor( cell, dir_idx );
    if (nei_cell < 0) { continue; }

    if (op.use_svd) { BeliefProp_cell_residue_svd(nei_cell); }
    else            { BeliefProp_cell_residue(nei_cell); }

    NormalizeMU_cell_residue( BUF_MU_NXT, nei_cell );
  }

  return -1.0;
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
int BeliefPropagation::filterKeep(uint64_t pos, std::vector<int32_t> &tile_id) {
  int32_t tile_idx,
          idx,
          n,
          tile_val,
          tv;

  n = getValI( BUF_TILE_IDX_N, pos );

  int nstart = n;

  for (idx=0; idx<n; idx++) {

    if (n <= 0) { break; }

    tile_val = getValI( BUF_TILE_IDX, idx, pos );

    for (tile_idx=0; tile_idx<tile_id.size(); tile_idx++) {
      if (tile_id[tile_idx] == tile_val) { break; }
    }
    if (tile_idx < tile_id.size()) { continue; }

    n--;
    tv = getValI( BUF_TILE_IDX, n, pos );
    SetValI( BUF_TILE_IDX, tile_val,  n,    pos );
    SetValI( BUF_TILE_IDX, tv,        idx,  pos );

    SetValI( BUF_TILE_IDX_N, n, pos  );

    idx--;
  }

  if ((n==1) && (n != nstart)) {
    st.iter_resolved++;
    st.total_resolved++;

    if (op.verbose >= VB_INTRASTEP ) {
      Vector3DI vp;
      vp = getVertexPos(pos);

      tile_val = getValI( BUF_TILE_IDX, 0, pos );
      printf("RESOLVE it:-1 cell:%i;[%i,%i,%i] tile:%i [fk]\n",
          (int)pos,
          (int)vp.x, (int)vp.y, (int)vp.z,
          (int)tile_val);
    }

  }

  if (n<=0) { return -1; }
  return 0;
}

// Discard tile entreis at cell positoin `pos`
//
int BeliefPropagation::filterDiscard(uint64_t pos, std::vector<int32_t> &tile_id) {
  int32_t tile_idx,
          idx,
          n,
          tile_val,
          tv;

  n = getValI( BUF_TILE_IDX_N, pos );
  int nstart = n;

  for (idx=0; idx<n; idx++) {

    if (n <= 0) { break; }

    tile_val = getValI( BUF_TILE_IDX, idx, pos );

    for (tile_idx=0; tile_idx<tile_id.size(); tile_idx++) {
      if (tile_id[tile_idx] == tile_val) { break; }
    }
    if (tile_idx==tile_id.size()) { continue; }

    n--;
    tv = getValI( BUF_TILE_IDX, n, pos );
    SetValI( BUF_TILE_IDX, tile_val, n, pos );
    SetValI( BUF_TILE_IDX, tv, idx, pos);

    SetValI( BUF_TILE_IDX_N, n, pos  );

    idx--;
  }

  if (n==1 && n != nstart) {
    st.iter_resolved++;
    st.total_resolved++;

    if (op.verbose >= VB_INTRASTEP ) {
      Vector3DI vp;
      vp = getVertexPos(pos);

      tile_val = getValI( BUF_TILE_IDX, 0, pos );
      printf("RESOLVE it:-1 cell:%i;[%i,%i,%i] tile:%i [fd]\n",
          (int)pos,
          (int)vp.x, (int)vp.y, (int)vp.z,
          (int)tile_val);
    }

  }

  if (n<=0) { return -1; }
  return 0;
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

  std::string tile_name = cs;
  return tileName2ID(tile_name);
}

/* void BeliefPropagation::UpdateRunTimeStat(int64_t num_step) {

  op.cur_step++;

  st.avg_step =
    ((((double)( op.cur_step-1)) * st.avg_step) + ((double) op.cur_step)) / ((double) op.cur_step);

  if (op.cur_step > st.upper_step) {
    st.upper_step = op.cur_step;
  }
} */


// print out state of BUF_NOTE, BUF_VISITED

void BeliefPropagation::debugInspect (Vector3DI pos, int tile) {

  int64_t vtx = getVertex(pos.x, pos.y, pos.z);
  int n, i, b;
  int sz = 8;

  printf ( "---------- Inspect: %d,%d,%d -> vtx: %d\n", (int)pos.x, (int)pos.y, (int)pos.z, (int)vtx );

  int valmax = fmin( sz, m_num_values );

  // inspect
  //
  printf ("BUF_F:  %d->{..} ", (int)BUF_F);
  for (n=0; n < 6; n++) {
    printf ("%d: ", (int)n );
    for (b=0; b < valmax; b++) { printf ("%f ", (float)getValF(BUF_F, tile, b, n)); }
    printf ("\n");
  }
  printf ("BUF_MU: %d->6nbr (%d):\n", (int)vtx, (int)tile );
  for (n=0; n < 6; n++) {
    printf ("%d: ",(int) n );
    for (i=0; i < sz; i++) { printf ("%f ", (float)getValF(BUF_MU, n, tile, vtx+i )); }
    printf ("\n" );
  }
  printf ("BUF_TILE_IDX_N: %d.. ", (int)vtx ); for (i=0; i < sz; i++) { printf ("%d ", (int)getValI(BUF_TILE_IDX_N, vtx+i )); }
  printf ("\nBUF_TILE_IDX: @%d= ", (int)vtx ); for (i=0; i < sz; i++) { printf ("%d ", (int)getValI(BUF_TILE_IDX, i, vtx+i )); }
  printf ("\nBUF_NOTE: %d.. ", (int)vtx ); for (i=0; i < sz; i++) { printf ("%d ", (int)getValI(BUF_NOTE, vtx+i )); }
  printf ("\n\n");

}

// print out state of BUF_NOTE, BUF_VISITED
//
void BeliefPropagation::debugPrintC() {

  int i, n, fold = 20, m;

  printf("NOTE[%i][%i]", (int) m_note_plane , (int)m_note_n[ m_note_plane  ]);
  for (m=0; m<2; m++) {
    n = m_note_n[m];
    for (i=0; i<n; i++) {
      if ((i%fold)==0) { printf("\n"); }
      printf(" %i", (int) getValL ( BUF_NOTE, i, m));
    }
    printf("\n");
  }

  n = m_num_verts;
  printf("VISITED[%i]\n", (int)m_num_verts);
  for (i=0; i<n; i++) {
    if ((i>0) && ((i%fold)==0)) { printf("\n"); }
    printf(" %i", (int)getValL ( BUF_VISITED, i ));
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

  int i=0, j=0, n=3, m=7, jnbr=0, a=0;
  int a_idx=0, a_idx_n=0;
  int64_t u=0;
  Vector3DI p;
  double v=0.0;
  float _vf = 0.0, f, _eps;

  int __a = 0;

  int64_t max_cell=-1;
  int32_t max_tile=-1, max_tile_idx=-1;
  float max_belief=-1.0;
  int count=-1;

  int print_rule = 0;

  _eps = op.eps_zero;

  std::vector< std::string > _dp_desc;

  _dp_desc.push_back("+1:0:0");
  _dp_desc.push_back("-1:0:0");
  _dp_desc.push_back("0:+1:0");
  _dp_desc.push_back("0:-1:0");
  _dp_desc.push_back("0:0:+1");
  _dp_desc.push_back("0:0:-1");

  printf("bp version: %s\n", BELIEF_PROPAGATION_VERSION);
  printf("op.verbose: %i\n", op.verbose);

  printf("res: (%i,%i,%i)\n", m_res.x, m_res.y, m_res.z);
  printf("bpres: (%i,%i,%i)\n", m_bpres.x, m_bpres.y, m_bpres.z);
  printf("num_verts: %i, m_num_values: %i\n", (int)m_num_verts, (int)m_num_values);
  printf("run_cuda: %i, op.use_svd: %i, op.use_checkerboard: %i\n",
      (int) op.use_cuda, (int) op.use_svd, (int) op.use_checkerboard);
  printf("eps_converge: %f, eps_zero: %f, rate: %f, max_step: %i, seed: %i\n",
      (float) op.eps_converge, (float) op.eps_zero,
      (float) op.step_rate, (int) op.max_step,
      (int) op.seed);
  printf("stat_enabled: %i\n", (int) st.enabled);

  if (st.enabled) {
    float f_ele = (float) (m_num_values * m_num_verts);
    printf("stats: max_step: %i, avg_step: %f\n", (int) op.max_step, (float) st.avg_step);
    printf("stats: num_culled: %i (density %f), num_collapsed: %i (density %f)\n",
        (int) st.num_culled, (float) st.num_culled / f_ele,
        (int) st.num_collapsed, (float) st.num_collapsed / f_ele);
  }

  printf("m_tile_name[%i]:\n", (int)m_tile_name.size());
  for (i=0; i < m_tile_name.size(); i++) {
    if ((i%m)==0) { printf("\n"); }
    v = getValF( BUF_G, i );
    printf(" %s(%2i):%0.4f)", m_tile_name[i].c_str(), i, (float)v);
  }
  printf("\n\n");

  //---
  /*
  for (jnbr=0; jnbr<6; jnbr++) {
    for (i=0; i<m_num_values; i++) {
      for (j=0; j<m_num_values; j++) {
        f = getVal(BUF_F, i, j, jnbr);
      }
    }
  }
  */

  if (print_rule) {
    for (jnbr=0; jnbr<6; jnbr++) {
      printf("dir[%i]:\n", jnbr);

      for (i=0; i<m_num_values; i++) {
        printf(" ");
        for (j=0; j<m_num_values; j++) {
          f = getValF(BUF_F, i, j, jnbr);
          if (f > _eps) {
            printf(" %5.2f", (float)getValF(BUF_F, i, j, jnbr));
          }
          else {
            printf("      ");
          }
        }
        printf("\n");
      }
      printf("\n");
    }
  }

  //---

  for (u=0; u<m_num_verts; u++) {
    p = getVertexPos(u);

    a_idx_n = getValI( BUF_TILE_IDX_N, u );

    cellUpdateBelief(u);

    printf("[%i,%i,%i](%i):\n", (int)p.x, (int)p.y, (int)p.z, (int)u);
    for (a_idx=0; a_idx<a_idx_n; a_idx++) {
      a = getValI( BUF_TILE_IDX, (int)a_idx, (int)u );

      __a = tileName2ID( m_tile_name[a] );

      printf("  %s(%2i): ", m_tile_name[a].c_str(), a);
      //printf("  %s(%i,%i): ", m_tile_name[a].c_str(), a, __a);

      for (jnbr=0; jnbr<getNumNeighbors(u); jnbr++) {
        v = getValF ( BUF_MU, jnbr, a, u );
        _vf = (float)v;
        printf(" [%s]", (char *)_dp_desc[jnbr].c_str());
        printf("(%i)", (int)jnbr);
        printf(":%f", (float)_vf);
        //printf(" [%s](%i):%f", (char *)_dp_desc[jnbr].c_str(), (int)jnbr, _vf);
      }

      printf(" {b:%f}", getValF( BUF_BELIEF, a ));

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

void BeliefPropagation::debugPrintMU() {

  int i=0, j=0, n=3, m=7, jnbr=0, a=0;
  int a_idx=0, a_idx_n=0;
  uint64_t u=0;
  Vector3DI p;
  double v=0.0;
  float _vf = 0.0, f, _eps;

  int __a = 0;

  int64_t max_cell=-1;
  int32_t max_tile=-1, max_tile_idx=-1;
  float max_belief=-1.0;
  int count=-1;

  int print_rule = 0;

  _eps = op.eps_zero;

  std::vector< std::string > _dp_desc;

  _dp_desc.push_back("+1:0:0");
  _dp_desc.push_back("-1:0:0");
  _dp_desc.push_back("0:+1:0");
  _dp_desc.push_back("0:-1:0");
  _dp_desc.push_back("0:0:+1");
  _dp_desc.push_back("0:0:-1");

  printf("bp version: %s\n", BELIEF_PROPAGATION_VERSION);
  printf("op.verbose: %i\n", op.verbose);

  printf("m_res: (%i,%i,%i)\n", m_res.x, m_res.y, m_res.z);
  printf("m_bpres: (%i,%i,%i)\n", m_bpres.x, m_bpres.y, m_bpres.z);
  printf("m_num_verts: %i, m_num_values: %i\n", (int)m_num_verts, (int)m_num_values);
  printf("op.use_cuda: %i, op.use_svd: %i, op.use_checkerboard: %i\n",
      (int)op.use_cuda, (int)op.use_svd, (int)op.use_checkerboard);
  printf("m_eps_converge: %f, op.eps_zero: %f, m_rate: %f, m_max_iteration: %i, seed: %i\n",
      (float) op.eps_converge, (float)op.eps_zero,
      (float) op.step_rate, (int) op.max_step,
      (int)op.seed);

  printf("m_tile_name[%i]:\n", (int)m_tile_name.size());
  for (i=0; i<m_tile_name.size(); i++) {
    if ((i%m)==0) { printf("\n"); }
    v = getValF( BUF_G, i );
    printf(" %s(%2i):%0.4f)", m_tile_name[i].c_str(), i, (float)v);
  }
  printf("\n\n");

  if (print_rule) {
    for (jnbr=0; jnbr<6; jnbr++) {
      printf("dir[%i]:\n", jnbr);

      for (i=0; i<m_num_values; i++) {
        printf(" ");
        for (j=0; j<m_num_values; j++) {
          f = getValF(BUF_F, i, j, jnbr);
          if (f > _eps) {
            printf(" %5.2f", (float)getValF(BUF_F, i, j, jnbr));
          }
          else {
            printf("      ");
          }
        }
        printf("\n");
      }
      printf("\n");
    }
  }

  //---

  for (u=0; u<m_num_verts; u++) {
    p = getVertexPos(u);

    a_idx_n = getValI( BUF_TILE_IDX_N, u );

    cellUpdateBelief(u);

    printf("[%i,%i,%i](%i):\n", (int)p.x, (int)p.y, (int)p.z, (int)u);
    for (a_idx=0; a_idx<a_idx_n; a_idx++) {
      a = getValI( BUF_TILE_IDX, (int)a_idx, (int)u );

      __a = tileName2ID( m_tile_name[a] );

      printf("  %s(%2i): mu_cur: ", m_tile_name[a].c_str(), a);

      for (jnbr=0; jnbr<getNumNeighbors(u); jnbr++) {
        v = getValF( BUF_MU, jnbr, a, u );
        _vf = (float)v;
        printf(" [%s]", (char *)_dp_desc[jnbr].c_str());
        printf("(%i)", (int)jnbr);
        printf(":%f", (float)_vf);
      }
      printf("\n");

      printf("  %s(%2i): mu_nxt: ", m_tile_name[a].c_str(), a);

      for (jnbr=0; jnbr<getNumNeighbors(u); jnbr++) {
        v = getValF( BUF_MU_NXT, jnbr, a, u );
        _vf = (float)v;
        printf(" [%s]", (char *)_dp_desc[jnbr].c_str());
        printf("(%i)", (int)jnbr);
        printf(":%f", (float)_vf);
      }
      printf("\n\n");

      //printf(" {b:%f}", getVal( BUF_BELIEF, a ));
      //printf("\n");
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

  int32_t n, tile_val, tv;

  n = getValI( BUF_TILE_IDX_N, pos );
  if (tile_idx >= n) { return -1; }

  tile_val = getValI( BUF_TILE_IDX, tile_idx, pos );
  tv = getValI( BUF_TILE_IDX, 0, pos );
  SetValI( BUF_TILE_IDX, tile_val, 0, pos);
  SetValI( BUF_TILE_IDX, tv, tile_idx, pos);
  SetValI( BUF_TILE_IDX_N, 1, pos );

  if (st.enabled) {
    st.num_collapsed += n-1;
  }

  return 0;
}

int BeliefPropagation::tileIdxRemove(uint64_t pos, int32_t tile_idx) {

  int32_t idx, n, tile_val, tv;

  n = getValI( BUF_TILE_IDX_N, pos );
  if (tile_idx >= n) { return -1; }
  if (n<=1) { return -1; }

  if (op.verbose >= VB_INTRASTEP ) {
    printf("tileIdxRemove before:");
    for (idx=0; idx<n; idx++) {
      printf(" (%i)idx:%i", (int) getValI (BUF_TILE_IDX, idx, pos), (int)idx);
    }
    printf("\n");
  }

  n--;

  tile_val = getValI( BUF_TILE_IDX, tile_idx, pos );
  tv = getValI( BUF_TILE_IDX, 0, pos );
  SetValI( BUF_TILE_IDX, tile_val, 0, pos);
  SetValI( BUF_TILE_IDX, tv, tile_idx, pos);
  SetValI( BUF_TILE_IDX_N, 1, pos );

  if (op.verbose >= VB_INTRASTEP ) {
    printf("tileIdxRemove after:");
    n = getValI( BUF_TILE_IDX_N, pos );
    for (idx=0; idx<n; idx++) {
      printf(" (%i)idx:%i", (int)getValI(BUF_TILE_IDX, idx, pos), (int)idx);
    }
    printf("\n");
  }

  return 0;
}


int BeliefPropagation::CullBoundary() {

  int ret=0;
  int64_t x, y, z, vtx;
  Vector3DI vp;
  int64_t new_vtx_idx;

  // set initial notes

  for (y=0; y<m_res.y; y++) {
    for (z=0; z<m_res.z; z++) {

      vtx = getVertex(0, y, z);
      assert ( vtx < m_num_verts );

      new_vtx_idx = m_note_n[ m_note_plane ];
      SetValL ( BUF_NOTE, (vtx), new_vtx_idx, m_note_plane );
      m_note_n[ m_note_plane ]++;

      if ((m_res.x-1) != 0) {

        vtx = getVertex(m_res.x-1, y, z);
        assert ( vtx < m_num_verts );

        new_vtx_idx = m_note_n[ m_note_plane ];
        SetValL ( BUF_NOTE, (vtx), new_vtx_idx, m_note_plane );
        m_note_n[ m_note_plane ]++;
      }

    }
  }

  for (x=1; x<(m_res.x-1); x++) {
    for (z=0; z<m_res.z; z++) {

      vtx = getVertex(x, 0, z);
      assert ( vtx < m_num_verts );

      new_vtx_idx = m_note_n[ m_note_plane ];
      SetValL ( BUF_NOTE, (vtx), new_vtx_idx, m_note_plane );
      m_note_n[ m_note_plane ]++;

      if ((m_res.y-1) != 0) {

        vtx = getVertex(x, m_res.y-1, z);
        assert ( vtx < m_num_verts );

        new_vtx_idx = m_note_n[ m_note_plane ];
        SetValL ( BUF_NOTE, (vtx), new_vtx_idx, m_note_plane );
        m_note_n[ m_note_plane ]++;
      }

    }
  }

  for (x=1; x<(m_res.x-1); x++) {
    for (y=1; y<(m_res.y-1); y++) {

      vtx = getVertex(x, y, 0);
      assert ( vtx < m_num_verts );

      new_vtx_idx = m_note_n[ m_note_plane ];
      SetValL ( BUF_NOTE, (vtx), new_vtx_idx, m_note_plane );
      m_note_n[ m_note_plane ]++;

      if ((m_res.z-1) != 0) {

        vtx = getVertex(x, y, m_res.z-1);
        assert ( vtx < m_num_verts );

        new_vtx_idx = m_note_n[ m_note_plane ];
        SetValL ( BUF_NOTE, (vtx), new_vtx_idx, m_note_plane );
        m_note_n[ m_note_plane ]++;
      }

    }
  }

  // propagate constraints
  // and cull boundary tile values
  //
  ret = cellConstraintPropagate();

  return ret;
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
// the 'visited' array.
// The 'visited' array has a list of vertices (cell
// positions) that need to be inspected to determine
// if any tiles should be removed.
//
void BeliefPropagation::cellFillVisited(uint64_t vtx, int32_t note_plane ) {

  int64_t i, nei_vtx;

  Vector3DI jp = getVertexPos(vtx);

  for (i=0; i<getNumNeighbors(vtx); i++) {
    nei_vtx  = getNeighbor(vtx, jp, i);
    if (nei_vtx<0) { continue; }
    if (getValL ( BUF_VISITED, nei_vtx ) != 0) { continue; }

    int32_t new_vert_idx = m_note_n [ note_plane ];
    SetValL ( BUF_NOTE, nei_vtx, new_vert_idx, note_plane );
    SetValL ( BUF_VISITED, 1, nei_vtx );
    m_note_n[note_plane]++;
  }

}

int BeliefPropagation::cellFillSingle(uint64_t vtx, int32_t note_plane) {

  if (getValL( BUF_VISITED, vtx ) != 0) { return 0; }

  int32_t new_vert_idx = m_note_n [ note_plane ];
  SetValL ( BUF_NOTE,    vtx, new_vert_idx, note_plane );
  SetValL ( BUF_VISITED, 1, vtx  );
  m_note_n[ note_plane ]++;

  return 1;
}

int BeliefPropagation::getTilesAtVertex ( int64_t vtx ) {

  int a = 0;
  int n_a = getValI ( BUF_TILE_IDX_N, vtx );

  ZeroBuf ( BUF_BELIEF );

  float p = 1.0 / n_a;

  for (int a_idx=0; a_idx<n_a; a_idx++) {
    a = getValI ( BUF_TILE_IDX, a_idx, vtx );
    SetValF ( BUF_BELIEF, p, a );
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
void BeliefPropagation::unfillVisited(int32_t note_idx){
  int64_t i, vtx;

  for (i=0; i < (int64_t) m_note_n[note_idx]; i++) {
    vtx = getValL ( BUF_NOTE, i, note_idx );
    SetValL( BUF_VISITED, 0, vtx );
  }
}

int BeliefPropagation::sanityAccessed() {
  int64_t i;

  for (i=0; i < m_num_verts; i++) {
    if (getValL ( BUF_VISITED, i)!=0) { return -1; }
  }
  return 0;
}


int BeliefPropagation::removeTileIdx (int64_t anch_cell, int32_t anch_tile_idx) {

  int anch_tile_n, anch_tile, last_tile;

  // get tile to be removed
  //
  anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );

  // decrement number of tiles
  //
  anch_tile_n = getValI( BUF_TILE_IDX_N, anch_cell );
  anch_tile_n--;
  if (anch_tile_n==0) { return -1; }

  // swap removed tile with last tile
  //
  last_tile = getValI ( BUF_TILE_IDX, anch_tile_n, anch_cell );
  SetValI( BUF_TILE_IDX, (anch_tile), anch_tile_n, anch_cell );
  SetValI( BUF_TILE_IDX, (last_tile), anch_tile_idx, anch_cell );

  SetValI( BUF_TILE_IDX_N, (anch_tile_n ), anch_cell );

  if (st.enabled) {
    st.num_culled += 1;
  }

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

  float _eps = op.eps_zero;
  Vector3DI jp;

  int resolved = 0;

  while (still_culling) {

    for (note_idx=0; note_idx < (int64_t) m_note_n[ m_note_plane  ]; note_idx++) {

      anch_cell = getValL ( BUF_NOTE, note_idx, m_note_plane  );

      jp = getVertexPos(anch_cell);

      anch_n_tile = getValI ( BUF_TILE_IDX_N, anch_cell );

      for (anch_b_idx=0; anch_b_idx < anch_n_tile; anch_b_idx++) {

        // Test if anchor tile has connection that falls out of bounds.
        // if so, remove tile from BUF_TILE_IDX and add unvisited
        // neighbors to BUF_NOTE for later processing.
        //
        tile_valid = 1;
        anch_b_val = getValI ( BUF_TILE_IDX, anch_b_idx, anch_cell);

        for (i=0; i<getNumNeighbors(anch_cell); i++) {
          nei_cell = getNeighbor(anch_cell, jp, i);

          if ((nei_cell<0) &&
              (getValF( BUF_F, anch_b_val, boundary_tile, i ) < _eps)) {

            if (anch_n_tile==1) {

              if (op.verbose >= VB_ERROR ) {
                printf("# BeliefPropagation::cellConstraintPropagate: ERROR, "
                        "cell %i slated to remove last remaining tile (tile %s(%i) "
                        "conflicts with out of bounds neighbor %s(%i) dir %s(%d))\n",
                        (int)anch_cell,
                        m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                        m_tile_name[boundary_tile].c_str(), (int)boundary_tile,
                        m_dir_desc[i].c_str(), (int)i);
              }

              return -1;
            }

            tile_valid = 0;

            if (op.verbose >= VB_INTRASTEP ) {
              printf("# REMOVE it:%i cell:%i;[%i,%i,%i] tile %i (boundary nei, tile:%i, dir:%i(%s)) [cp.0]\n",
                  (int)op.cur_iter,
                  (int)anch_cell,
                  (int)jp.x, (int)jp.y, (int)jp.z,
                  (int)anch_b_val,
                  (int)boundary_tile, (int)i, (char *)m_dir_desc[i].c_str());

               //printf("# removing tile %i from cell %i (boundary nei, tile:%i, dir:%i(%s))\n",
               //   (int)anch_b_val, (int)anch_cell,
               //   (int)boundary_tile, (int)i, (char *)m_dir_desc[i].c_str());
            }

            removeTileIdx (anch_cell, anch_b_idx);

            if ( getValI( BUF_TILE_IDX_N, anch_cell ) == 1 ) {
              resolved++;

              if (op.verbose >= VB_INTRASTEP ) {
                printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i [cp.0]\n",
                    (int)op.cur_iter,
                    (int)anch_cell,
                    (int)jp.x, (int)jp.y, (int)jp.z,
                    (int)getValI( BUF_TILE_IDX, 0, anch_cell ) );
              }

            }

            cellFillVisited (anch_cell, 1 - m_note_plane );

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
          nei_cell = getNeighbor(anch_cell, jp, i);

          if (nei_cell<0) { continue; }

          anch_has_valid_conn = 0;

          nei_n_tile = getValI ( BUF_TILE_IDX_N, nei_cell );
          for (nei_a_idx=0; nei_a_idx < nei_n_tile; nei_a_idx++) {
            nei_a_val = getValI ( BUF_TILE_IDX, nei_a_idx, nei_cell );

            if (getValF( BUF_F, anch_b_val, nei_a_val, i ) > _eps) {
              anch_has_valid_conn = 1;
              break;
            }
          }

          if (!anch_has_valid_conn) {
            if (anch_n_tile==1) {

              if (op.verbose >= VB_ERROR ) {
                printf("# BeliefPropagation::cellConstraintPropagate: ERROR, "
                        "cell %i slated to rmove last remaining tile (tile %s(%i) "
                        "conflicts with neighbor cell %i, tile %s(%i) dir %s(%d))\n",
                        (int)anch_cell,
                        m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                        (int)nei_cell,
                        m_tile_name[nei_a_val].c_str(), (int)nei_a_val,
                        m_dir_desc[i].c_str(), (int)i);
              }

              return -1;
            }

            tile_valid = 0;

            if (op.verbose >= VB_INTRASTEP ) {
              printf("# REMOVE it:%i cell:%i;[%i,%i,%i] tile %i (invalid conn dir:%i(%s), tile:%i) [cp.1]\n",
                  (int)op.cur_iter,
                  (int)anch_cell,
                  (int)jp.x, (int)jp.y, (int)jp.z,
                  (int)anch_b_val,
                  (int)i, (char *)m_dir_desc[i].c_str(), (int)nei_a_val);
            }

            removeTileIdx(anch_cell, anch_b_idx);

            if ( getValI( BUF_TILE_IDX_N, anch_cell ) == 1 ) {
              resolved++;

              if (op.verbose >= VB_INTRASTEP ) {
                printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i [cp.1]\n",
                    (int)op.cur_iter,
                    (int)anch_cell,
                    (int)jp.x, (int)jp.y, (int)jp.z,
                    (int)getValI( BUF_TILE_IDX, 0, anch_cell ) );
              }


            }

            cellFillVisited (anch_cell, 1 - m_note_plane );
            anch_b_idx--;
            anch_n_tile--;

            break;
          }

        }

        if (!tile_valid) { continue; }
      }
    }

    unfillVisited (1 - m_note_plane );

    if (m_note_n[ m_note_plane ] == 0) { still_culling = 0; }

    m_note_n[ m_note_plane ] = 0;
    m_note_plane  = 1 - m_note_plane ;
  }

  // count resolved
  //
  st.iter_resolved += resolved;
  st.total_resolved += resolved;

  return 0;
}
