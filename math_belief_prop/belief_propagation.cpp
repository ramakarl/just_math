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

// #define USE_PERF             // explicit perf instrumentation (uncomment to opt in)

#ifdef USE_PERF
  #include "timex.h"      // app perf
#else
  #define PERF_PUSH(x)
  #define PERF_POP()
#endif

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
  st.constraints = -1;

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
  op.use_lookahead = 0;

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

  op.block_schedule = OPT_BLOCK_NONE;

  return 0;
}

void BeliefPropagation::SelectAlgorithm ( int alg_idx ) {
  op.alg_idx = alg_idx;

  switch (alg_idx) {
  case ALG_BP:                                // BP
    op.alg_cell_opt = ALG_CELL_ANY;
    op.alg_tile_opt = ALG_TILE_MAX_BELIEF;
    op.alg_run_opt  = ALG_RUN_VANILLA;
    break;
  case ALG_WFC:                               // WFC
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_WFC;
    op.alg_cell_opt = ALG_CELL_WFC;
    break;
  case ALG_MMS_SEQ:                           // MMS_SEQ
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_MMS;
    op.alg_cell_opt = ALG_CELL_MMS;
    op.block_schedule = OPT_BLOCK_SEQUENTIAL;
    break;
  case ALG_MMS_RAND1:                         // MMS_RAND1
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_MMS;
    op.alg_cell_opt = ALG_CELL_MMS;
    op.block_schedule = OPT_BLOCK_RANDOM_POS;
    break;
  case ALG_MMS_RAND2:                         // MMS_RAND2
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_MMS;
    op.alg_cell_opt = ALG_CELL_MMS;
    op.block_schedule = OPT_BLOCK_RANDOM_POS_SIZE;
    break;

  case ALG_BMS:                               // BMS, Breakout Model Synth
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_BREAKOUT;
    op.alg_cell_opt = ALG_CELL_BREAKOUT;
    op.block_schedule = OPT_BLOCK_RANDOM_POS;
    break;

  case ALG_BMS_MIN:                           // BMS, Breakout Model Synth, min block entropy
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_BREAKOUT;
    op.alg_cell_opt = ALG_CELL_BREAKOUT;
    op.block_schedule = OPT_BLOCK_MIN_ENTROPY;
    break;
  case ALG_BMS_MIN_NOISE:                     // BMS, Breakout Model Synth, min block entropy + noise
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_BREAKOUT;
    op.alg_cell_opt = ALG_CELL_BREAKOUT;
    op.block_schedule = OPT_BLOCK_NOISY_MIN_ENTROPY;
    break;

  case ALG_BMS_MAX_NOISE:                     // BMS, Breakout Model Synth, max block entropy + noise
    op.alg_accel    = ALG_ACCEL_NONE;
    op.alg_run_opt  = ALG_RUN_BREAKOUT;
    op.alg_cell_opt = ALG_CELL_BREAKOUT;
    op.block_schedule = OPT_BLOCK_NOISY_MAX_ENTROPY;
    break;

  case ALG_BP_MIN:                            // BP, Min Entropy
    op.alg_cell_opt = ALG_CELL_MIN_ENTROPY;
    op.alg_tile_opt = ALG_TILE_MAX_BELIEF;
    op.alg_run_opt  = ALG_RUN_VANILLA;
    break;
  case ALG_BP_MIN_WAVE:                       // BP, Min Entropy, Wavefront
    op.alg_cell_opt = ALG_CELL_MIN_ENTROPY;
    op.alg_tile_opt = ALG_TILE_MAX_BELIEF;
    op.alg_run_opt  = ALG_RUN_VANILLA;
    op.alg_accel    = ALG_ACCEL_WAVE;
    break;
  case ALG_BP_MIN_RESIDUAL:                   // BP, Min Entropy, Residual
    op.alg_cell_opt = ALG_CELL_MIN_ENTROPY;
    op.alg_tile_opt = ALG_TILE_MAX_BELIEF;
    op.alg_run_opt  = ALG_RUN_RESIDUAL;
    break;
  default:        // unspecified, BP default
    op.alg_cell_opt = ALG_CELL_ANY;
    op.alg_tile_opt = ALG_TILE_MAX_BELIEF;
    op.alg_run_opt  = ALG_RUN_VANILLA;
    break;
  };
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

  PERF_PUSH("ZeroBuf");
  m_buf[id].FillBuffer ( 0 );
  PERF_POP();
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

  //-- Construct PREFATORY_TILE_IDX
  //
  AllocBuf ( BUF_PREFATORY_TILE_IDX,   'i', m_num_values, m_num_verts );
  AllocBuf ( BUF_PREFATORY_TILE_IDX_N, 'i', m_num_verts );

  //-- Construct SAVE_TILE_IDX
  //
  AllocBuf ( BUF_SAVE_TILE_IDX,   'i', m_num_values, m_num_verts );
  AllocBuf ( BUF_SAVE_TILE_IDX_N, 'i', m_num_verts );

  // initialize to all tiles per vertex
  //
  for (int i=0; i<m_num_verts; i++) {

    // vtx i has num_vals possible number of tiles
    //
    SetValI ( BUF_TILE_IDX_N, (m_num_values), i );
    SetValI ( BUF_PREFATORY_TILE_IDX_N, (0), i );
    SetValI ( BUF_SAVE_TILE_IDX_N, (0), i );

    for (int b=0; b<m_num_values; b++) {

      // initially put all tile ids
      // in order at vtx i
      //
      SetValI ( BUF_TILE_IDX, b, b, i );
      SetValI ( BUF_PREFATORY_TILE_IDX, b, b, i );
      SetValI ( BUF_SAVE_TILE_IDX, b, b, i );
    }
  }

  //-- Construct visited
  //
  AllocBuf ( BUF_VISITED, 'l', m_num_verts );

  //-- Construct Residual BP buffers
  //
  AllocBuf ( BUF_RESIDUE_HEAP,          'f', 6 * m_num_verts * m_num_values );
  AllocBuf ( BUF_RESIDUE_HEAP_CELL_BP,  'l', 6 * m_num_verts * m_num_values );
  AllocBuf ( BUF_RESIDUE_CELL_HEAP,     'l', 6 * m_num_verts * m_num_values );

  AllocBuf ( BUF_BT,     'l', 2 * m_num_verts * m_num_values );
  AllocBuf ( BUF_BT_IDX, 'l',     m_num_verts * m_num_values );

  //-- Construct note
  //
  AllocBuf ( BUF_NOTE,    'l', m_num_verts, 2 );
  m_note_n[0] = 0;
  m_note_n[1] = 0;
  m_note_plane = 0;

  //-- Randomize MU
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

  //-- for block algorithms (mms, bms)
  //
  AllocBuf ( BUF_BLOCK, 'i', m_num_verts );

  //-- for block alg entropy calculations
  //
  AllocBuf ( BUF_CELL_ENTROPY, 'f', m_num_verts );
  AllocBuf ( BUF_BLOCK_ENTROPY, 'f', m_num_verts );

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

void BeliefPropagation::ComputeBP_DiffMUField () {

  int i, n_a, a;
  float v0, v1, d, max_diff;
  Vector3DI jp;

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

// ComputeTile0Field
// - called prior to CheckConstraints
// - computes a resolved tile field in which each
//   tile is just the first entry in the available list. eg. WFC
// - compare this to ComputeBP_BeliefField in which each tile
//   is selected based on BP belief
//
void BeliefPropagation::ComputeTile0Field() {

  int tile_val;

  for (int j=0; j < m_num_verts; j++) {

    tile_val = getValI ( BUF_TILE_IDX, 0, j );

    SetValI( BUF_TILES, tile_val , j );
  }

}

// ComputeNoteField
// - mark the notes for visualization
//
void BeliefPropagation::ComputeNoteField () {

  int64_t note_idx;
  int64_t vtx;

  if (m_note_n[m_note_plane]==0) return;

  // printf ( "%d\n", m_note_n[ m_note_plane ] );

  ZeroBuf ( BUF_VIZ );

  for (note_idx=0; note_idx < (int64_t) m_note_n[ m_note_plane  ]; note_idx++) {

    vtx = getValL ( BUF_NOTE, note_idx, m_note_plane  );

    SetValF( BUF_VIZ, 1.0, vtx );
  }

}


// ComputeBP_BeliefField
// - fills BUF_TILES with maxbelief tiles (always)
// - fills BUF_VIZ with maxbelief values (if VIZ_BELIEF)
//
void BeliefPropagation::ComputeBP_BeliefField () {

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
      //  (will be overwritten on next bp step)
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
  case VIZ_TILES_2D:         msg = "VIZ_TILES_2D"; break;
  case VIZ_TILE0:            msg = "VIZ_TILE0"; break;
  case VIZ_TILECOUNT:        msg = "VIZ_TILECOUNT"; break;
  case VIZ_NOTES:            msg = "VIZ_NOTES"; break;
  case VIZ_CONSTRAINT:       msg = "VIZ_CONSTRAINT"; break;
  case VIZ_BP_BELIEF:        msg = "VIZ_BP_BELIEF"; break;
  case VIZ_BP_ENTROPY:       msg = "VIZ_BP_ENTROPY"; break;
  case VIZ_BP_MU:            msg = "VIZ_BP_MU"; break;
  case VIZ_BP_DMU:           msg = "VIZ_BP_DMU"; break;
  };
  printf ( "  Visualizing: %s\n", msg.c_str() );
}

void BeliefPropagation::PrepareVisualization ()
{
  // visualization prep
  // call this *before* updateMU, and use
  // to prepare for calling getVisSample
  //
  PERF_PUSH("PrepareVis");

  clock_t t1;
  if (st.instr) t1 = clock();
  switch (op.viz_opt) {
  case VIZ_TILE0:
      // simple viz. matches json output.
      // show the final tile resolved at tile entry 0. if multiple take the first one.
      //.. nothing to do here
      break;
  case VIZ_TILECOUNT:
      // simple viz. number of available tiles per cell.
      //.. nothing to do here
      break;
  case VIZ_CONSTRAINT:
      // this visualization is useful but more costly.
      // compute a resolved tile field, then checks how many remaining unresolved
      // constraints each tile has.
      ComputeTile0Field ();
      CheckConstraints ();
      break;
  case VIZ_NOTES:
      // visualize notes
      ComputeNoteField ();
      break;
  case VIZ_BP_BELIEF:
      // find the maximum belief tile for each cell.
      ComputeBP_BeliefField ();
      break;
  case VIZ_BP_MU:
      //ComputeMUField ();
      break;
  case VIZ_BP_DMU:
      ComputeBP_DiffMUField ();
      break;
  };

  if (st.instr) {st.time_viz += clock()-t1;}

  PERF_POP();
}

// WFC  cnt = getTilesAtVertex( j );
// DMU  dmu =  scalar * std::max(0.0f, std::min(1.0f, pow ( src.getVal ( BUF_DMU, j ), 0.1f ) ));
//
Vector4DF BeliefPropagation::getVisSample ( int64_t v ) {

  float f, r, g, b, c;
  Vector4DF s;

  int i;
  float vexp = 0.3;
  float vscale = 0.1;

  float vmax = op.eps_converge * 10.0;

  switch (op.viz_opt) {
  case VIZ_TILES_2D:
    // tiles for 2D render. get literal tile value & count
    i = getValI ( BUF_TILE_IDX, 0, v);
    f = getValI ( BUF_TILE_IDX_N, v );
    s = Vector4DF( i, i, i, f );
    break;
  case VIZ_TILE0:
    // readily available. no prepare needed.
    // get tile ID normalized to num tiles
    i = getValI ( BUF_TILE_IDX, 0, v );
    r = float(int(i*327) % 255) / 255.0f;
    b = float(int(i*67125) % 255) / 255.0f;
    f = float(i) / float(getNumValues(v));
    s = Vector4DF( f, f, f, 0.5 );
    break;
  case VIZ_TILECOUNT:
    // readily available. no prepare needed.
    // visualize 1/TILE_NDX_N as alpha, so opaque/white = fully resolved
    i = getValI ( BUF_TILE_IDX_N, v );
    f = 1.0 / float(i);
    s = Vector4DF( f, f, f, f );
    break;
  case VIZ_CONSTRAINT:
    // visualize remaining constraints per cell
    // constraints are associated with faces, so max per cell is 6
    c = getValI ( BUF_C, v ) / 6.0f;
    s = Vector4DF( c, c, c, c );
    break;
  case VIZ_NOTES:
    // visualize notes
    f = getValF ( BUF_VIZ, v );
    s = Vector4DF(f,f,f,f);
    break;
  case VIZ_BP_BELIEF:
    // get maxbelief value
    f = getValF ( BUF_B, v );
    f = f / st.max_belief;
    s = Vector4DF(f,f,f,f);
    break;
  case VIZ_BP_MU:
    // BP only. requires PrepareVisualization for ComputeMUField
    f = getValF ( BUF_VIZ, v );
    s = Vector4DF(f,f,f,f);
    break;
  case VIZ_BP_DMU:
    // BP only. requires PrepareVisualization for ComputeDiffMUField
    f = getValF ( BUF_VIZ, v );
    c = 0.1 + std::max(0.0f, std::min(1.0f, pow ( f * vscale / vmax, vexp ) ));
    if ( f <= op.eps_converge ) s = Vector4DF(0,c,0,c);
    else                        s = Vector4DF(c,c,c,c);
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

  // get linear interpolation eps
  //
  float _eps = getLinearEps();

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
              //if ( dmu >= op.eps_converge * conv_frac ) eval++;
              if ( dmu >= _eps * conv_frac ) eval++;
            }
        }
        dmu = getValF( BUF_MU_NXT, 0, 0, anch_cell );
        //if ( eval==0 && dmu < op.eps_converge * conv_frac ) { continue; }
        if ( eval==0 && dmu < _eps * conv_frac ) { continue; }
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

  float belief_eps = 0.05;

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

    //---- attempt at more randomness in maps
    //float u = 1.0 - float(op.cur_iter) / op.max_iter;
    //if ( f >= max_p - belief_eps*u && m_rand.randF(0,1) > 0.5f*u ) {

    if ( f >= max_p ) {
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

// find minimum entropy cell and tile
// If non-null, min_cell, min_tile, min_tile_idx and min_entropy will all be filled
//   in appropriately if a minimum entropy cell and tile is found
//
// return values:
//
// 0                - grid is fully realized (without contradictions, presumably), so no entry chosen
// positive value   - returns count of number of minimum entropy values (within eps_zero)
// negative value   - error
//
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

    if (anch_tile_idx_n==0) { return -1; }
    if (anch_tile_idx_n==1) { continue; }

    g_sum = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
      g = getValF( BUF_G, anch_tile );
      g_sum += g;
    }

    if (g_sum < _eps) { continue; }

    _entropy = 0.0;
    for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
      anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
      g = getValF( BUF_G, anch_tile ) / g_sum;
      if (g > _eps) {
        _entropy += -g * logf(g);
      }
    }


    // initialize min entropy
    //
    if (count==0) {
      _min_cell     = anch_cell;
      _min_entropy = _entropy;
      count=1;
      continue;
    }

    if (_entropy < (_min_entropy + _eps)) {

      if (_entropy < _min_entropy) {
        _min_entropy  = _entropy;
        _min_cell     = anch_cell;
        count=1;
      }

      // randomize 'equal' choices
      //
      else {

        count++;
        p = m_rand.randF();

        if ( p < (1.0/(float)count) ) {
          _min_entropy  = _entropy;
          _min_cell     = anch_cell;
        }
      }

    }

  }

  if (_min_cell<0) { return 0; }

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

        break;
      }
    }

  }

  // ? just pick the first one?
  //
  else {

    _min_tile = getValI( BUF_TILE_IDX, 0, anch_cell );
    _min_tile_idx = 0;

  }

  if (min_cell)     { *min_cell     = _min_cell; }
  if (min_tile)     { *min_tile     = _min_tile; }
  if (min_tile_idx) { *min_tile_idx = _min_tile_idx; }
  if (min_entropy)  { *min_entropy  = _min_entropy; }

  return count;
}


// find minimum entropy cell and tile restricted to a block
// If non-null, min_cell, min_tile, min_tile_idx and min_entropy will all be filled
//   in appropriately if a minimum entropy cell and tile is found
//
// return values:
//
// 0                - block is fully realized (without contradictions, presumably), so no entry chosen
// positive value   - returns count of number of minimum entropy values (within eps_zero)
// negative value   - error
//
int BeliefPropagation::chooseMinEntropyBlock( std::vector<int64_t> &block_bound,
                                              int64_t *min_cell,
                                              int32_t *min_tile,
                                              int32_t *min_tile_idx,
                                              float *min_entropy) {

  int64_t anch_cell=0;
  int32_t anch_tile_idx,
          anch_tile_idx_n,
          anch_tile;
  int count=0;

  float p, g, _entropy, g_sum, g_cdf;

  float _min_entropy= -1.0;
  int64_t _min_cell = -1;
  int32_t _min_tile = -1,
          _min_tile_idx = -1;
  float _eps = op.eps_zero;

  int64_t block_s_x = -1,
          block_s_y = -1,
          block_s_z = -1;
  int64_t block_n_x = -1,
          block_n_y = -1,
          block_n_z = -1;

  int32_t ix, iy, iz;

  block_s_x = block_bound[0];
  block_n_x = block_bound[1];

  block_s_y = block_bound[2];
  block_n_y = block_bound[3];

  block_s_z = block_bound[4];
  block_n_z = block_bound[5];

  PERF_PUSH("chooseMinEntropyBlock");

  for (iz=block_s_z; iz<(block_s_z + block_n_z); iz++) {
    for (iy=block_s_y; iy<(block_s_y + block_n_y); iy++) {
      for (ix=block_s_x; ix<(block_s_x + block_n_x); ix++) {

        anch_cell = getVertex( (int)ix, (int)iy, (int)iz );

        anch_tile_idx_n = getValI( BUF_TILE_IDX_N, anch_cell );

        if (anch_tile_idx_n==0) { PERF_POP(); return -1; }
        if (anch_tile_idx_n==1) { continue; }

        g_sum = 0.0;
        for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
          anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
          g = getValF( BUF_G, anch_tile );
          g_sum += g;
        }

        if (g_sum < _eps) { continue; }

        _entropy = 0.0;
        for (anch_tile_idx=0; anch_tile_idx < anch_tile_idx_n; anch_tile_idx++) {
          anch_tile = getValI( BUF_TILE_IDX, anch_tile_idx, anch_cell );
          g = getValF( BUF_G, anch_tile ) / g_sum;
          if (g > _eps) {
            _entropy += -g * logf(g);
          }
        }

        // initialize min entropy
        //
        if (count==0) {
          _min_cell     = anch_cell;
          _min_entropy = _entropy;
          count=1;
          continue;
        }

        if (_entropy < (_min_entropy + _eps)) {

          if (_entropy < _min_entropy) {
            _min_entropy  = _entropy;
            _min_cell     = anch_cell;
            count=1;
          }

          // randomize 'equal' choices
          //
          else {

            count++;
            p = m_rand.randF();

            if ( p < (1.0/(float)count) ) {
              _min_entropy  = _entropy;
              _min_cell     = anch_cell;
            }
          }

        }
      }
    }

  }

  if (_min_cell<0) { PERF_POP(); return 0; }

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

        break;
      }
    }

  }

  // ? just pick the first one?
  //
  else {

    _min_tile = getValI( BUF_TILE_IDX, 0, anch_cell );
    _min_tile_idx = 0;

  }

  if (min_cell)     { *min_cell     = _min_cell; }
  if (min_tile)     { *min_tile     = _min_tile; }
  if (min_tile_idx) { *min_tile_idx = _min_tile_idx; }
  if (min_entropy)  { *min_entropy  = _min_entropy; }

  PERF_POP();

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

  int64_t cell=-1;
  int32_t n_idx=0;

  int32_t ix=0, iy=0, iz=0;
  int32_t x=0, y=0, z=0;
  int32_t end_x=0, end_y=0, end_z=0;
  int32_t block_odd_dx=0,
          block_odd_dy=0,
          block_odd_dz=0;

  m_num_nbrs = getNumNeighbors(0);

  m_rand.seed ( op.seed );

  int v = m_rand.randI();  // first random # (used as spot check)

  if (op.verbose >= VB_RUN ) {
    printf ("  bpc start. GRID = %d,%d,%d, SEED = %d\n", op.X, op.Y, op.Z, op.seed );
  }

  //------------
  //------------ BLOCK calculations
  //------------

  // clip block parameters (if needed)
  //
  op.block_size[0] = ( (op.block_size[0] < m_bpres.x) ? op.block_size[0] : m_bpres.x );
  op.block_size[1] = ( (op.block_size[1] < m_bpres.y) ? op.block_size[1] : m_bpres.y );
  op.block_size[2] = ( (op.block_size[2] < m_bpres.z) ? op.block_size[2] : m_bpres.z );

  op.sub_block_range[0][0] = ( (op.sub_block_range[0][0] < m_bpres.x) ? op.sub_block_range[0][0] : m_bpres.x );
  op.sub_block_range[0][1] = ( (op.sub_block_range[0][1] < m_bpres.x) ? op.sub_block_range[0][1] : m_bpres.x );

  op.sub_block_range[1][0] = ( (op.sub_block_range[1][0] < m_bpres.y) ? op.sub_block_range[1][0] : m_bpres.y );
  op.sub_block_range[1][1] = ( (op.sub_block_range[1][1] < m_bpres.y) ? op.sub_block_range[1][1] : m_bpres.y );

  op.sub_block_range[2][0] = ( (op.sub_block_range[2][0] < m_bpres.z) ? op.sub_block_range[2][0] : m_bpres.z );
  op.sub_block_range[2][1] = ( (op.sub_block_range[2][1] < m_bpres.z) ? op.sub_block_range[2][1] : m_bpres.z );

  m_block_admissible_tile.clear();
  for (n_idx=0; n_idx<m_num_values; n_idx++) {
    m_block_admissible_tile.push_back(n_idx);
  }

  // calculate block index bounds.
  //
  op.block_idx[0] = 0;
  op.block_idx[1] = 0;
  op.block_idx[2] = 0;

  block_odd_dx = op.block_size[0] / 2;
  block_odd_dy = op.block_size[1] / 2;
  block_odd_dz = op.block_size[2] / 2;

  if (op.verbose >= VB_RUN) {
    printf ("  calc block index bounds.\n");
  }

  end_x = m_bpres.x - op.block_size[0];
  for (ix=0,x=0; ix<m_bpres.x; ix++) {
    op.block_idx[0]++;

    if (x >= end_x) { break; }
    if ((ix%2)==0) { x += block_odd_dx; }
    else { x = ((ix+1)/2) * op.block_size[0]; }
    if (x > end_x) { x = end_x; }
  }

  end_y = m_bpres.y - op.block_size[1];
  for (iy=0,y=0; iy<m_bpres.y; iy++) {
    op.block_idx[1]++;

    if (y >= end_y) { break; }
    if ((iy%2)==0) { y += block_odd_dy; }
    else { y = ((iy+1)/2) * op.block_size[1]; }
    if (y > end_y) { y = end_y; }
  }

  end_z = m_bpres.z - op.block_size[2];
  for (iz=0,z=0; iz<m_bpres.z; iz++) {
    op.block_idx[2]++;

    if (z >= end_z) { break; }
    if ((iz%2)==0) { z += block_odd_dz; }
    else { z = ((iz+1)/2) * op.block_size[2]; }
    if (z > end_z) { z = end_z; }
  }

  //------------
  //------------
  //------------


  // reset stats (must do first)
  //
  ResetStats ();

  // rebuild dynamic bufs
  //
  if (op.verbose >= VB_RUN) { printf ("  rebuild dynamic bufs.\n"); }
  ConstructDynamicBufs ();

  // taken care of in constructdyanmicbufs
  //RandomizeMU ();

  // debugInspect (Vector3DI(1,1,0), 0 );

  // cull boundary
  //
  if (op.verbose >= VB_RUN) {
    printf ("  cull boundary..\n");
  }

  ret = CullBoundary();
  if (ret < 0) { return ret; }

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

int BeliefPropagation::finish (int ret) {

  if ( ret==0 ) {
    // success. complete.
    printf ( "  DONE (SUCCESS).\n" );
  } else {
    // post error condition
    switch (ret) {
    case -1: printf ( "  DONE (FAIL). ERROR: chooseMaxBelief.\n" ); break;
    case -2: printf ( "  DONE (FAIL). ERROR: tileIdxCollapse.\n" ); break;
    case -3: printf ( "  DONE (FAIL). ERROR: cellConstraintPropagate.\n" ); break;
    case -77: printf ( "  DONE (STOPPED BY USER).\n"); break;
    default:
        printf ( "  DONE (FAIL).\n"); break;
    };
  }
  return ret;
}

void BeliefPropagation::advance_seed ( int amt ) {
  op.seed += amt;
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

    if ( (x < 0) || (x >= m_res.x) ||
         (y < 0) || (y >= m_res.y) ||
         (z < 0) || (z >= m_res.z) ) {
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
  for (int n=0; n < BUF_MAX; n++) {
    m_buf[ n ].Clear ();
  }

  // reset stats
  ResetStats();

  // do not erase ops as they
  // may be needed for re-init
}

void BeliefPropagation::init_dir_desc() {

  m_dir_desc.clear();

  m_dir_desc.push_back("+1:0:0");
  m_dir_desc.push_back("-1:0:0");
  m_dir_desc.push_back("0:+1:0");
  m_dir_desc.push_back("0:-1:0");
  m_dir_desc.push_back("0:0:+1");
  m_dir_desc.push_back("0:0:-1");

}

int BeliefPropagation::init( int Rx, int Ry, int Rz,
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
  if ( m_bpres.x > 1024 || m_bpres.y > 1024 || m_bpres.z > 1024 ) {
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
  //RandomizeMU (); // taken care of in constructdynamicbufs

  // options
  //
  op.use_cuda  = false;

  if (op.use_svd) {

    // m_num_values x m_num_values is an upper bound
    // on the matrix size. The dimensions used will
    // be m_num_values x d and d x m_num_values for
    // U and V respectively.
    //
    ConstructSVDBufs ();

    init_SVD();
  }

  // set max iterations
  //
  op.cur_iter = 0;
  op.max_iter = m_num_verts;


  // block init
  //
  if (op.block_size[0] == 0) { op.block_size[0] = 16; }
  if (op.block_size[1] == 0) { op.block_size[1] = 16; }
  if (op.block_size[2] == 0) { op.block_size[2] = 16; }

  op.sub_block[0] = 0;
  op.sub_block[1] = 0;
  op.sub_block[2] = 0;

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

  }

  return 0;
}

//---

// deprecated?
//
int BeliefPropagation::wfc() {

  int ret = 1;
  int64_t it;

  ret = wfc_start();
  if (ret < 0) {
    printf("ERROR: start failed (%i)\n", ret);
    return ret;
  }

  for (it = 0; it < m_num_verts; it++) {

    ret = wfc_step ( it );

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

  return ret;
}

// deprecated?
//
int BeliefPropagation::wfc_start() {
  int ret=0;

  m_rand.seed ( op.seed );

  ret = CullBoundary();
  return ret;
}

// deprecated?
//
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

  cellFillVisitedNeighbor(cell, m_note_plane );
  unfillVisited( m_note_plane );

  ret = cellConstraintPropagate();
  if (ret < 0) { return -3; }

  return 1;
}

//-------


void BeliefPropagation::_saveTileIdx(void) {
  int64_t cell=-1;
  int32_t tile=-1,
          tile_idx=-1,
          n_idx=-1;

  PERF_PUSH( "_saveTileIdx" );

  for (cell=0; cell<m_num_verts; cell++) {
    n_idx = getValI( BUF_TILE_IDX_N, cell );
    SetValI( BUF_SAVE_TILE_IDX_N, n_idx, cell );
    for (tile_idx=0; tile_idx<n_idx; tile_idx++) {
      tile = getValI( BUF_TILE_IDX, tile_idx, cell );
      SetValI( BUF_SAVE_TILE_IDX, tile, tile_idx, cell );
    }
  }
  PERF_POP();
}

void BeliefPropagation::_restoreTileIdx(void) {
  int64_t cell=-1;
  int32_t tile=-1,
          tile_idx=-1,
          n_idx=-1;

  PERF_PUSH( "_restoreTileIdx" );

  for (cell=0; cell<m_num_verts; cell++) {
    n_idx = getValI( BUF_SAVE_TILE_IDX_N, cell );
    SetValI( BUF_TILE_IDX_N, n_idx, cell );
    for (tile_idx=0; tile_idx<n_idx; tile_idx++) {
      tile = getValI( BUF_SAVE_TILE_IDX, tile_idx, cell );
      SetValI( BUF_TILE_IDX, tile, tile_idx, cell );
    }
  }

  PERF_POP();

}

// Compute individual cell entropies, using the BUF_G
// buffer to determine the individual tile probabilities.
// Cell entries with only 1 tile have 0 entropy.
//
// return:
// 0  - success
// !0 - failure (currently can't happen)
//
int BeliefPropagation::ComputeCellEntropy(void) {
  int32_t x, y, z,
          bx, by, bz,
          mx, my, mz;
  int64_t cell;
  int32_t tile, tile_idx, n_tile;

  float cell_entropy = 0.0,
        cell_renorm = 0.0,
        f = 0.0,
        lg2 = 0.0;

  lg2 = log(2.0);

  for (z=0; z<m_res.z; z++) {
    for (y=0; y<m_res.y; y++) {
      for (x=0; x<m_res.x; x++) {

        cell_renorm = 0.0;
        cell_entropy = 0.0;

        cell = getVertex(x,y,z);
        n_tile = getValI( BUF_TILE_IDX_N, cell );
        if (n_tile > 0) {
          for (tile_idx=0; tile_idx<n_tile; tile_idx++) {
            tile = getValI( BUF_TILE_IDX, tile_idx, cell );
            f = getValF( BUF_G, tile );
            cell_renorm += f;

            cell_entropy += (f * log(f) / lg2 );
          }
          cell_entropy /= cell_renorm;
          cell_entropy -= log(cell_renorm) / lg2;
          cell_entropy = -cell_entropy;
        }

        SetValF( BUF_CELL_ENTROPY, cell_entropy, cell );

      }
    }
  }

  return 0;
}

// Compute a block sum of entropies in BUF_CELL_ENTROPY
// and store in BUF_BLOCK_ENTROPY.
//
// if `reuse_cell_entropy` is non zero, don't recalculate
// the cell entropies. Otherwise, populate the cell entropies
// (BUF_CELL_ENTROPY).
//
// Block size is stored in op.block_size[].
// Uses a dynamic programming method to re-use partially
// constructed answers. This might suffer from some roundoff
// issues but is significantly faster than doing it the naive way.
//
// The algorithm proceeds in phases:
// * first calculate the B[0,0,0] block entropy
// * calculate the block entropy in each of the principle directions
//   (B[x,0,0], B[0,y,0], B[0,0,z]) subtracting off the receding plane
//   and adding the incoming plane from BUF_CELL_ENTROPY
// * calculate the xy, xz, yz corner planes (B[x,y,0], B[x,0,z], B[0,y,z])
//   by subtracting off the receding line from BUF_CELL_ENTROPY
//   and adding in the incoming line from BUF_CELL_ENTROPY
// * finally, do the 'bulk' middle section (B[1:,1:,1:]) by re-using
//   already calculated sum block entries and adding/subtracting the
//   appropriate
//
// return:
// 0  - success
// !0 - failure (currently can't happen)
//
int BeliefPropagation::ComputeBlockEntropy(int32_t reuse_cell_entropy) {
  int32_t x, y, z,
          xx, yy, zz,
          bx, by, bz,
          mx, my, mz;
  int64_t cell,
          cell_a, cell_b,
          cell_00, cell_01, cell_10, cell_11,
          be_cell,
          te_cell;
  int32_t tile, tile_idx, n_tile,
          c_idx;

  float f,
        block_entropy;

  //                  0  1  2  3  4  5  6  7
  //                  +  -  -  +  -  +  +  -
  int32_t coef[8] = { 1,-1,-1, 1,-1, 1, 1,-1 };

  int32_t d_idx[8][3] = {
    { -1, -1, -1 },
    { -1, -1,  0 },
    { -1,  0, -1 },
    { -1,  0,  0 },
    {  0, -1, -1 },
    {  0, -1,  0 },
    {  0,  0, -1 },
    {  0,  0,  0 }
  };

  int32_t n_b[3] = { 0, 0, 0 };
  int32_t bs[3] = { 0, 0, 0 };

  n_b[0] = m_res.x - op.block_size[0] + 1;
  n_b[1] = m_res.y - op.block_size[1] + 1;
  n_b[2] = m_res.z - op.block_size[2] + 1;

  bs[0] = op.block_size[0];
  bs[1] = op.block_size[1];
  bs[2] = op.block_size[2];

  // reuse_cell_entropy != 0
  // ===> reuse and don't recompute the cell
  //      entropy buffer (BUF_CELL_ENTROPY)
  //
  if (reuse_cell_entropy==0) {
    ComputeCellEntropy();
  }


  // init B[0,0,0]
  // O( s * s * s )
  //
  block_entropy = 0.0;
  for (z=0; z<bs[2]; z++) {
    for (y=0; y<bs[1]; y++) {
      for (x=0; x<bs[0]; x++) {
        cell = getVertex(x,y,z);
        block_entropy += getValF( BUF_CELL_ENTROPY, cell );
      }
    }
  }
  SetValF( BUF_BLOCK_ENTROPY, block_entropy, 0 );


  // B[0,0,z]
  // O( s * s * (Z-s) )
  //
  x=0; y=0;
  for (z=1; z<n_b[2]; z++) {
    block_entropy = getValF( BUF_BLOCK_ENTROPY, getVertex(x,y,z-1) );
    for (yy=0; yy<bs[1]; yy++) {
      for (xx=0; xx<bs[0]; xx++) {
        cell_a = getVertex( xx, yy, (z+bs[2]-1) );
        cell_b = getVertex( xx, yy, (z-1) );
        block_entropy += getValF( BUF_CELL_ENTROPY, cell_a ) - getValF( BUF_CELL_ENTROPY, cell_b );
      }
    }
    SetValF( BUF_BLOCK_ENTROPY, block_entropy, getVertex(x,y,z) );
  }


  // B[0,y,0]
  // O( s * (Y-s) * s )
  //
  x=0; z=0;
  for (y=1; y<n_b[1]; y++) {

    block_entropy = getValF( BUF_BLOCK_ENTROPY, getVertex(x,y-1,z) );
    for (zz=0; zz<bs[2]; zz++) {
      for (xx=0; xx<bs[0]; xx++) {
        cell_a = getVertex( xx, (y+bs[1]-1), zz );
        cell_b = getVertex( xx, (y-1), zz );

        block_entropy +=
            getValF( BUF_CELL_ENTROPY, getVertex( xx, (y+bs[1]-1), zz ) )
          - getValF( BUF_CELL_ENTROPY, getVertex( xx, (y-1),        zz ) );
      }
    }

    SetValF( BUF_BLOCK_ENTROPY, block_entropy, getVertex(x,y,z) );
  }

  // B[x,0,0]
  // O( (X-s) * s * s )
  //
  y=0; z=0;
  for (x=1; x<n_b[0]; x++) {
    block_entropy = getValF( BUF_BLOCK_ENTROPY, getVertex(x-1,y,z) );
    for (zz=0; zz<bs[2]; zz++) {
      for (yy=0; yy<bs[1]; yy++) {
        cell_a = getVertex( (x+bs[0]-1), yy, zz );
        cell_b = getVertex( (x-1), yy, zz);
        block_entropy += getValF( BUF_CELL_ENTROPY, cell_a ) - getValF( BUF_CELL_ENTROPY, cell_b );
      }
    }
    SetValF( BUF_BLOCK_ENTROPY, block_entropy, getVertex(x,y,z) );
  }



  // B[x,y,0]
  // O( (X-s) * (Y-s) * s )
  //
  z=0;
  for (y=1; y<n_b[1]; y++) {
    for (x=1; x<n_b[0]; x++) {
      block_entropy =
          getValF( BUF_BLOCK_ENTROPY, getVertex(x,    y-1,  z) )
        + getValF( BUF_BLOCK_ENTROPY, getVertex(x-1,  y,    z) )
        - getValF( BUF_BLOCK_ENTROPY, getVertex(x-1,  y-1,  z) );
      for (zz=0; zz<bs[2]; zz++) {
        block_entropy +=
            getValF( BUF_CELL_ENTROPY, getVertex( x-1,        y-1,        zz ) )
          - getValF( BUF_CELL_ENTROPY, getVertex( x-1,        y+bs[1]-1,  zz ) )
          - getValF( BUF_CELL_ENTROPY, getVertex( x+bs[0]-1,  y-1,        zz ) )
          + getValF( BUF_CELL_ENTROPY, getVertex( x+bs[0]-1,  y+bs[1]-1,  zz ) );
      }
      SetValF( BUF_BLOCK_ENTROPY, block_entropy, getVertex(x,y,z) );
    }
  }

  // B[x,0,z]
  // O( (X-s) * (Y-s) * s )
  //
  y=0;
  for (z=1; z<n_b[2]; z++) {
    for (x=1; x<n_b[0]; x++) {
      block_entropy =
          getValF( BUF_BLOCK_ENTROPY, getVertex(x,    y,  z-1) )
        + getValF( BUF_BLOCK_ENTROPY, getVertex(x-1,  y,  z  ) )
        - getValF( BUF_BLOCK_ENTROPY, getVertex(x-1,  y,  z-1) );
      for (yy=0; yy<bs[1]; yy++) {
        block_entropy +=
            getValF( BUF_CELL_ENTROPY, getVertex( x-1,        yy, z-1        ) )
          - getValF( BUF_CELL_ENTROPY, getVertex( x-1,        yy, z+bs[2]-1 ) )
          - getValF( BUF_CELL_ENTROPY, getVertex( x+bs[0]-1,  yy, z-1        ) )
          + getValF( BUF_CELL_ENTROPY, getVertex( x+bs[0]-1,  yy, z+bs[2]-1 ) );
      }
      SetValF( BUF_BLOCK_ENTROPY, block_entropy, getVertex(x,y,z) );
    }
  }

  // B[0,y,z]
  // O( s * (Y-s) * (Z-s) )
  //
  x=0;
  for (z=1; z<n_b[2]; z++) {
    for (y=1; y<n_b[1]; y++) {
      block_entropy =
          getValF( BUF_BLOCK_ENTROPY, getVertex(x,  y,    z-1) )
        + getValF( BUF_BLOCK_ENTROPY, getVertex(x,  y-1,  z  ) )
        - getValF( BUF_BLOCK_ENTROPY, getVertex(x,  y-1,  z-1) );
      for (xx=0; xx<bs[0]; xx++) {
        block_entropy +=
            getValF( BUF_CELL_ENTROPY, getVertex( xx, y-1,        z-1        ) )
          - getValF( BUF_CELL_ENTROPY, getVertex( xx, y-1,        z+bs[2]-1 ) )
          - getValF( BUF_CELL_ENTROPY, getVertex( xx, y+bs[1]-1,  z-1        ) )
          + getValF( BUF_CELL_ENTROPY, getVertex( xx, y+bs[1]-1,  z+bs[2]-1 ) );
      }
      SetValF( BUF_BLOCK_ENTROPY, block_entropy, getVertex(x,y,z) );
    }
  }

  // B[1:,1:,1:]
  // O( (X -s) * (Y -s) * (Z -s ) )
  //

  for (z=1; z<n_b[2]; z++) {
    for (y=1; y<n_b[1]; y++) {
      for (x=1; x<n_b[0]; x++) {

        block_entropy = 0.0;
        for (c_idx=0; c_idx<8; c_idx++) {
          bx = x + d_idx[c_idx][0];
          by = y + d_idx[c_idx][1];
          bz = z + d_idx[c_idx][2];

          if (c_idx<7) {
            block_entropy += coef[c_idx] * getValF( BUF_BLOCK_ENTROPY, getVertex( bx, by, bz ) );
          }

          xx = x - 1 + bs[0]*(d_idx[c_idx][0]+1);
          yy = y - 1 + bs[1]*(d_idx[c_idx][1]+1);
          zz = z - 1 + bs[2]*(d_idx[c_idx][2]+1);

          block_entropy += -coef[c_idx] * getValF( BUF_CELL_ENTROPY, getVertex( xx, yy, zz ) );
        }

        SetValF( BUF_BLOCK_ENTROPY, block_entropy, getVertex(x,y,z) );

      }
    }
  }

  return 0;

}

// Noise function, used in breakout's pickMaxEntropyNoiseBlock
// to add noise to max entropy block pick.
// Returns random value based on op.noise_func and
// noise_coefficient.
//
// See https://mathworld.wolfram.com/RandomNumber.html
// for motivation on generating the power law random variable.
//
//
//  WIP, UNTESTED
//
float BeliefPropagation::pickNoiseFunc() {
  float f, val = 0.0;
  float x_0=(1/1024.0), x_1=(1024.0*1024.0), alpha = -2.0;

  float tx0, tx1;

  if (op.noise_coefficient < op.eps_zero) {
    return 0.0;
  }

  switch (op.noise_func) {
    case OPT_NOISE_FUNC_UNIFORM:
      val = m_rand.randF() * op.noise_coefficient;
      break;
    case OPT_NOISE_FUNC_POWER_LAW:

      f = m_rand.randF();

      tx0 = pow(x_0, alpha+1.0);
      tx1 = pow(x_1, alpha+1.0);

      val = pow( (f*(tx1 - tx0) + tx0), 1.0/(alpha+1.0) );
      val *= op.noise_coefficient;
      break;

    default:
      val=0.0;
      break;
  }

  return val;
}


// Pick the maximum entropy block.
// Entropy blocks are calculated from the simple sum of
// cell entropies.
// call ComputeBlockEntropy to populate BUF_BLOCK_ENTROPY
// and BUF_CELL_ENTROPY, then do a sweep to find the maximum.
//
// stores chosen block in:
//
//  op.sub_block[]
//
//
// return:
// >=0  - success, number of maximum equal entropy blocks
// <0   - failure (currently can't happen)
//
int BeliefPropagation::pickMaxEntropyNoiseBlock(void) {
  int32_t x,y,z;

  // block buffer size: (X,Y,Z) - blocksize(x,y,z)
  //
  int32_t n_b[3] = {0,0,0};

  float max_entropy = -1, cur_entropy;
  int32_t max_x=0, max_y=0, max_z=0;
  int64_t cell;

  int32_t equal_entropy_count=0;

  float df = 0.0;

  // we have significant round off error because
  // of the running sums, so use a local epsilon
  // to try and mitigate the issue.
  //
  //float _eps = (1.0/16.0);
  float _eps = op.eps_zero;

  n_b[0] = m_res.x - op.block_size[0]+1;
  n_b[1] = m_res.y - op.block_size[1]+1;
  n_b[2] = m_res.z - op.block_size[2]+1;

  if (op.verbose >= VB_DEBUG) {
    printf("## pickMaxEntropyNoiseBlock: block bounds: [%i,%i,%i]\n", n_b[0], n_b[1], n_b[2]);
  }

  ComputeBlockEntropy();

  for (z=0; z<n_b[2]; z++) {
    for (y=0; y<n_b[1]; y++) {
      for (x=0; x<n_b[0]; x++) {

        cell = getVertex(x,y,z);
        cur_entropy = getValF( BUF_BLOCK_ENTROPY, cell );

        df = pickNoiseFunc();
        cur_entropy += df;

        if ((max_entropy < 0.0) ||
            ( (cur_entropy - max_entropy) > -_eps )) {

          // if we have a choice between blocks of equal entropy,
          // choose one from the list at random.
          //
          if (fabs(cur_entropy - max_entropy) < _eps) {
            equal_entropy_count++;
          }
          else {
            equal_entropy_count = 1;
          }

          if (m_rand.randF() <= (1.0/(float)equal_entropy_count)) {
            max_entropy = cur_entropy;
            max_x = x;
            max_y = y;
            max_z = z;
          }
        }

      }
    }
  }

  op.sub_block[0] = max_x;
  op.sub_block[1] = max_y;
  op.sub_block[2] = max_z;

  return (int)equal_entropy_count;
}

// Pick the minimum entropy block.
// Entropy blocks are calculated from the simple sum of
// cell entropies.
// call ComputeBlockEntropy to populate BUF_BLOCK_ENTROPY
// and BUF_CELL_ENTROPY, then do a sweep to find the minimum.
//
// stores chosen block in:
//
//  op.sub_block[]
//
//
// return:
// >=0  - success, number of minimum equal entropy blocks
// <0   - failure (currently can't happen)
//
int BeliefPropagation::pickMinEntropyNoiseBlock(void) {
  int32_t x,y,z;

  // block buffer size: (X,Y,Z) - blocksize(x,y,z)
  //
  int32_t n_b[3] = {0,0,0};

  float min_entropy = -1, cur_entropy;
  int32_t min_x=0, min_y=0, min_z=0;
  int64_t cell;

  int32_t equal_entropy_count=0;

  n_b[0] = m_res.x - op.block_size[0]+1;
  n_b[1] = m_res.y - op.block_size[1]+1;
  n_b[2] = m_res.z - op.block_size[2]+1;

  if (op.verbose >= VB_DEBUG) {
    printf("## pickMinEntropyNoiseBlock: block bounds: [%i,%i,%i]\n", n_b[0], n_b[1], n_b[2]);
  }

  ComputeBlockEntropy();

  for (z=0; z<n_b[2]; z++) {
    for (y=0; y<n_b[1]; y++) {
      for (x=0; x<n_b[0]; x++) {

        cell = getVertex(x,y,z);
        cur_entropy = getValF( BUF_BLOCK_ENTROPY, cell );

        cur_entropy += pickNoiseFunc();

        if ((min_entropy < 0.0) ||
            ( (cur_entropy - min_entropy) < op.eps_zero )) {

          // if we have a choice between blocks of equal entropy,
          // choose one from the list at random.
          //
          if (fabs(cur_entropy - min_entropy) < op.eps_zero) {
            equal_entropy_count++;
          }
          else {
            equal_entropy_count = 1;
          }

          if (m_rand.randF() <= (1.0/(float)equal_entropy_count)) {
            min_entropy = cur_entropy;
            min_x = x;
            min_y = y;
            min_z = z;
          }
        }

      }
    }
  }

  op.sub_block[0] = min_x;
  op.sub_block[1] = min_y;
  op.sub_block[2] = min_z;

  return (int)equal_entropy_count;
}


/*
// choose block with minimum (average) entropy (?)
//
// $\sum_{b \in B} \sum_{v \in \text{tile}(b)} \frac{g_b(v)}{|B|}$
//
// choose block with minimum (average) entropy (?)
// and add a noise factor to allow for some randomness in choice
//
// $\text(rand)() + \sum_{b \in B} \sum_{v \in \text{tile}(b)} \frac{g_b(v)}{|B|}$
//
int BeliefPropagation::pickMinEntropyNoiseBlock(void) {

  //DEBUG
  printf("## cp op.block_schedule:%i\n", (int)op.block_schedule);
  fflush(stdout);

  double _block_entropy = 0.0,
         _cell_entropy = 0.0,
         _min_block_entropy = 0.0,
         _cell_renorm = 0.0,
         lg2 = log(2.0);
  float _f;

  int32_t _block_choice[3] = {0,0,0};

  int32_t _start_block[3] = {0,0,0};
  int32_t _end_block_pos[3] = {0};
  int32_t _sx, _sy, _sz,
          _x, _y, _z;
  int64_t _cell;
  int32_t _tile_idx, _tile, _n_idx;

  int32_t _unfixed_cell_count = 0,
          _blocks_considered=0;

  _end_block_pos[0] = m_res.x - op.block_size[0] + 1;
  _end_block_pos[1] = m_res.y - op.block_size[1] + 1;
  _end_block_pos[2] = m_res.z - op.block_size[2] + 1;

  // very ineffient, testing idea out
  //
  for (_sz=0; _sz<_end_block_pos[2]; _sz++) {
    for (_sy=0; _sy<_end_block_pos[1]; _sy++) {
      for (_sx=0; _sx<_end_block_pos[0]; _sx++) {

        _unfixed_cell_count = 0;
        _block_entropy = 0.0;
        for (_z=_sz; _z<(_sz+op.block_size[2]); _z++) {
          for (_y=_sy; _y<(_sy+op.block_size[1]); _y++) {
            for (_x=_sx; _x<(_sx+op.block_size[0]); _x++) {

              _cell_entropy = 0.0;
              _cell_renorm = 0.0;

              _cell = getVertex((int)_x, (int)_y, (int)_z);
              _n_idx = getValI( BUF_TILE_IDX_N, _cell );

              if (_n_idx <= 1) { continue; }
              for (_tile_idx=0; _tile_idx<_n_idx; _tile_idx++) {

                _tile = getValI( BUF_TILE_IDX, _tile_idx, _cell );

                _f = getValF( BUF_G, _tile );
                _cell_renorm += (double)_f;

                _cell_entropy += (_f * log(_f) / lg2);

              }
              _cell_entropy /= _cell_renorm;
              _cell_entropy -= (log(_cell_renorm) / lg2);
              _cell_entropy = -_cell_entropy;

              _block_entropy += _cell_entropy;

              _unfixed_cell_count++;

            }
          }
        }

        if (_unfixed_cell_count==0) { continue; }

        if (_blocks_considered == 0) {
          _min_block_entropy = _block_entropy;
          _block_choice[0] = _sx;
          _block_choice[1] = _sy;
          _block_choice[2] = _sz;
        }

        if ( _block_entropy < _min_block_entropy ) {
          _min_block_entropy = _block_entropy;
          _block_choice[0] = _sx;
          _block_choice[1] = _sy;
          _block_choice[2] = _sz;
        }

        _blocks_considered++;


      }
    }
  }

  printf("## pickMinEntropyNoiseBlock _min_block_entropy:%3.4f {%i,%i,%i}) (num_cell:%i)\n",
      (float)_min_block_entropy,
      (int)_block_choice[0],
      (int)_block_choice[1],
      (int)_block_choice[2],
      (int)_blocks_considered);


  op.sub_block[0] = _block_choice[0];
  op.sub_block[1] = _block_choice[1];
  op.sub_block[2] = _block_choice[2];

  return (int)_unfixed_cell_count;
}
*/

//  0 - success
// -1 - error
//
int BeliefPropagation::RealizePre(void) {

  int ret = 0;

  int64_t cell=-1;
  int32_t tile=-1,
          tile_idx=-1,
          n_idx=-1,
          orig_tile=-1;
  float belief=-1.0, d = -1.0;

  // reset steps
  //
  op.cur_step = 0;

  float _eps = getLinearEps();

  int32_t x=0,y=0,z=0;

  int32_t ix=0, iy=0, iz=0;
  int32_t end_s[3];

  int sanity=0;

  if (op.verbose >= VB_INTRASTEP) {
    printf("RealizePre cp.0\n");
  }

  clock_t t1 = clock();

  //-----------
  //----------- REALIZEPRE - BLOCK SCHEDULE SECTION
  //-----------

  // block schedule is set means we're using an algorithm
  // that needs a block choice
  //
  if (op.block_schedule != OPT_BLOCK_NONE) {


    // If fail count is zero, it means we're at the start of the block
    // choice phase, so pick a new block.
    // Otherwise, keep the block that was previously picked.
    //
    if (m_block_fail_count==0) {

      // random position, fixed size
      //
      if (op.block_schedule == OPT_BLOCK_RANDOM_POS) {
        op.sub_block[0] = (int)(m_rand.randF() * (float)(m_res.x - op.block_size[0]));
        op.sub_block[1] = (int)(m_rand.randF() * (float)(m_res.y - op.block_size[1]));
        op.sub_block[2] = (int)(m_rand.randF() * (float)(m_res.z - op.block_size[2]));
      }

      // random position, random size
      //
      else if (op.block_schedule == OPT_BLOCK_RANDOM_POS_SIZE) {

        op.block_size[0] = ( op.sub_block_range[0][0] +
                            (int32_t)( m_rand.randF() * (float)(op.sub_block_range[0][1] - op.sub_block_range[0][0]) ) );
        op.block_size[1] = ( op.sub_block_range[1][0] +
                            (int32_t)( m_rand.randF() * (float)(op.sub_block_range[1][1] - op.sub_block_range[1][0]) ) );
        op.block_size[2] = ( op.sub_block_range[2][0] +
                            (int32_t)( m_rand.randF() * (float)(op.sub_block_range[2][1] - op.sub_block_range[2][0]) ) );

        op.sub_block[0] = (int)(m_rand.randF() * (float)(m_res.x - op.block_size[0]));
        op.sub_block[1] = (int)(m_rand.randF() * (float)(m_res.y - op.block_size[1]));
        op.sub_block[2] = (int)(m_rand.randF() * (float)(m_res.z - op.block_size[2]));

      }

      // sequential and overlapping
      //
      else if (op.block_schedule == OPT_BLOCK_SEQUENTIAL) {

        end_s[0] = m_bpres.x - op.block_size[0];
        end_s[1] = m_bpres.y - op.block_size[1];
        end_s[2] = m_bpres.z - op.block_size[2];

        //iz = op.cur_iter / (op.block_idx[0] * op.block_idx[1]);
        //iy = ( op.cur_iter - (iz * op.block_idx[0] * op.block_idx[1]) ) / (op.block_idx[0]) ;
        //ix = ( op.cur_iter - (iz * op.block_idx[0] * op.block_idx[1]) - (iy * op.block_idx[0]) );

        iz = op.seq_iter / (op.block_idx[0] * op.block_idx[1]);
        iy = ( op.seq_iter - (iz * op.block_idx[0] * op.block_idx[1]) ) / (op.block_idx[0]) ;
        ix = ( op.seq_iter - (iz * op.block_idx[0] * op.block_idx[1]) - (iy * op.block_idx[0]) );

        ix %= op.block_idx[0];
        iy %= op.block_idx[1];
        iz %= op.block_idx[2];

        x = (ix/2) * op.block_size[0];
        if ((ix%2)==1) { x += (op.block_size[0]/2); }
        if (ix == (op.block_idx[0]-1)) { x = end_s[0]; }

        y = (iy/2) * op.block_size[1];
        if ((iy%2)==1) { y += (op.block_size[1]/2); }
        if (iy == (op.block_idx[1]-1)) { y = end_s[1]; }

        z = (iz/2) * op.block_size[2];
        if ((iz%2)==1) { z += (op.block_size[2]/2); }
        if (iz == (op.block_idx[2]-1)) { z = end_s[2]; }

        op.sub_block[0] = x;
        op.sub_block[1] = y;
        op.sub_block[2] = z;

        //DEBUG
        //
        /*
        printf("#####\n");
        printf("#####\n");
        printf("## end_s:(%i,%i,%i)\n", (int)end_s[0], (int)end_s[1], (int)end_s[2]);
        printf("## ixyz: (%i,%i,%i)\n", (int)ix, (int)iy, (int)iz);
        printf("## iz = %i ( op.seq_iter:%i / (op.block_idx[0]:%i * op.block_idx[1]:%i) )\n",
            (int)iz, (int)op.seq_iter, (int)op.block_idx[0], (int)op.block_idx[1]);
        printf("## iy = %i ( op.seq_iter:%i - (iz:%i * (op.block_idx[0]:%i * op.block_idx[1]:%i) / (op.block_idx[0]:%i) )\n",
            (int)iy, (int)op.seq_iter, (int)iz, (int)op.block_idx[0], (int)op.block_idx[1], (int)op.block_idx[0] );
        printf("## ix = %i ( op.seq_iter:%i - (iz:%i * (op.block_idx[0]:%i * op.block_idx[1]:%i) - (iy:%i * op.block_idx[0]:%i) )\n",
            (int)ix, (int)op.seq_iter, (int)iz, (int)op.block_idx[0], (int)op.block_idx[1], (int)iy, (int)op.block_idx[0]);
        printf("## xyz: (%i,%i,%i)\n", (int)x, (int)y, (int)z);
        printf("#####\n");
        printf("#####\n");
        */
        //
        //DEBUG

      }

      else if ((op.block_schedule == OPT_BLOCK_MIN_ENTROPY) ||
               (op.block_schedule == OPT_BLOCK_NOISY_MIN_ENTROPY)) {

        pickMinEntropyNoiseBlock();

      }

      else if (op.block_schedule == OPT_BLOCK_NOISY_MAX_ENTROPY) {
        pickMaxEntropyNoiseBlock();
      }

      else {
        // error in block choice option?
        //
      }

      if (op.verbose >= VB_STEP) {
        printf("RealizePre : choosing new block ([%i+%i][%i+%i][%i+%i]) (block sched:%i)\n",
            (int)op.sub_block[0], (int)op.block_size[0],
            (int)op.sub_block[1], (int)op.block_size[1],
            (int)op.sub_block[2], (int)op.block_size[2],
            (int)op.block_schedule);
      }

    }

    else {

      if (op.verbose >= VB_STEP) {
        printf("RealizePre : keeping block ([%i+%i][%i+%i][%i+%i]) (block sched:%i)\n",
            (int)op.sub_block[0], (int)op.block_size[0],
            (int)op.sub_block[1], (int)op.block_size[1],
            (int)op.sub_block[2], (int)op.block_size[2],
            (int)op.block_schedule);
      }

    }

  }

  // non block algorithm
  //
  else { }


  //-----------
  //----------- REALIZEPRE - ALG SECTION
  //-----------

  //---
  switch (op.alg_run_opt) {
  case ALG_RUN_VANILLA:
    // after we've propagated constraints, BUF_MU
    // needs to be renormalized
    //
    WriteBoundaryMUbuf(BUF_MU);
    NormalizeMU(BUF_MU);

    if (op.verbose >= VB_STEP) {
      printf("# RealizePre %f (%i/%i) eps[%f:%f]\n",
          (float) _eps, (int) op.cur_iter, (int) op.max_iter,
          (float) op.eps_converge_beg, (float) op.eps_converge_end);
    }
    break;

  case ALG_RUN_RESIDUAL:

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
    // over correctly.
    //
    // populate the indexHeap: priority queue with maximum
    // difference of MU and MU_NXT as heap key in addition
    // to keep the "mu index" (position in MU buffer)
    //
    // From this point forward, updates to BUF_MU or BUF_MU_NXT
    // will need a corresponding bookkeeping call to indexXHeap
    // to keep track of the maximum difference between
    // the two buffers and corresponding cell index information.
    //
    d = step(1);
    d = step(0);
    indexHeap_init();

    break;

  case ALG_RUN_WFC:

    // Nothing to be done.
    // All relevant code is in RealizeStep and RealizePost
    //
    break;

  case ALG_RUN_BREAKOUT:

    // inefficient, but just to get working, save whole grid state
    // so that if we need to restore state after a block choice failure,
    // we can reset it
    //
    _saveTileIdx();

    // fuzz out a block
    //
    m_note_n[ m_note_plane ] = 0;
    m_note_n[ 1 - m_note_plane  ] = 0;

    for (x=op.sub_block[0]; x<(op.sub_block[0]+op.block_size[0]); x++) {
      for (y=op.sub_block[1]; y<(op.sub_block[1]+op.block_size[1]); y++) {
        for (z=op.sub_block[2]; z<(op.sub_block[2]+op.block_size[2]); z++) {

          n_idx=0;
          cell = getVertex(x,y,z);

          // we need to 'fuzz' a block by loading the prefatory state as
          // there might be user level constraints that need to be pushed
          // to whichever block is getting fuzzed.
          //

          n_idx = getValI( BUF_PREFATORY_TILE_IDX_N, cell );
          for (tile_idx=0; tile_idx < n_idx; tile_idx++) {
            tile = getValI( BUF_PREFATORY_TILE_IDX, tile_idx, cell );
            SetValI( BUF_TILE_IDX, tile, tile_idx, cell );

            cellFillVisitedSingle ( cell, m_note_plane );
            cellFillVisitedNeighbor ( cell, m_note_plane );
          }
          SetValI( BUF_TILE_IDX_N, n_idx, cell );

        }
      }
    }

    // reset visited from above so that constraint propagate
    // can use it.
    //
    unfillVisited( m_note_plane  );

    // If we get an error here during the initial constraint propagate
    // phase, before we start trying block realizations in RealizeStep,
    // this is not strictly a hard error.
    // If we fail to set an initial state, we should consider
    // this as a failed breakout state so we can proceed to the
    // 'soften' phase.
    // Regardless, we should return a 'success' code
    // as we still want to continue trying, while
    // saving information to pass to RealizePost to make
    // sure to do the soften phase.
    //
    // If we were unable to get into an arc consistent state before
    // this step, that is an error.
    // Meaning, if the prefatory state, after the setup
    // constraint propagation phase, is not arc consistent, that
    // is a valid error state and should have been handled before
    // we got here.
    //

    // propagate constraints to remove neighbor tiles,
    // and count number resolved (only 1 tile val remain)
    //

    // keep this. necessary here after fuzzing
    //
    ret = cellConstraintPropagate();
    if (ret < 0) {
      m_return = -1;
      ret = 0;
    }

    break;

  case ALG_RUN_MMS:

    // fuzz out a block
    //
    m_note_n[ m_note_plane ] = 0;
    m_note_n[ 1 - m_note_plane  ] = 0;

    for (x=op.sub_block[0]; x<(op.sub_block[0]+op.block_size[0]); x++) {
      for (y=op.sub_block[1]; y<(op.sub_block[1]+op.block_size[1]); y++) {
        for (z=op.sub_block[2]; z<(op.sub_block[2]+op.block_size[2]); z++) {

          n_idx=0;
          cell = getVertex(x,y,z);

          orig_tile = getValI( BUF_TILE_IDX, 0, cell );

          for (tile_idx=0; tile_idx < m_block_admissible_tile.size(); tile_idx++) {
            tile = m_block_admissible_tile[tile_idx];
            SetValI( BUF_TILE_IDX, tile, tile_idx, cell );

            cellFillVisitedSingle ( cell, m_note_plane );
            cellFillVisitedNeighbor ( cell, m_note_plane );

            n_idx++;
          }

          SetValI( BUF_TILE_IDX_N, n_idx, cell );

          // save original value
          //
          SetValI( BUF_BLOCK, orig_tile, cell );

        }
      }
    }

    // reset visited from above so that constraint propagate
    // can use it.
    //
    unfillVisited( m_note_plane  );

    // propagate constraints to remove neighbor tiles,
    // and count number resolved (only 1 tile val remain)
    //
    ret = cellConstraintPropagate();
    if (ret < 0) { break; }
    break;

  default:
    // bad algorithm choice error?
    //
    break;
  }

  // measure elapsed time
  clock_t t2 = clock();
  st.elapsed_time += ((double(t2) - t1)*1000.0) / CLOCKS_PER_SEC;

  return ret;
}

// check to make sure sub block has a fully realized state
// block is interleaved block start and size:
// [x, dx, y, dy, z, dz]
//
// 0   - success (sanity passed)
// !0  - error
//
int BeliefPropagation::sanityBreakoutRealizedBlock(std::vector<int32_t> &block) {

  int32_t sx = block[0],
          dx = block[1],

          sy = block[2],
          dy = block[3],

          sz = block[4],
          dz = block[5];

  int32_t x,y,z;
  int ret=0;

  int64_t cell;
  int32_t n_idx, tile, tile_idx;

  for (z=sz; z<(sz+dz); z++) {
    for (y=sy; y<(sy+dy); y++) {
      for (x=sx; x<(sx+dx); x++) {
        cell = getVertex(x,y,z);

        n_idx = getValI( BUF_TILE_IDX_N, cell );
        if (n_idx != 1) { return -1; }

      }
    }
  }

  return 0;
}

// check to make sure BUF_SAVE_TILE_IDX* is the same as BUF_TILE_IDX*
//
// 0   - success (sanity passed)
// !0  - error
//
int BeliefPropagation::sanityBreakoutSavedTileGrid(void) {
  int ret=0;

  int64_t cell;
  int32_t tile_idx;
  int32_t n_idx_orig, tile_orig;
  int32_t n_idx_save, tile_save;

  for (cell=0; cell<m_num_verts; cell++) {
    n_idx_orig = getValI( BUF_TILE_IDX_N, cell );
    n_idx_save = getValI( BUF_SAVE_TILE_IDX_N, cell );

    if (n_idx_orig != n_idx_save) { return -1; }

    for (tile_idx=0; tile_idx<n_idx_orig; tile_idx++) {
      tile_orig = getValI( BUF_TILE_IDX, tile_idx, cell );
      tile_save = getValI( BUF_SAVE_TILE_IDX, tile_idx, cell );

      if (tile_orig != tile_save) { return -2; }
    }

  }

  return 0;
}

// returns max and min in a block in _debug_stat.
// _block_bounds is interleaved start and end of block (end non-inclusive)
// [sx, ex, sy, ey, sz, ez]
//
// returns 0
//
int BeliefPropagation::sanityBreakoutStatBlock(std::vector<int32_t> &_debug_stat, int32_t *_block_bounds) {

  int32_t sx = _block_bounds[0],
          ex = _block_bounds[1],

          sy = _block_bounds[2],
          ey = _block_bounds[3],

          sz = _block_bounds[4],
          ez = _block_bounds[5];

  int32_t x,y,z,
          n_idx;
  int64_t cell;

  _debug_stat.clear();
  _debug_stat.push_back(-1);
  _debug_stat.push_back(-1);

  for (z=sz; z<(ez); z++) {
    for (y=sy; y<(ey); y++) {
      for (x=sx; x<(ex); x++) {
        cell = getVertex((int)x, (int)y, (int)z);

        n_idx = getValI(BUF_TILE_IDX_N, cell);

        if (_debug_stat[0] < 0) { _debug_stat[0] = n_idx; }
        if (_debug_stat[1] < 0) { _debug_stat[1] = n_idx; }

        if (n_idx < _debug_stat[0]) { _debug_stat[0] = n_idx; }
        if (_debug_stat[1] < n_idx) { _debug_stat[1] = n_idx; }

      }
    }
  }

  return 0;
}

// check to see if grid is in 'ground' state (all cell
// entries // only have one entry).
//
// 0  - success, all cell entries have exactly one tile
// >0 - fail, number of cells > 0
// <0 - at least one cell has 0 tiles
//
int BeliefPropagation::sanityGroundState(void) {
  int64_t cell;
  int32_t n_idx;

  int count=0;

  for (cell=0; cell<m_num_verts; cell++) {
    n_idx = getValI(BUF_TILE_IDX_N, cell);
    if (n_idx == 0) { return -1; }
    if (n_idx > 1) { count++; }
  }

  return count;
}

// inefficient fucntion to count number of fixed tiles
//
int64_t BeliefPropagation::numFixed(void) {
  int64_t cell, count=0;
  int32_t n_idx;

  for (cell=0; cell<m_num_verts; cell++) {
    n_idx = getValI(BUF_TILE_IDX_N, cell);
    if (n_idx == 1) { count++; }
  }

  return count;
}

// CollapseAndPropagate
//  1: continue to next pre/step
//  0: never returns zero
// <0: error
int BeliefPropagation::CollapseAndPropagate (int64_t& cell, int32_t& tile, int32_t& tile_idx ) {

    int32_t n_idx=-1;
    Vector3DI vp;

    // 1 = continue condition.
    // display chosen cell
    //
    if (op.verbose >= VB_INTRASTEP ) {
      vp = getVertexPos(cell);
      n_idx = getValI ( BUF_TILE_IDX_N, cell );
      printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i, tile_idx:%i / %i [rp]\n",
          (int) op.cur_iter,
          (int) cell,
          (int) vp.x, (int)vp.y, (int)vp.z,
          (int) tile, (int)tile_idx, (int)n_idx);
    }

    // reset iter resolved and advance total
    //
    st.iter_resolved = 1;
    st.total_resolved++;

    // assume continue
    //
    int ret = 1;

    // collapse
    //
    int c_ret = tileIdxCollapse( cell, tile_idx );

    if (c_ret >=0 ) {
      m_note_n[ m_note_plane ] = 0;
      m_note_n[ 1 - m_note_plane  ] = 0;

      cellFillVisitedNeighbor ( cell, m_note_plane );
      unfillVisited( m_note_plane  );

      // propagate constraints to remove neighbor tiles,
      // and count number resolved (only 1 tile val remain)
      //
      c_ret = cellConstraintPropagate();
      //  0: success
      // <0: contradiction found

      // error. constraint prop failed
      //
      if (c_ret < 0) { ret = -3; }

    }
    // error. collapse failed
    //
    else {
      ret = -2;
    }
    return ret;
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
  int32_t tile=-1,
          tile_idx=-1,
          n_idx=-1,
          orig_tile=-1;
  float belief=-1.0,
        d = -1.0;

  int32_t _soften_bounds[6] = {0};

  int32_t x=0, y=0, z=0;

  Vector3DI vp;

  int64_t fixed_count = 0;

  //DEBUG
  //
  std::vector<int32_t> _debug_stat;
  int sanity=0;
  //
  //DEBUG

  // measure elapsed time
  clock_t t1 = clock();

  // resolved = all tiles complete (1 tile), and arc-consistent state
  // arc-consistent = ac3.
  //

  // choose the cell and propagate choice
  //
  switch (op.alg_cell_opt) {
    case ALG_CELL_WFC:

      ret = chooseMinEntropy( &cell, &tile, &tile_idx, &belief);
      //  0: SOLVED GRID. assuming arc consistent. all cells collapsed to single tileid.
      //>=1: One or more tiles ready to collapse (ie. the cell).

      if ( ret >= 1 ) {
        ret = CollapseAndPropagate (cell, tile, tile_idx);
      }
      break;

    case ALG_CELL_BREAKOUT:

      if (op.verbose >= VB_STEP) {
        printf("RealizePost: BREAKOUT m_return: %i (ground_state:%i)\n", (int)m_return, sanityGroundState());
      }

      // assume continue
      //
      ret = 1;

      if (m_return == 0) {

        if (op.verbose >= VB_STEP) {
          printf("RealizePost: BREAKOUT-accept ([%i+%i][%i+%i][%i+%i] (m_block_fail_count:%i / m_block_retry_limit:%i)\n",
              (int)op.sub_block[0], (int)op.block_size[0],
              (int)op.sub_block[1], (int)op.block_size[1],
              (int)op.sub_block[2], (int)op.block_size[2],
              (int)m_block_fail_count,
              (int)m_block_retry_limit);
        }

        op.seq_iter++;
        m_block_fail_count=0;

      }

      else if (m_return < 0) {

        if (op.verbose >= VB_STEP) {
          printf("RealizePost: BREAKOUT-restore ([%i+%i][%i+%i][%i+%i] (m_block_fail_count:%i / m_block_retry_limit:%i)\n",
              (int)op.sub_block[0], (int)op.block_size[0],
              (int)op.sub_block[1], (int)op.block_size[1],
              (int)op.sub_block[2], (int)op.block_size[2],
              (int)m_block_fail_count,
              (int)m_block_retry_limit);
        }

        // before we soften, we need to restore the grid state to before we started mucking
        // around with fixing a block within it
        //
        _restoreTileIdx();

        // SOFTEN phase
        //
        if (m_block_fail_count >= m_block_retry_limit) {
          op.seq_iter++;
          m_block_fail_count = 0;

          if (op.verbose >= VB_STEP) {
            printf("RealizePost: BREAKOUT-SOFTEN ([%i+%i][%i+%i][%i+%i] (failed to find breakout block)\n",
                (int)op.sub_block[0], (int)op.block_size[0],
                (int)op.sub_block[1], (int)op.block_size[1],
                (int)op.sub_block[2], (int)op.block_size[2]);
          }

          // assume neighbor blocks to soften are same size as center block that was
          // attempting to be fixed
          //
          _soften_bounds[0] = op.sub_block[0] - op.block_size[0];
          _soften_bounds[1] = op.sub_block[0] + (2*op.block_size[0]);

          _soften_bounds[2] = op.sub_block[1] - op.block_size[1];
          _soften_bounds[3] = op.sub_block[1] + (2*op.block_size[1]);

          _soften_bounds[4] = op.sub_block[2] - op.block_size[2];
          _soften_bounds[5] = op.sub_block[2] + (2*op.block_size[2]);

          if (_soften_bounds[0] < 0) { _soften_bounds[0] = 0; }
          if (_soften_bounds[2] < 0) { _soften_bounds[2] = 0; }
          if (_soften_bounds[4] < 0) { _soften_bounds[4] = 0; }

          if (_soften_bounds[1] >= m_bpres.x) { _soften_bounds[1] = m_bpres.x; }
          if (_soften_bounds[3] >= m_bpres.y) { _soften_bounds[3] = m_bpres.y; }
          if (_soften_bounds[5] >= m_bpres.z) { _soften_bounds[5] = m_bpres.z; }

          if (op.verbose >= VB_STEP) {
            printf("RealizePost: BREAKOUT-SOFTEN bounds ([%i:%i][%i:%i][%i:%i]\n",
                (int)_soften_bounds[0], (int)_soften_bounds[1],
                (int)_soften_bounds[2], (int)_soften_bounds[3],
                (int)_soften_bounds[4], (int)_soften_bounds[5]);
          }

          m_note_n[ m_note_plane ] = 0;
          m_note_n[ 1 - m_note_plane  ] = 0;

          for (z=_soften_bounds[4]; z<_soften_bounds[5]; z++) {
            for (y=_soften_bounds[2]; y<_soften_bounds[3]; y++) {
              for (x=_soften_bounds[0]; x<_soften_bounds[1]; x++) {

                cell = getVertex(x,y,z);

                n_idx = getValI( BUF_PREFATORY_TILE_IDX_N, cell );
                SetValI( BUF_TILE_IDX_N, n_idx, cell );

                for (tile_idx=0; tile_idx<n_idx; tile_idx++) {
                  tile = getValI( BUF_PREFATORY_TILE_IDX, tile_idx, cell );
                  SetValI( BUF_TILE_IDX, tile, tile_idx, cell );
                }

                cellFillVisitedSingle ( cell, m_note_plane );
                cellFillVisitedNeighbor ( cell, m_note_plane );

              }
            }
          }

          if (op.verbose >= VB_DEBUG) {
            sanityBreakoutStatBlock(_debug_stat, _soften_bounds);
            printf("## RealizePost: breakout-soften ([%i:%i][%i:%i][%i:%i] (n_idx min:%i, max:%i)\n",
                  (int)_soften_bounds[0], (int)_soften_bounds[1],
                  (int)_soften_bounds[2], (int)_soften_bounds[3],
                  (int)_soften_bounds[4], (int)_soften_bounds[5],
                  (int)_debug_stat[0], (int)_debug_stat[1]);

          }


          // reset visited from above so that constraint propagate
          // can use it.
          //
          unfillVisited( m_note_plane  );

          // if the constraint propgation fails, we're in a bad state
          // as we should have been in an even more unrestricted state
          // before we started since we restored from a previously
          // arc consistent state and now we've softened (made wildcard)
          // the block and it's neighbors in question.
          //
          ret = cellConstraintPropagate();
          if (ret < 0) {
            //op.cur_iter++;
          }

        }

      }

      if ( ret >= 0 ) {
          // no error..

          // Whether we've softened or accepted the block, we're finished.
          // We're taking control away from the code at the bottom
          // since we're not resolving a single cell/tile now,
          // so we need to do some housekeeping ourselves.
          //

          //op.cur_iter++;

          // slow way to check to see if we've converged
          //
          if (numFixed() == m_num_verts) {
            ret = 0;

          } else {
            ret = 1;
          }
      }

      break;

    case ALG_CELL_MMS:


      // TODO: potentially check for iteration count.
      // There's a check on the outer loop (in main)
      // to only iteration for a certain number but it
      // should probably go here as well.
      //
      // For now, just return 1 (continue) so that
      // it's handled at a higher level.
      //

      if (op.verbose >= VB_STEP) {
        printf("RealizePost: WFC_MMS m_return %i\n", (int)m_return);
      }

      // if wfc failed on the block, reset to previously known good state
      //
      if (m_return < 0) {

        if (op.verbose >= VB_STEP) {
          printf("RealizePost: MMS restore ([%i+%i][%i+%i][%i+%i] (MMS block fail) (m_block_fail_count:%i / m_block_retry_limit:%i)\n",
              (int)op.sub_block[0], (int)op.block_size[0],
              (int)op.sub_block[1], (int)op.block_size[1],
              (int)op.sub_block[2], (int)op.block_size[2],
              (int)m_block_fail_count,
              (int)m_block_retry_limit);
        }

        // restore block
        //
        for (x=op.sub_block[0]; x<(op.sub_block[0] + op.block_size[0]); x++) {
          for (y=op.sub_block[1]; y<(op.sub_block[1] + op.block_size[1]); y++) {
            for (z=op.sub_block[2]; z<(op.sub_block[2] + op.block_size[2]); z++) {

              cell = getVertex(x,y,z);
              orig_tile = getValI( BUF_BLOCK, cell );
              SetValI( BUF_TILE_IDX, orig_tile, 0, cell );
              SetValI( BUF_TILE_IDX_N, 1, cell );
            }
          }
        }

        // only retry the block m_block_retry_limit times.
        // if we hit the retry limit, reset the fail count so
        // that a new block is chosen in RealizePre
        //
        if (m_block_fail_count >= m_block_retry_limit) {

          op.seq_iter++;
          m_block_fail_count=0;

          if (op.verbose >= VB_STEP) {
            printf("RealizePost: MMS giving up on block ([%i+%i][%i+%i][%i+%i] (mms block fail)\n",
                (int)op.sub_block[0], (int)op.block_size[0],
                (int)op.sub_block[1], (int)op.block_size[1],
                (int)op.sub_block[2], (int)op.block_size[2]);
          }


        }

      }

      // else we accept move on
      //
      else {

        if (op.verbose >= VB_STEP) {
          printf("RealizePost: MMS accept block ([%i+%i][%i+%i][%i+%i]\n",
              (int)op.sub_block[0], (int)op.block_size[0],
              (int)op.sub_block[1], (int)op.block_size[1],
              (int)op.sub_block[2], (int)op.block_size[2]);
        }

        op.seq_iter++;
        m_block_fail_count=0;
      }

      // we're taking control away from the code at the bottom
      // since we're not resolving a single cell/tile now,
      // so we need to do some housekeeping ourselves.
      //
      //op.cur_iter++;
      ret = 1;
      break;

    case ALG_CELL_ANY:

      ret = chooseMaxBelief( &cell, &tile, &tile_idx, &belief );

      if ( ret >= 1 ) {
        ret = CollapseAndPropagate (cell, tile, tile_idx);
      }
      break;

    case ALG_CELL_MIN_ENTROPY:

      ret = chooseMinEntropyMaxBelief( &cell, &tile, &tile_idx, &belief );

      if ( ret >= 1 ) {
        ret = CollapseAndPropagate (cell, tile, tile_idx);
      }
      break;

    default:
      // no alg
      ret = -1;
      break;

  }

  //-- complete timing
  // **NOTE**: Should not need to CheckConstraints here. Each algorithm should be able
  // to determine when it succeeds. Use to confirm success, should result in constraints=0.
  //
  /*if (st.enabled) {
      // check constraints
      printf ( "  checking constraints (warning: slow)\n");
      st.constraints = CheckConstraints ();
  }*/

  PrepareVisualization();

  st.eps_curr = getLinearEps();
  st.post = ret;

  clock_t t2 = clock();
  st.elapsed_time += ((double(t2) - t1)*1000.0) / CLOCKS_PER_SEC;

  // print iter stats
  //
  if (ret==1 && op.verbose >= VB_STEP ) {
    printf ("%s", getStatMessage().c_str() );
  }
  // print run completion
  if (ret<=0 && op.verbose >= VB_RUN ) {

    st.success = (ret==0);

    printf ("%s", getStatMessage().c_str() );
  }

  op.cur_iter++;
  return ret;
}


std::string BeliefPropagation::getStatMessage () {

  char msg[1024] = {0};

  snprintf ( msg, 1024,
             "  %s: %d/%d, Iter: %d, %4.1fmsec, constr:%d, resolved: %d/%d/%d (%4.1f%%), Grid:%d,%d,%d, Seed: %d\n",
              ((st.post==1) ? "RUN" :
              ((st.success==1) ? "RUN_SUCCESS" : "RUN_FAIL")),
              op.cur_run, op.max_run, op.cur_iter,
              st.elapsed_time, (int)st.constraints,
              st.iter_resolved, (int)st.total_resolved, op.max_iter, 100.0*float(st.total_resolved)/op.max_iter,
              op.X, op.Y, op.Z, op.seed );


  //-- belief prop stats
  /* snprintf ( msg, 1024,
             "  %s: %d/%d, Iter: %d, %4.1fmsec, constr:%d, resolved: %d/%d/%d (%4.1f%%), steps %d/%d, max.dmu %f, eps %f, av.mu %1.5f, av.dmu %1.8f\n",
              ((st.post==1) ? "RUN" :
              ((st.success==1) ? "RUN_SUCCESS" : "RUN_FAIL")),
              op.cur_run, op.max_run, op.cur_iter,
              st.elapsed_time, (int)st.constraints,
              st.iter_resolved, (int)st.total_resolved, op.max_iter, 100.0*float(st.total_resolved)/op.max_iter,
              op.cur_step, op.max_step,
              st.max_dmu, st.eps_curr, st.ave_mu, st.ave_dmu ); */

  // b:%f, n:%f, bp:%f, v:%f, md:%f, u:%f
  // st.time_boundary, st.time_normalize, st.time_bp, st.time_viz, st.time_maxdiff, st.time_updatemu );

  return msg;
}

std::string BeliefPropagation::getStatCSV (int mode) {

  char msg[1024] = {0};
  int status;
  status = (st.post==1) ? 0 : (st.success==1) ? 1 : -1;

  snprintf ( msg, 1024,
      "%d, %d, %d, %4.1f, %d, %d, %d, %d, %4.1f%%, %d, %d, %d, %d",
      op.cur_run, status, op.cur_iter, st.elapsed_time,
      (int) st.constraints, st.iter_resolved, st.total_resolved, op.max_iter, 100.0*float(st.total_resolved)/op.max_iter,
      op.X, op.Y, op.Z, op.seed );


 //     op.cur_step, op.max_step, st.max_dmu, st.eps_curr,
 //     st.ave_mu, st.ave_dmu,
 //     st.time_boundary, st.time_normalize, st.time_bp, st.time_viz, st.time_maxdiff, st.time_updatemu );

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

// 0 - converged
// 1 - not converged yet
//
int BeliefPropagation::RealizeStep(void) {

  float   d=-1.0,
          f_residue=-1.0,
          belief=-1.0;
  int64_t mu_idx=-1,
          cell=-1;
  int32_t idir=-1,
          tile=-1,
          tile_idx=-1,
          n_idx=-1;

  Vector3DI vp;
  int resolved = 0;

  std::vector< int64_t > _block_bound;

  // measure elapsed time
  //
  clock_t t1 = clock();

  // get linear interpolation eps
  //
  float _eps = getLinearEps();

  // assume continue
  //
  int ret = 1,
      _ret = -1;

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

  else if (op.alg_run_opt == ALG_RUN_WFC) {

    // all computation for WFC happens in RelaizePost.
    // In some sense, running WFC is like running BP
    // with 0 steps.
    //

    // make sure to indicate that wfc should 'stop'
    // stepping and go into RealizePost
    //
    ret = 0;

  }

  else if (op.alg_run_opt == ALG_RUN_MMS) {

    // Here we run WFC on the block we've fuzzed
    // Since the whole grid outside of the block we've fuzzed
    // is in a 'ground state' (tile count exactly 1), we can
    // run vanilla WFC without worrying about moving out of the
    // block.
    //
    ret = chooseMinEntropy( &cell, &tile, &tile_idx, &belief);
    m_return = ret;

    if (ret >= 1) {

      // 1 = continue condition.
      // display chosen cell
      //
      if (op.verbose >= VB_INTRASTEP ) {
        vp = getVertexPos(cell);
        n_idx = getValI ( BUF_TILE_IDX_N, cell );
        printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i, belief:%f (tile_idx:%i / %i) [rs]\n",
            (int)op.cur_iter,
            (int)cell,
            (int)vp.x, (int)vp.y, (int)vp.z,
            (int)tile, 0.0, (int)tile_idx, (int)n_idx);
      }

      // reset iter resolved and advance total
      //
      st.iter_resolved = 1;
      st.total_resolved++;

      // collapse
      //
      _ret = tileIdxCollapse( cell, tile_idx );
      if (_ret >= 0) {

        m_note_n[ m_note_plane ] = 0;
        m_note_n[ 1 - m_note_plane  ] = 0;

        cellFillVisitedNeighbor ( cell, m_note_plane );
        unfillVisited( m_note_plane  );

        // propagate constraints to remove neighbor tiles,
        // and count number resolved (only 1 tile val remain)
        //
        _ret = cellConstraintPropagate();

      }

      // collapse failed
      //
      else {
        ret = -2;
        m_return = ret;
      }

      // ret inherits chooseMinEntropy return value (presumably 1)
      // unless there's an error, in which case we propagate that
      // value on
      //
      if (_ret < 0) {
        ret = -3;
        m_return = ret;
        m_block_fail_count++;
      }

    }

  }

  else if (op.alg_run_opt == ALG_RUN_BREAKOUT) {

    // here we run WFC on the block we've fuzzed
    //

    _block_bound.push_back( op.sub_block[0] );
    _block_bound.push_back( op.block_size[0] );

    _block_bound.push_back( op.sub_block[1] );
    _block_bound.push_back( op.block_size[1] );

    _block_bound.push_back( op.sub_block[2] );
    _block_bound.push_back( op.block_size[2] );

    ret = chooseMinEntropyBlock( _block_bound, &cell, &tile, &tile_idx, &belief);
    m_return = ret;

    if (ret >= 1) {

      // 1 = continue condition.
      // display chosen cell
      //
      if (op.verbose >= VB_INTRASTEP ) {
        vp = getVertexPos(cell);
        n_idx = getValI ( BUF_TILE_IDX_N, cell );
        printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i, belief:%f (tile_idx:%i / %i) [rs-breakout]\n",
            (int)op.cur_iter,
            (int)cell,
            (int)vp.x, (int)vp.y, (int)vp.z,
            (int)tile, 0.0, (int)tile_idx, (int)n_idx);
      }

      // reset iter resolved and advance total
      //
      st.iter_resolved = 1;
      st.total_resolved++;

      // collapse
      //
      _ret = tileIdxCollapse( cell, tile_idx );
      if (_ret >= 0) {

        m_note_n[ m_note_plane ] = 0;
        m_note_n[ 1 - m_note_plane  ] = 0;

        cellFillVisitedNeighbor ( cell, m_note_plane );
        unfillVisited( m_note_plane  );

        // propagate constraints to remove neighbor tiles,
        // and count number resolved (only 1 tile val remain)
        //
        _ret = cellConstraintPropagate();

      }
      // collapse failed
      //
      else {
        ret = -2;
        m_return = ret;
      }

      // ret inherits chooseMinEntropy return value (presumably 1)
      // unless there's an error, in which case we propagate that
      // value on
      //
      if (_ret < 0) {
        //m_breakout_block_fail_count++;
        m_block_fail_count++;
        ret = -3;
        m_return = ret;
      }

    }

    // If we get an error in the choice of tile to fix,
    // we want to stop but don't want to communicate that
    // to the higher level.
    // RealizePost will act approparitely to either
    // back out into the previous state or implement
    // the soften step.
    //
    // m_return will contain the return code that
    // RealizePost can use.
    //
    else if (ret < 0) {
      m_block_fail_count++;
      ret = 0;
    }

  }

  else if (op.alg_run_opt == ALG_RUN_BACKTRACK) {

    ret = 0;
  }

  //--- unknown algorithm

  else { ret = -1; }

  // complete timing
  //
  clock_t t2 = clock();
  st.elapsed_time += ((double(t2) - t1)*1000.0) / CLOCKS_PER_SEC;

  // always increment cur_step;
  op.cur_step++;

  // ret 1 is a 'continue' state, so if we haven't finished,
  // make sure we don't loop forever by incrementing cur_step
  // and return max_step is exceedex
  //
  if ( ret==1 ) {
    if (op.cur_step >= op.max_step )  { ret = -2; }
  }

  // prep vis
  PrepareVisualization();

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

  // REQUIRED: ComputeBeliefField to be called
  // before invoking this function.

  // compute the unresolved constraints at each vertex
  //
  int cnt = 0;
  for (int64_t vtx = 0; vtx < m_num_verts; vtx++) {
    cnt += CheckConstraints ( vtx );
  }

  return cnt;
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

// deprecated?
//
float BeliefPropagation::step (int update_mu) {

  float max_diff = -1.0;
  int m_calc_residue = 1;

  clock_t t1, t2;

  // initial boundary conditions
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

  PrepareVisualization();

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

// Discard tile entries at cell position `pos`
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

    if (op.verbose >= VB_DEBUG ) {
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

// Add tiles at cell positoin `pos`.
// If a tile already exists, do nothing.
//
// return:
//
// >=0 - number of tiles added to pos
// <0  - error (currently can't happen)
//
int BeliefPropagation::filterAdd(uint64_t pos, std::vector<int32_t> &tile_id) {
  int32_t tile_idx,
          idx,
          n,
          tile_val,
          tv,
          found;
  std::vector< int32_t > add_list;

  n = getValI( BUF_TILE_IDX_N, pos );
  int nstart = n;

  for (tile_idx=0; tile_idx<tile_id.size(); tile_idx++) {
    tile_val = tile_id[tile_idx];

    found = 0;
    for (idx=0; idx<n; idx++) {
      if (tile_val == getValI( BUF_TILE_IDX, idx, pos )) {
        found = 1;
        break;
      }
    }

    if (found==0) { add_list.push_back(tile_val); }
  }
  if (add_list.size()==0) { return 0; }

  for (tile_idx=0; tile_idx<add_list.size(); tile_idx++) {
    SetValI( BUF_TILE_IDX, add_list[tile_idx], n, pos );
    n++;
  }
  SetValI( BUF_TILE_IDX_N, n, pos );

  return (int)add_list.size();
}

// Inefficiant scan to recover tile ID from tile name
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

void BeliefPropagation::debugPrintCellEntropy() {
  int32_t x,y,z;

  for (z=0; z<m_res.z; z++) {
    for (y=0; y<m_res.y; y++) {
      for (x=0; x<m_res.x; x++) {
        printf(" %2.3f", getValF( BUF_CELL_ENTROPY, getVertex(x,y,z)) );
      }
      printf("\n");
    }
    printf("\n");
  }
  printf("\n");

}

void BeliefPropagation::debugPrintBlockEntropy() {
  int32_t x,y,z;
  int32_t n_b[3];

  n_b[0] = m_res.x - op.block_size[0] + 1;
  n_b[1] = m_res.y - op.block_size[1] + 1;
  n_b[2] = m_res.z - op.block_size[2] + 1;

  for (z=0; z<n_b[2]; z++) {
    for (y=0; y<n_b[1]; y++) {
      for (x=0; x<n_b[0]; x++) {
        //printf(" %2.3f", getValF( BUF_BLOCK_ENTROPY, getVertex(x,y,z)) );
        printf(" %4.8f", getValF( BUF_BLOCK_ENTROPY, getVertex(x,y,z)) );
      }
      printf("\n");
    }
    printf("\n");
  }
  printf("\n");

}

void BeliefPropagation::debugPrintTerse(int buf_id) {

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

  int buf_tile_idx = BUF_TILE_IDX,
      buf_tile_idx_n = BUF_TILE_IDX_N;

  if (buf_id == BUF_PREFATORY_TILE_IDX) {
    buf_tile_idx = BUF_PREFATORY_TILE_IDX;
    buf_tile_idx_n = BUF_PREFATORY_TILE_IDX_N;
  }
  else if (buf_id == BUF_SAVE_TILE_IDX) {
    buf_tile_idx = BUF_SAVE_TILE_IDX;
    buf_tile_idx_n = BUF_SAVE_TILE_IDX_N;
  }

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
  printf("stat_enabled: %i\n", (int) st.enabled);
  printf("op{max_step:%i, block_retry_limit:%i, noise_coefficient:%f, eps_zero:%f}\n",
      (int)op.max_step,
      (int)m_block_retry_limit,
      (float)op.noise_coefficient,
      (float)op.eps_zero);

  printf("m_tile_name[%i]:\n", (int)m_tile_name.size());
  for (i=0; i < m_tile_name.size(); i++) {
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

  printf("buf_id:%i, buf:%i, buf_n:%i\n", buf_id, buf_tile_idx, buf_tile_idx_n);
  for (u=0; u<m_num_verts; u++) {
    p = getVertexPos(u);
    a_idx_n = getValI( buf_tile_idx_n, u );

    printf("[%i,%i,%i](%i): ", (int)p.x, (int)p.y, (int)p.z, (int)u);
    for (a_idx=0; a_idx<a_idx_n; a_idx++) {
      a = getValI( buf_tile_idx, (int)a_idx, (int)u );
      printf(" %i", (int)a);
    }
    printf("\n");
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
  printf("eps_converge: [%f,%f](%f), eps_zero: %f, rate: %f, max_step: %i, seed: %i\n",
      (float) op.eps_converge_beg, (float) op.eps_converge_end,
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

  PERF_PUSH("tileIdxCollapse");

  n = getValI( BUF_TILE_IDX_N, pos );
  if (tile_idx >= n) { PERF_POP(); return -1; }

  tile_val = getValI( BUF_TILE_IDX, tile_idx, pos );
  tv = getValI( BUF_TILE_IDX, 0, pos );
  SetValI( BUF_TILE_IDX, tile_val, 0, pos);
  SetValI( BUF_TILE_IDX, tv, tile_idx, pos);
  SetValI( BUF_TILE_IDX_N, 1, pos );

  if (st.enabled) {
    st.num_collapsed += n-1;
  }
  PERF_POP();

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
  int64_t note_idx;


  PERF_PUSH("CullBoundary");

  // set initial notes

  for (y=0; y<m_res.y; y++) {
    for (z=0; z<m_res.z; z++) {

      vtx = getVertex(0, y, z);
      assert ( vtx < m_num_verts );

      note_idx = m_note_n[ m_note_plane ];
      SetValL ( BUF_NOTE, (vtx), note_idx, m_note_plane );
      m_note_n[ m_note_plane ]++;

      if ((m_res.x-1) != 0) {

        vtx = getVertex(m_res.x-1, y, z);
        assert ( vtx < m_num_verts );

        note_idx = m_note_n[ m_note_plane ];
        SetValL ( BUF_NOTE, (vtx), note_idx, m_note_plane );
        m_note_n[ m_note_plane ]++;
      }

    }
  }

  for (x=1; x<(m_res.x-1); x++) {
    for (z=0; z<m_res.z; z++) {

      vtx = getVertex(x, 0, z);
      assert ( vtx < m_num_verts );

      note_idx = m_note_n[ m_note_plane ];
      SetValL ( BUF_NOTE, (vtx), note_idx, m_note_plane );
      m_note_n[ m_note_plane ]++;

      if ((m_res.y-1) != 0) {

        vtx = getVertex(x, m_res.y-1, z);
        assert ( vtx < m_num_verts );

        note_idx = m_note_n[ m_note_plane ];
        SetValL ( BUF_NOTE, (vtx), note_idx, m_note_plane );
        m_note_n[ m_note_plane ]++;
      }

    }
  }

  for (x=1; x<(m_res.x-1); x++) {
    for (y=1; y<(m_res.y-1); y++) {

      vtx = getVertex(x, y, 0);
      assert ( vtx < m_num_verts );

      note_idx = m_note_n[ m_note_plane ];
      SetValL ( BUF_NOTE, (vtx), note_idx, m_note_plane );
      m_note_n[ m_note_plane ]++;

      if ((m_res.z-1) != 0) {

        vtx = getVertex(x, y, m_res.z-1);
        assert ( vtx < m_num_verts );

        note_idx = m_note_n[ m_note_plane ];
        SetValL ( BUF_NOTE, (vtx), note_idx, m_note_plane );
        m_note_n[ m_note_plane ]++;
      }

    }
  }

  // propagate constraints
  // and cull boundary tile values
  //
  ret = cellConstraintPropagate();

  PERF_POP();

  return ret;
}

// To speed up the 'collapse' propagation, two
// auxiliary data structures are stored, one a copy
// of the grid x dim that holds a 'note' about whether
// it's been accessed or not, and a list of vertices
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
void BeliefPropagation::cellFillVisitedNeighbor(uint64_t vtx, int32_t note_plane ) {

  int64_t i, nei_vtx;

  Vector3DI jp = getVertexPos(vtx);

  for (i=0; i<getNumNeighbors(vtx); i++) {
    nei_vtx  = getNeighbor(vtx, jp, i);
    if (nei_vtx<0) { continue; }
    if (getValL ( BUF_VISITED, nei_vtx ) != 0) { continue; }

    int32_t note_idx = m_note_n [ note_plane ];
    SetValL ( BUF_NOTE, nei_vtx, note_idx, note_plane );
    SetValL ( BUF_VISITED, 1, nei_vtx );
    m_note_n[ note_plane ]++;
  }

}

// accelerated version
void BeliefPropagation::cellFillVisitedNeighborFast (Vector3DI jp, uint64_t vtx, int32_t note_plane ) {

  int64_t i, nei_vtx;
  int32_t note_idx;

  for (i=0; i < m_num_nbrs; i++) {
    nei_vtx  = getNeighbor(vtx, jp, i);
    if (nei_vtx < 0) { continue; }
    if (getValL ( BUF_VISITED, nei_vtx ) != 0) { continue; }

    note_idx = m_note_n [ note_plane ];
    SetValL ( BUF_NOTE, nei_vtx, note_idx, note_plane );
    SetValL ( BUF_VISITED, 1, nei_vtx );
    m_note_n[ note_plane ]++;
  }

}

int BeliefPropagation::cellFillVisitedSingle(uint64_t vtx, int32_t note_plane) {

  if (getValL( BUF_VISITED, vtx ) != 0) { return 0; }

  int32_t note_idx = m_note_n [ note_plane ];
  SetValL ( BUF_NOTE,    vtx, note_idx, note_plane );
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
// the current BUF_NOTE plane as this holds all vertices that were
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
  Vector3DI jp,
            _pos, _nei_pos;

  int resolved = 0;

  // vis prep for notes
  if (op.viz_opt == VIZ_NOTES) {
    PrepareVisualization ();
  }

  // perf: num neighbors is constantant on dimension
  int num_nbrs = getNumNeighbors(0);

  PERF_PUSH ( "cellConstrProp" );

  // cull noted cells tagged as changed
  //
  while (still_culling) {

    // printf ( "notes: %d\n", m_note_n[ m_note_plane ] );

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

        for (i=0; i < m_num_nbrs; i++) {

          if (getValF( BUF_F, anch_b_val, boundary_tile, i ) < _eps) {

            nei_cell = getNeighbor(anch_cell, jp, i);
            if (nei_cell<0) {

                if (anch_n_tile==1) {

                  if ( (op.alg_run_opt == ALG_RUN_MMS) ||
                       (op.alg_run_opt == ALG_RUN_BREAKOUT) ) {
                    if (op.verbose >= VB_STEP) {
                      _pos = getVertexPos(anch_cell);
                      printf("# cellConstraintPropagate: conflict, "
                              "cell %i(%i,%i,%i) slated to remove last remaining tile (tile %s(%i) "
                              "conflicts with out of bounds neighbor %s(%i) dir %s(%d)) [ccp-block.0]\n",
                              (int)anch_cell,
                              (int)_pos.x, (int)_pos.y, (int)_pos.z,
                              m_tile_name[anch_b_val].c_str(),
                              (int)anch_b_val,
                              m_tile_name[boundary_tile].c_str(), (int)boundary_tile,
                              m_dir_desc[i].c_str(), (int)i);
                    }
                  }
                  else if (op.verbose >= VB_ERROR ) {
                    _pos = getVertexPos(anch_cell);
                    printf("# BeliefPropagation::cellConstraintPropagate: conflict, "
                            "cell %i(%i,%i,%i), slated to remove last remaining tile (tile %s(%i) "
                            "conflicts with out of bounds neighbor %s(%i) dir %s(%d))\n",
                            (int)anch_cell,
                            (int)_pos.x, (int)_pos.y, (int)_pos.z,
                            m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                            m_tile_name[boundary_tile].c_str(), (int)boundary_tile,
                            m_dir_desc[i].c_str(), (int)i);
                  }

                  // unwind
                  //
                  unfillVisited (1 - m_note_plane );

                  PERF_POP();
                  return -1;
                }

                tile_valid = 0;

                //if (op.verbose >= VB_INTRASTEP ) {
                if (op.verbose >= VB_DEBUG) {
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

                  //if (op.verbose >= VB_INTRASTEP ) {
                  if (op.verbose >= VB_DEBUG) {
                    printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i [cp.0]\n",
                        (int)op.cur_iter,
                        (int)anch_cell,
                        (int)jp.x, (int)jp.y, (int)jp.z,
                        (int)getValI( BUF_TILE_IDX, 0, anch_cell ) );
                  }

                }

                cellFillVisitedNeighborFast ( jp, anch_cell, 1 - m_note_plane );

                anch_b_idx--;
                anch_n_tile--;

                break;

            } // nei_cell<0
          } // BUF_F < eps

        }  // endfor each nbr of anch_cell


        if (!tile_valid) { continue; }

        // Test for at least one valid neighbor from the anchor point.
        // That is, for each anchor cell and tile value, make sure
        // there is at least one "admissible" tile in the appropriate
        // direction by checking the BUF_F table.
        //
        for (i=0; i < m_num_nbrs; i++) {
          nei_cell = getNeighbor(anch_cell, jp, i);

          if (nei_cell<0) { continue; }

          anch_has_valid_conn = 0;

          // innermost loop: for all cells B, all nbrs A, B->A, all tiles, check rule func F
          /*nei_n_tile = getValI ( BUF_TILE_IDX_N, nei_cell );
          for (nei_a_idx=0; nei_a_idx < nei_n_tile; nei_a_idx++) {
            nei_a_val = getValI ( BUF_TILE_IDX, nei_a_idx, nei_cell );

            if (getValF( BUF_F, anch_b_val, nei_a_val, i ) > _eps) {
              anch_has_valid_conn = 1;
              break;
            }
          }*/

          //--- fast version
          nei_n_tile = getValI ( BUF_TILE_IDX_N, nei_cell );
          int32_t* nei_a_ptr = (int32_t*) getPtr ( BUF_TILE_IDX, 0, nei_cell );
          int32_t* nei_a_end = nei_a_ptr + nei_n_tile;
          for (; nei_a_ptr < nei_a_end; nei_a_ptr++) {
              if (getValF( BUF_F, anch_b_val, *nei_a_ptr, i ) > _eps) {    // nei_a_val is random access, so no accel of BUF_F ptr
                  anch_has_valid_conn = 1;
                  break;
              }
          }
          //---


          if (!anch_has_valid_conn) {
            if (anch_n_tile==1) {

              nei_a_val = *nei_a_ptr;

              if ((op.alg_run_opt == ALG_RUN_MMS) ||
                  (op.alg_run_opt == ALG_RUN_BREAKOUT)) {
                if (op.verbose >= VB_STEP) {
                  _pos = getVertexPos(anch_cell);
                  _nei_pos = getVertexPos(nei_cell);
                  printf("# cellConstraintPropagate: conflict, "
                          "cell %i(%i,%i,%i) slated to remove last remaining tile (tile %s(%i) "
                          "conflicts with neighbor cell %i(%i,%i,%i), tile %s(%i) dir %s(%d)) [ccp-block.1]\n",
                          (int)anch_cell,
                          (int)_pos.x, (int)_pos.y, (int)_pos.z,
                          m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                          (int)nei_cell,
                          (int)_nei_pos.x, (int)_nei_pos.y, (int)_nei_pos.z,
                          m_tile_name[nei_a_val].c_str(), (int)nei_a_val,
                          m_dir_desc[i].c_str(), (int)i);
                }
              }
              else if (op.verbose >= VB_ERROR ) {
                _pos = getVertexPos(anch_cell);
                _nei_pos = getVertexPos(nei_cell);
                printf("# BeliefPropagation::cellConstraintPropagate: conflict, "
                        "cell %i(%i,%i,%i) slated to rmove last remaining tile (tile %s(%i) "
                        "conflicts with neighbor cell %i(%i,%i,%i), tile %s(%i) dir %s(%d)) [e0]\n",
                        (int)anch_cell,
                        (int)_pos.x, (int)_pos.y, (int)_pos.z,
                        m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                        (int)nei_cell,
                        (int)_nei_pos.x, (int)_nei_pos.y, (int)_nei_pos.z,
                        m_tile_name[nei_a_val].c_str(), (int)nei_a_val,
                        m_dir_desc[i].c_str(), (int)i);
              }

              // unwind
              //
              unfillVisited (1 - m_note_plane );

              PERF_POP();
              return -1;
            }

            tile_valid = 0;

            //if (op.verbose >= VB_INTRASTEP ) {
            if (op.verbose >= VB_DEBUG) {
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

              //if (op.verbose >= VB_INTRASTEP ) {
              if (op.verbose >= VB_DEBUG) {
                printf("RESOLVE it:%i cell:%i;[%i,%i,%i] tile:%i [cp.1]\n",
                    (int)op.cur_iter,
                    (int)anch_cell,
                    (int)jp.x, (int)jp.y, (int)jp.z,
                    (int)getValI( BUF_TILE_IDX, 0, anch_cell ) );
              }


            }

            cellFillVisitedNeighborFast (jp, anch_cell, 1 - m_note_plane );
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

  PERF_POP();
  return 0;
}

int BeliefPropagation::btPush(int64_t bt_cur_stack_idx, int64_t cell, int64_t tile_val) {

  int64_t bt_idx=0;

  bt_idx = getValI( BUF_BT_IDX, bt_cur_stack_idx );

  //DEBUG
  printf("btPush cell: %i, tile_val: %i [%i,%i]\n",
      (int)cell, (int)tile_val, (int)bt_idx, (int)(bt_idx+1));

  SetValI( BUF_BT, cell, bt_idx );
  bt_idx++;

  SetValI( BUF_BT, (int64_t)tile_val , bt_idx );
  bt_idx++;

  SetValI( BUF_BT_IDX, bt_idx, bt_cur_stack_idx );

  return 0;
}

int BeliefPropagation::btUnwind(int64_t bt_cur_stack_idx) {

  int64_t n=0,
          cell = 0,
          tile_val = 0;

  int64_t bt_idx=0,
          bt_idx_st=0,
          bt_idx_en=0;

  if (bt_cur_stack_idx > 0) {
    bt_idx_st = getValI( BUF_BT_IDX, bt_cur_stack_idx-1 );
  }
  bt_idx_en = getValI( BUF_BT_IDX, bt_cur_stack_idx );

  for (bt_idx=bt_idx_st; bt_idx < bt_idx_en; bt_idx+=2) {
    cell      = getValI( BUF_BT, bt_idx+0 );
    tile_val  = getValI( BUF_BT, bt_idx+1 );

    n = getValI( BUF_TILE_IDX_N, cell );

    //DEBUG
    printf("btUnwind cell: %i, tile_val %i (n %i)\n",
        (int)cell, (int)tile_val, (int)n);


    SetValI( BUF_TILE_IDX, tile_val, n, cell );
    n++;
    SetValI( BUF_TILE_IDX_N, n, cell );
  }

  // reset BT_IDX buf
  //
  SetValI( BUF_BT_IDX, bt_idx_st, bt_cur_stack_idx );

  return 0;
}

// fix tile_val at cell and propagate constraints.
// BUF_TILE_IDX might get shuffled as a result
// but should hold the same values as it did
// before the call.
//
// Uses BUF_BT and BUF_BT_IDX as storage to know
// what to unwind after the attempt
//
//  1 - contradiction
//  0 - no contradiction
// -1 - error
//
int BeliefPropagation::cellConstraintPropagate_lookahead(int64_t cell, int32_t tile_val) {

  int still_culling=1, i;

  int64_t note_idx, anch_cell, nei_cell;
  int64_t nei_n_tile, nei_a_idx, nei_a_val;

  int64_t v,
          idx,
          anch_n_tile,
          anch_idx,
          anch_b_idx,
          anch_b_val;
  int anch_has_valid_conn = 0;

  int boundary_tile = 0, tile_valid = 0;
  int gn_idx = 0;

  float _eps = op.eps_zero;
  Vector3DI jp;

  int resolved = 0,
      contradiction = 0;

  int64_t bt_idx = 0;

  // find the index of the tile we'r about to fix
  //
  anch_n_tile = getValI( BUF_TILE_IDX_N, cell );
  if (anch_n_tile <= 1) { return 0; }
  for (anch_idx=0; anch_idx < anch_n_tile; anch_idx++) {
    if ( getValI( BUF_TILE_IDX, anch_idx, cell ) == tile_val ) {

      //DEBUG
      printf("### FOUND: cell:%i anch_idx:%i tile_val:%i\n",
          (int)cell, (int)anch_idx, (int)tile_val);
      break;
    }
  }
  if (anch_idx == anch_n_tile) {

    printf("### ERROR! NOT FOUND cell:%i tile_val:%i (/%i)\n",
        (int)cell, (int)tile_val, (int)anch_n_tile);

    return -1;
  }

  // remember which tiles we're about to remove from the
  // collapse
  //
  for ( idx=0; idx < anch_n_tile; idx++) {
    if (idx == anch_idx) { continue; }
    v = getValI( BUF_TILE_IDX, idx, cell );
    btPush( 0, cell, v );
  }

  // collapse the current cell for tile_val
  //
  tileIdxCollapse( cell, anch_idx );

  m_note_n[ m_note_plane ]      = 0;
  m_note_n[ 1 - m_note_plane ]  = 0;
  cellFillVisitedNeighbor ( cell, m_note_plane );
  unfillVisited ( m_note_plane );

  int num_nbrs = getNumNeighbors(0);

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

              if (op.verbose >= VB_DEBUG) {

                printf("# BeliefPropagation::cellConstraintPropagate_lookahead: contradiction, "
                        "cell %i slated to rmove last remaining tile (tile %s(%i) "
                        "conflicts with neighbor cell %i, tile %s(%i) dir %s(%d)) [la.0]\n",
                        (int)anch_cell,
                        m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                        (int)nei_cell,
                        m_tile_name[nei_a_val].c_str(), (int)nei_a_val,
                        m_dir_desc[i].c_str(), (int)i);
              }

              contradiction = 1;
              break;
            }

            tile_valid = 0;

            if (op.verbose >= VB_DEBUG) {
              printf("# bt_remove it:%i cell:%i;[%i,%i,%i] tile %i (boundary nei, tile:%i, dir:%i(%s)) [cp.0]\n",
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
                printf("bt_resolve it:%i cell:%i;[%i,%i,%i] tile:%i [cp.0]\n",
                    (int)op.cur_iter,
                    (int)anch_cell,
                    (int)jp.x, (int)jp.y, (int)jp.z,
                    (int)getValI( BUF_TILE_IDX, 0, anch_cell ) );
              }

            }

            cellFillVisitedNeighbor (anch_cell, 1 - m_note_plane );

            anch_b_idx--;
            anch_n_tile--;

            break;
          }
        }

        if (contradiction) { break; }

        if (!tile_valid) { continue; }

        // Test for at least one valid neighbor from the anchor point.
        // That is, for each anchor cell and tile value, make sure
        // there is at least one "admissible" tile in the appropriate
        // direction by checking the BUF_F table.
        //
        for (i=0; i < num_nbrs; i++) {
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

              if (op.verbose >= VB_DEBUG ) {
                printf("# BeliefPropagation::cellConstraintPropagate_lookahead: contradiction, "
                        "cell %i slated to rmove last remaining tile (tile %s(%i) "
                        "conflicts with neighbor cell %i, tile %s(%i) dir %s(%d)) [la.1]\n",
                        (int)anch_cell,
                        m_tile_name[anch_b_val].c_str(), (int)anch_b_val,
                        (int)nei_cell,
                        m_tile_name[nei_a_val].c_str(), (int)nei_a_val,
                        m_dir_desc[i].c_str(), (int)i);
              }

              contradiction = 1;
              break;
            }

            tile_valid = 0;

            if (op.verbose >= VB_DEBUG ) {
              printf("# bt_remove it:%i cell:%i;[%i,%i,%i] tile %i (invalid conn dir:%i(%s), tile:%i) [cp.1]\n",
                  (int)op.cur_iter,
                  (int)anch_cell,
                  (int)jp.x, (int)jp.y, (int)jp.z,
                  (int)anch_b_val,
                  (int)i, (char *)m_dir_desc[i].c_str(), (int)nei_a_val);
            }

            v = getValI( BUF_TILE_IDX, anch_b_idx, anch_cell );
            btPush( 0, anch_cell, v);

            removeTileIdx(anch_cell, anch_b_idx);

            if ( getValI( BUF_TILE_IDX_N, anch_cell ) == 1 ) {
              resolved++;

              if (op.verbose >= VB_DEBUG ) {
                printf("bt_reolve it:%i cell:%i;[%i,%i,%i] tile:%i [cp.1]\n",
                    (int)op.cur_iter,
                    (int)anch_cell,
                    (int)jp.x, (int)jp.y, (int)jp.z,
                    (int)getValI( BUF_TILE_IDX, 0, anch_cell ) );
              }


            }

            cellFillVisitedNeighbor (anch_cell, 1 - m_note_plane );
            anch_b_idx--;
            anch_n_tile--;

            break;
          }

          if (contradiction) { break; }

        }

        if (contradiction) { break; }

        if (!tile_valid) { continue; }
      }

      if (contradiction) { break; }
    }

    unfillVisited (1 - m_note_plane );

    if (contradiction) { break; }

    if (m_note_n[ m_note_plane ] == 0) { still_culling = 0; }

    m_note_n[ m_note_plane ] = 0;
    m_note_plane  = 1 - m_note_plane ;
  }

  btUnwind(0);

  return contradiction;
}

