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

#ifndef DEF_BELIEF_PROPAGATION
#define DEF_BELIEF_PROPAGATION

#include <algorithm>
#include "mersenne.h"
#include "dataptr.h"

//extern "C" {
//#include "lib/svdlib.h"
//}

#include <Eigen/SVD>

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

#define BELIEF_PROPAGATION_VERSION "0.8.4"

#define VB_SUPPRESS     -1
#define VB_NONE         0
#define VB_EXPERIMENT   1
#define VB_MULTIRUN     2
#define VB_RUN          3
#define VB_STEP         4
#define VB_INTRASTEP    5
#define VB_DEBUG        6

// level at which errors should be reported
#define VB_ERROR        3

#define OPT_PTRS
#define OPT_MUPTR
#define OPT_FH
#define OPT_MUBOUND

#define OPT_BLOCK_NONE                -1
#define OPT_BLOCK_RANDOM_POS          0
#define OPT_BLOCK_RANDOM_POS_SIZE     1
#define OPT_BLOCK_SEQUENTIAL          2
#define OPT_BLOCK_MIN_ENTROPY         3
#define OPT_BLOCK_NOISY_MIN_ENTROPY   4
#define OPT_BLOCK_NOISY_MAX_ENTROPY   5

#define OPT_NOISE_FUNC_UNIFORM        0
#define OPT_NOISE_FUNC_POWER_LAW      1

#define MU_NOCOPY 0
#define MU_COPY   1

#define VIZ_NONE        0
#define VIZ_TILES_2D    1
#define VIZ_TILE0       2       // visualized resolved tile 0. similar to json output. no overhead. 
#define VIZ_TILECOUNT   3       // number of tiles available per cell. no overhead. 
#define VIZ_CONSTRAINT  4       // visualize remaining constraints. high overhead (eg. 5%)
#define VIZ_NOTES       5       // visualize notes. some overhead.
#define VIZ_ENTROPY     6

#define VIZ_BP_BELIEF   7       // BP only. max belief among available tiles. some overhead.
#define VIZ_BP_ENTROPY  8       // BP only
#define VIZ_BP_MU       9       // BP only
#define VIZ_BP_DMU      10       // BP only

// primary algorithm selector
#define ALG_BP                  0
#define ALG_BP_MIN              1
#define ALG_BP_MIN_WAVE         2
#define ALG_BP_MIN_RESIDUAL     3
#define ALG_WFC                 -1
#define ALG_MMS_SEQ             -2
#define ALG_MMS_RAND1           -3
#define ALG_MMS_RAND2           -4
#define ALG_BMS                 -5
#define ALG_BMS_MIN             -6
#define ALG_BMS_MIN_NOISE       -7
#define ALG_BMS_MAX_NOISE       -8

// algorithm settings
#define ALG_CELL_WFC            31
#define ALG_CELL_MMS            32 // Merrell's Model Synthesis, aka 'block wfc', aka Modify in Place Model Synthesis
#define ALG_CELL_ANY            33
#define ALG_CELL_MIN_ENTROPY    34
#define ALG_CELL_BREAKOUT       35

#define ALG_TILE_MAX_BELIEF     36

#define ALG_RUN_VANILLA         37
#define ALG_RUN_RESIDUAL        38
#define ALG_RUN_WFC             39
#define ALG_RUN_MMS             40
#define ALG_RUN_BACKTRACK       41
#define ALG_RUN_BREAKOUT        42

#define ALG_ACCEL_NONE          0
#define ALG_ACCEL_WAVE          1

// memory locations: cpu + (z*mUseRY + y)*mUseRX + x

// static buffers (input)                                                                                               // Allocation (B=num_vals)
#define BUF_G           1     // tile weights,  weight, all values - beliefprop, G(a) vector                            // <B, 1, 1>
#define BUF_F           2     // tile rules,    rule-to-rule - beliefprop, F(a,b) vector - <src, dest, direction>       // <B, B, 6>

// dynamic buffers
#define BUF_MU          3     // message prob,  6*B,        all verts - beliefprop, mu{i,j}(a,b) vector (B=# values)    // <6, B, num_vert>
#define BUF_MU_NXT      4     // message prob', 6*B,        all verts - beliefprop, mu'{i,j}(a,b) vector (B=# values)   // <6, B, num_vert>
#define BUF_TILE_IDX    5     // tile indexes,  val list,   all verts - beliefprof                                      // <B, num_vert, 1>
#define BUF_TILE_IDX_N  6     // # of tile idx, 1x int,     all verts - beliefprof                                      // <num_vert, 1, 1>

// scratch buffers
#define BUF_H           7     // temporary,     val list,   single vert - beliefprop                                    // <B, 1, 1>
#define BUF_BELIEF      8     // temporary,     val list,   single vert - beliefprop                                    // <B, 1, 1>
#define BUF_VISITED     9     // temporary,     1x int,     all verts - beliefprop                                      // <num_vert, 1, 1>
#define BUF_NOTE        10    // which cells,   2x int,     all verts (of size cell count (i32))                        // <num_vert, 2, 1>
#define BUF_VIZ         11    // vizualization, 1x float,   all verts                                                   // <num_vert, 1, 1>
#define BUF_TILES       12    // maxb tile,     1x int,     all verts                                                   // <num_vert, 1, 1>
#define BUF_B           13    // maxb value,    1x float,   all verts                                                   // <num_vert, 1, 1>
#define BUF_C           14    // num constraint,1x int,     all verts                                                   // <num_vert, 1, 1>

// svd & residual bp
#define BUF_SVD_U       16    //                                                                                        // <B,  B*, 6>
#define BUF_SVD_Vt      17    //                                                                                        // <B*, B,  6>
#define BUF_SVD_VEC     18    //                                                                                        // <B*, 1,  1>

// auxiliary buffers for residual belief propagation
//
// BUF_RESIDUE_HEAP         : heap of absolute differences of mu and mu_nxt (float)
// BUF_RESIDUE_HEAP_CELL_BP : back pointer of heap value location in CELL_HEAP (in64_t)
// BUF_RESIDUE_CELL_HEAP    : mapping of cell (and direction, value) to heap position (in64_t).
//                            That is, mapping of mu index to position in heap
//
// All sizes should be (Vol) * 6 * B.
// That is, {volume} x {#neighbors} x {#values} : (dim[0]*dim[1]*dim[2] * 6 * B).
//
// All these structures are for book keeping so that we can get the maximum difference of
// mu and mu_nxt in addition to allowing arbitrary updates on other cells.
//
// The basic residual bp update step will fetch the maximum difference, copy the mu value
// from the mu_nxt buffer to the mu buffer, then update neighboring cells by updating their
// mu_nxt values, updating the residue_heap along the way.
//
#define BUF_RESIDUE_HEAP          19    //                                                                               // <6*B*num_vert, 1, 1>
#define BUF_RESIDUE_HEAP_CELL_BP  20    //                                                                               // <6*B*num_vert, 1, 1>
#define BUF_RESIDUE_CELL_HEAP     21    //                                                                               // <6*B*num_vert, 1, 1>

#define BUF_BT                    22    // backtrack list,      val list, interleaved cell/tile val - backtrack wfc     // <2*B*num_vert, 1, 1>
#define BUF_BT_IDX                23    // backtrack stack ptr, val list, index pointer into BUF_BT - backtrack wfc     // <B*num_vert, 1, 1>

#define BUF_BLOCK                 24    // saved block tile,    1x int,   all verts (?) - MMS                           // <num_vert, 1, 1>

//--
//
// prefatory state for breakout model synthesis
//
#define BUF_PREFATORY_TILE_IDX    25    // soften tile idxs,    val list, all verts (?) - breakout model synth    // <B, num_vert, 1>
#define BUF_PREFATORY_TILE_IDX_N  26    // # soften tile idxs,  1x int,   all verts (?) - breakout model synth    // <num_vert, 1, 1>

#define BUF_SAVE_TILE_IDX         27    // save tile idxss,     val list, all verts (?) - breakout model synth    // <B, num_vert, 1>
#define BUF_SAVE_TILE_IDX_N       28    // # save tile idxs,    1x int,   all verts (?) - breakout model synth    // <num_vert, 1, 1>
//--

#define BUF_CELL_ENTROPY          29    // cell entropy buf,    1x float, all verts (?) - breakout model synth    // <num_vert, 1, 1>
#define BUF_BLOCK_ENTROPY         30    // block entropy buf,   1x float, all verts (?) - breakout model synth    // <num_vert, 1, 1>


#define BUF_MAX         32      // this is buffer count limit. increase if more needed.


// #define NOUT        -134217728       // 512x512x512 limit
#define NOUT        -1073741824

// Belief propagation - options

typedef struct _bp_opt_t {

  float     alpha;

  int       X, Y, Z, D;

  std::string name_fn;
  std::string rule_fn;

  std::string tileset_fn,
              tilemap_fn,
              tileobj_fn,
              tilefilter_fn,
              outstl_fn;

  int32_t   tileset_stride_x,
            tileset_stride_y;
  int32_t   tileset_margin,
            tileset_spacing;
  int32_t   tileset_width,
            tileset_height;
  int       tiled_reverse_y;

  std::string  constraint_cmd;
  std::string  admissible_tile_range_cmd;

  std::vector< int32_t > cull_list;

  int       seed;

  float     step_rate;

  float     eps_converge,
            eps_converge_beg,
            eps_converge_end,
            eps_zero;

  int32_t   sub_block[3],
            block_size[3],
            block_idx[3],
            sub_block_range[3][2],
            seq_iter;
  
  int32_t   block_schedule;
  bool      adaptive_soften;
  int       jitter_block;
  int       solved_tile_cnt;
  int       entropy_bias;
  float     entropy_pct;
  float     entropy_flip;
  float     entropy_radius;
  int       wrap_count;
  
  Vector3DF prev_block_centroid;
  float     prev_block_mass;

  Vector3DF curr_block_centroid;
  float     curr_block_mass;

  int64_t   step_cb;
  float     state_info_d;
  int64_t   state_info_iter;

  int64_t   index_heap_size;

  int32_t   cur_run;
  int32_t   max_run;

  int32_t   cur_iter;
  int32_t   max_iter;

  int32_t   cur_step;
  int32_t   max_step;

  int32_t   alg_idx;            // ALG_RUN_VANILLA or ALG_RUN_RESIDUE
  int32_t   alg_cell_opt;       // ALG_CELL_ANY, ALG_CELL_MIN_ENTROPY
  int32_t   alg_tile_opt;       // ALG_TILE_MAX_BELIEF
  int32_t   alg_run_opt;
  int32_t   alg_accel;

  int32_t   viz_opt;            // VIS_NONE, VIS_MU, VIS_BELIEF, etc..

  bool      use_cuda;
  int       use_svd;
  int       use_checkerboard;

  int       use_lookahead;

  float     block_noise_coefficient,
            block_noise_alpha;
  int32_t   block_noise_func;

  float     wfc_noise_coefficient,
            wfc_noise_alpha;
  int32_t   wfc_noise_func;

  //int32_t   experiment_idx;
  std::string experiment_str;

  // As a general rule of thumb, the verbosity is:
  //
  // 0  - NONE
  //        no output (default),
  //        unless error (which should go to stderr?)
  //
  // 1  - SUMMARY
  //        output summary information or other output
  //        at the end of a completed run
  //
  // 2  - RUN
  //        output summary information at end
  //        of each step
  //
  // 3  - STEP
  //        output information intra-step
  //
  // 4  - DEBUG
  //        catchall for debug/anything printing
  //
  int       verbose;

} bp_opt_t;

// Belief propagation - constraint ops

typedef struct constraint_op_type {
  char op;
  std::vector< int > dim_range;
  std::vector< int > tile_range;
} constraint_op_t;


// Belief propagation - statistics

typedef struct _bp_stat_type {

  char    enabled;
  int     post;

  float   max_belief;

  int     upper_step;
  double  avg_step,
          second_moment_step;

  float   eps_curr;

  float   max_dmu,
          ave_mu,
          ave_dmu;

  int64_t num_culled,
          num_collapsed,
          num_chosen;

  int     iter_resolved,
          total_resolved;

  int     total_block_cnt,
          num_block_retry,
          num_block_fail,
          num_block_success,
          num_block_fails,
          num_soften;
  float   ave_block_try;

  float   elapsed_time;

  int64_t constraints; 
  bool    success;

  bool    instr;
  float   time_boundary,
          time_normalize,
          time_bp,
          time_viz,
          time_maxdiff,
          time_updatemu;

  // number of tiles in a cell (more than one)
  //
  float   occupancy_mean,
          occupancy_mode,
          occupancy_second_moment;

  // cluster is yet to be defined but an initial
  // guess is that it's the size of a number of
  // cells of 'forced' tiles. For example,
  // force a tile, this might have a cascade
  // effect on other tiles, the size of that
  // cascade would be a cluster.
  // Needs some thinking and might need
  // some more infrastructure to implement
  //
  float   cluster_mean,
          cluster_mode,
          cluster_second_moment;

} bp_stat_t;

// Belief propagation - experiments

typedef struct _bp_expr_type {

  std::string name;

  int         num_expr;
  int         num_run;

  Vector3DI   grid_min, grid_max;

  int         maxstep_min, maxstep_max;
  float       steprate_min, steprate_max;
  float       eps_min, eps_max;

} bp_expr_t;



// Belief propagation

class BeliefPropagation {
public:
  BeliefPropagation() {
    int i,j;

    op.verbose = VB_NONE;

    default_opts();

    m_return = 0;

    op.sub_block[0] = 0;
    op.sub_block[1] = 0;
    op.sub_block[2] = 0;
    op.block_size[0] = 0;
    op.block_size[1] = 0;
    op.block_size[2] = 0;
    op.block_idx[0] = 0;
    op.block_idx[1] = 0;
    op.block_idx[2] = 0;

    m_block_fail_count = 0;
    m_block_retry_limit = 10;       // default: 10 retries

    op.seq_iter = 0;
    op.adaptive_soften = true;     // default: no adaptive soften
    op.jitter_block = 2;            // default: no block jitter
    op.entropy_bias = 1;            // default: no mass entropy bias    
    op.entropy_pct = 0.5;
    op.entropy_flip = 0.8;
    

    op.block_noise_coefficient = 1.0/128.0;
    op.block_noise_alpha = -2.0;
    op.block_noise_func = OPT_NOISE_FUNC_POWER_LAW;

    op.wfc_noise_coefficient = 1.0/128.0;
    op.wfc_noise_alpha = -2.0;
    op.wfc_noise_func = OPT_NOISE_FUNC_POWER_LAW;
    
    //op.experiment_idx = -1;

    //m_breakout_block_fail_count = 0;
    //m_breakout_soften_limit = 10;

    

    op.viz_opt = VIZ_TILE0;

    for (i=0; i<3; i++) {
      for (j=0; j<2; j++) {
        op.sub_block_range[i][j] = 0;
      }
    }

  };

  //------------------------ high level API

  void      SelectAlgorithm ( int alg_idx );

  int       default_opts ();

  void      reset ();

  int       init( int, int, int,
              std::vector< std::string  >           tile_name_list,
              std::vector< float >                  tile_weight_list,
              std::vector< std::vector < float > >  rule_list );

  int       start();
  int       finish( int final_ret );
  void      advance_seed ( int amt=1 );

  void      jitterBlock ();
  void      getBlockCenterOfMass (Vector3DF& center, float& mass);

  int       RealizePre();
  int       RealizeIter();
  int       RealizeStep();
  int       RealizePost();
  int       Realize();

  void      getCurrentBlock ( Vector3DI& bmin, Vector3DI& bmax );
  Vector3DI getErrorCell () { return getVertexPos(m_error_cell);}
  Vector3DI getErrorCause () { return getVertexPos(m_error_cause);}

  int       CollapseAndPropagate (int64_t& cell, int32_t& tile, int32_t& tile_idx );
  int       CheckConstraints ( int64_t p );
  int       CheckConstraints ();

  void      SetVis (int viz_opt);

  //----
  void _saveTileIdx(void);
  void _restoreTileIdx(void);

  std::string getAlgName (int alg) {
    std::string name;
    switch (alg) {
    case ALG_BP:          name="bp";   break;
    case ALG_BP_MIN:      name="bpn"; break;  
    case ALG_BP_MIN_WAVE: name="bpnw"; break;  
    case ALG_BP_MIN_RESIDUAL: name="bpnr"; break;  
    case ALG_WFC:         name="wfc"; break;  
    case ALG_MMS_SEQ :     name="mms"; break;  
    case ALG_MMS_RAND1:    name="mmsr1"; break;  
    case ALG_MMS_RAND2:    name="mmsr2"; break;  
    case ALG_BMS:           name="bms"; break;  
    case ALG_BMS_MIN :      name="bmsm"; break;       
    case ALG_BMS_MIN_NOISE: name="bmsmn"; break;  
    case ALG_BMS_MAX_NOISE: name="bmsxn"; break;
    };
    return name;
  }

  //------------------------ belief propagation, mid-level API

  int   init_SVD(void);

  int   filter_constraint(std::vector< std::vector< int32_t > > &constraint_list);

  void  gp_state_print();

  int   _pick_tile(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   _pick_tile_max_belief(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   _pick_tile_min_belief(int64_t anch_cell, int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);
  int   _pick_tile_pdf(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);

  void  init_dir_desc();
  float  step(int update_mu);
  float  step_residue(int32_t idir, int64_t cell, int32_t tile);

  //---------------------------------------- residual belief propagation
  //
  int64_t getMuIdx( int32_t idir, int64_t cell, int32_t tile );
  int64_t getMuPos( int64_t idx, int32_t *idir, int64_t *cell, int32_t *tile );

  void    indexHeap_init(void);
  void    indexHeap_swap(int64_t heap_idx_a, int64_t heap_idx_b);
  int32_t indexHeap_push(float val);

  void    indexHeap_update(int64_t heap_idx, float val);
  void    indexHeap_update_mu_idx(int64_t mu_idx, float val);
  void    indexHeap_update_mu_pos(int32_t idir, int64_t cell, int32_t tile, float val);

  int64_t indexHeap_peek(int64_t *mu_idx, float *val);
  int64_t indexHeap_peek_mu_pos(int32_t *idir, int64_t *cell, int32_t *tile_val, float *val);


  int32_t indexHeap_consistency(void);
  int32_t indexHeap_mu_consistency(void);

  void    indexHeap_debug_print(void);


  //----------------------- belief propagation (low-level)

  void  ConstructStaticBufs ();
  void  ConstructDynamicBufs ();
  void  ConstructTempBufs ();
  void  ConstructSVDBufs ();

  float BeliefProp();
  float BeliefProp_svd ();

  float BeliefProp_cell_residue(int64_t);
  float BeliefProp_cell_residue_svd(int64_t);

  void  UpdateMU ();

  float getVertexBelief ( uint64_t j );
  float _getVertexBelief ( uint64_t j );

  int   getMaxBeliefTile ( uint64_t j );

  void  cellUpdateBelief(int64_t anch_cell);
  int   chooseMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   chooseMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);

  int   chooseMinEntropy(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);
  int   chooseMinEntropyMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   chooseMinEntropyMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);

  int   chooseMinEntropyWithinBlock(std::vector<int64_t> &block_bound, int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);
  int   chooseMinEntropyWithinNoisyBlock(std::vector<int64_t> &block_bound, int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);

  void  WriteBoundaryMUbuf(int buf_id);
  void  TransferBoundaryMU (int src_id, int dst_id);
  void  InitializeDMU (int buf_id=BUF_MU_NXT);
  float MaxDiffMU();
  float MaxDiffMUCellTile(float *max_diff, int64_t *max_cell, int64_t *max_tile_idx, int64_t *max_dir_idx);

  void  RandomizeMU ();

  void  NormalizeMU ();
  void  NormalizeMU (int id);
  void  NormalizeMU_cell_residue (int buf_id, int64_t cell);

  int filterKeep(uint64_t pos, std::vector<int32_t> &tile_id);
  int filterDiscard(uint64_t pos, std::vector<int32_t> &tile_id);
  int filterAdd(uint64_t pos, std::vector<int32_t> &tile_id);
  int32_t tileName2ID (std::string &tile_name);
  int32_t tileName2ID (char *);

  // used for visualization
  //
  void  PrepareVisualization ();
  
  void  ComputeTile0Field();
  void  ComputeNoteField ();
  void  ComputeBP_BeliefField ();  
  void  ComputeBP_DiffMUField ();  
  

  int   ComputeCellEntropy();
  int   ComputeBlockEntropy(int32_t reuse_cell_entropy=0);



  // non "strict" bp functions but helpful still
  //
  int   CullBoundary();
  int   cellConstraintPropagate();
  void  cellFillVisitedNeighbor(uint64_t vtx, int32_t note_idx); 
  void  cellFillVisitedNeighborFast (Vector3DI jp, uint64_t vtx,  int32_t note_idx); 
  int   cellFillVisitedSingle(uint64_t vtx, int32_t note_idx);

  void  cellReturnToPrefatory ( int64_t cell );

  int   tileIdxCollapse(uint64_t pos, int32_t tile_idx);
  int   tileIdxRemove(uint64_t pos, int32_t tile_idx);

  // note_idx is the 'plane' of BUF_NOTE to unwind
  //
  void  unfillVisited(int32_t note_idx);
  int   removeTileIdx(int64_t anch_cell, int32_t anch_tile_idx);
  int   sanityAccessed();

  // sanity functions to help debug breakout model synthesis
  //
  int sanityBreakoutRealizedBlock(std::vector<int32_t> &block);
  int sanityBreakoutSavedTileGrid(void);
  int sanityBreakoutStatBlock(std::vector<int32_t> &_debug_stat, int32_t *_bounds);
  int sanityGroundState(void);


  // number of fixed tiles (only 1 tile in cell)
  //
  int64_t numFixed();
  int pickMinEntropyNoiseBlock(void);
  int pickMaxEntropyNoiseBlock(void);
  //float pickNoiseFunc(void);
  float pickNoiseFunc(int32_t noise_func, float noise_coefficient, float noise_alpha);

  //----------------------- visualization

  Vector4DF getVisSample ( int64_t v );

  //------------------------ memory management

  void          AllocBuf (int id, char dt, uint64_t cntx=1, uint64_t cnty=1, uint64_t cntz=1 );
  void          ZeroBuf (int id);

  // 3D spatial neighbor functions
  //
  int64_t       getNeighbor(uint64_t j, int nbr);
  int64_t       getNeighbor(uint64_t j, Vector3DI jp, int nbr);

  Vector3DI     getVertexPos(int64_t j);
  int64_t       getVertex(int x, int y, int z);
  int           getTilesAtVertex ( int64_t vtx );
  int           getOppositeDir(int nbr)  { return m_dir_inv[nbr]; }

  //----------------------- new accessor functions

  // caller does type casting
  //
  inline void*  getPtr(int id, int x=0, int y=0, int z=0)     {return (void*) m_buf[id].getPtr (x, y, z);}

  inline int32_t getValI(int id, int x=0, int y=0, int z=0)            {return *(int32_t*) m_buf[id].getPtr (x, y, z);}
  inline int64_t getValL(int id, int x=0, int y=0, int z=0)            {return *(int64_t*) m_buf[id].getPtr (x, y, z);}
  inline float   getValF(int id, int x=0, int y=0, int z=0)            {return *(float*) m_buf[id].getPtr (x, y, z);}

  inline void   SetValI(int id, int32_t val, int x, int y=0, int z=0)     {*(int32_t*) m_buf[id].getPtr(x, y, z) = val;}
  inline void   SetValL(int id, int64_t val, int x, int y=0, int z=0)     {*(int64_t*) m_buf[id].getPtr(x, y, z) = val;}
  inline void   SetValF(int id, float val, int x, int y=0, int z=0)     {*(float*)   m_buf[id].getPtr(x, y, z) = val;}

  inline int    getNumNeighbors(int j)        {return (m_bpres.z==1) ? 4 : 6;}

  inline int    getNumValues(int j)          {return m_num_values;}
  inline int    getNumVerts()            {return m_num_verts;}

  //----------------------- options & stat accessors

  void          ResetStats ();

  std::string   getStatMessage ();

  std::string   getStatCSV (int mode=0);

  bp_opt_t*     get_opt()              { return &op; }
  bp_stat_t*    get_stat()             { return &st; }

  int           getStep()              { return op.cur_step; }
  int           getVerbose()           { return op.verbose; }
  float         getLinearEps ();
  float         getElapsedTime()       { return st.elapsed_time; }
  void          setConverge ( bp_opt_t* op, float c );

  //-------------------------- wave function collapse
  int   wfc();
  int   wfc_start();
  int   wfc_step(int64_t it);

  //-------------------------- backtracking (wip)

  int cellConstraintPropagate_lookahead(int64_t, int32_t);
  int btPush(int64_t, int64_t, int64_t);
  int btUnwind(int64_t);


  //-------------------------- debugging functions

  // helper arrays and functions for ease of testing and simple use
  //
  void  debugPrint();
  void  debugPrintTerse(int buf_idx=BUF_TILE_IDX);
  void  debugPrintC();
  void  debugPrintS();
  void  debugPrintMU();

  void  debugPrintCellEntropy();
  void  debugPrintBlockEntropy();

  void  debugInspect (Vector3DI pos, int tile);

  // run time statistics and other information
  //
  // void    UpdateRunTimeStat(int64_t num_step);


  //------------------------- member variables

  // primary data stored in buffers
  //
  DataPtr       m_buf[ BUF_MAX ];

  // problem size
  //
  int64_t       m_num_verts;    // Xi = 0..X (graph domain)
  int64_t       m_num_values;   //  B = 0..Bm-1 (value domain)
  Vector3DI     m_bpres;        // 3D spatial belief prop res

  Vector3DI     m_res;          // volume res

  std::vector< std::string > m_tile_name;
  std::vector< std::string > m_dir_desc;
  int           m_dir_inv[6];

  int           nbr_lookup[6][512];
  int           m_num_nbrs;

  uint64_t      m_note_n[2];
  int64_t       m_note_plane;

  int64_t       m_svd_nsv[6];

  Mersenne      m_rand;

  int32_t       m_return;

  std::vector< int32_t > m_block_admissible_tile;

  // unused right now...
  //
  //int64_t       m_breakout_block_fail_count,
  //              m_breakout_soften_limit;

  int64_t       m_block_fail_count,
                m_block_retry_limit,
                m_last_fail_count,
                m_soften_fail_count;

  int           m_soften_range;
  
  int64_t       m_error_cell;
  int64_t       m_error_cause;
  std::string   m_error_name;

  // parameters/options
  //
  bp_opt_t      op;

  // statistics
  //
  bp_stat_t     st;

  // experiments
  //
  bp_expr_t     expr;

};


#endif
