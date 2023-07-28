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

#define BELIEF_PROPAGATION_VERSION "0.6.0"

#define VB_NONE         0
#define VB_ERROR        0
#define VB_EXPERIMENT   1
#define VB_RUN          2
#define VB_STEP         3
#define VB_INTRASTEP    4
#define VB_DEBUG        5

#define OPT_PTRS
#define OPT_MUPTR
#define OPT_FH
#define OPT_MUBOUND

#define MU_NOCOPY 0
#define MU_COPY 1

#define VIZ_NONE        0
#define VIZ_MU          1
#define VIZ_DMU         2
#define VIZ_BELIEF      3
#define VIZ_CONSTRAINT  4
#define VIZ_TILECOUNT   5
#define VIZ_ENTROPY     6
#define VIZ_CHANGE      7
#define VIZ_RESPICK     8

#define ALG_CELL_WFC            31
#define ALG_CELL_ANY            32
#define ALG_CELL_MIN_ENTROPY    33

#define ALG_TILE_MAX_BELIEF     34

#define ALG_RUN_VANILLA         35
#define ALG_RUN_RESIDUAL        36
#define ALG_RUN_WFC             37
#define ALG_RUN_BACKTRACK       38

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

// auxiliary buffers for residual belief propagaion
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

#define BUF_BT                    22    // <2*B*num_vert, 1, 1>
#define BUF_BT_IDX                23    // <B*num_vert, 1, 1>

#define BUF_MAX         30      // this is buffer count limit. increase if more needed.


#define NOUT        -134217728

// Belief propagation - options

typedef struct _bp_opt_t {

  float     alpha;

  int       X, Y, Z, D;

  std::string name_fn;
  std::string rule_fn;

  std::string tileset_fn,
              tilemap_fn,
              tileobj_fn,
              outstl_fn;

  int32_t   tileset_stride_x,
            tileset_stride_y;
  int32_t   tileset_margin,
            tileset_spacing;
  int32_t   tileset_width,
            tileset_height;
  int       tiled_reverse_y;

  std::string  constraint_cmd;

  std::vector< int32_t > cull_list;

  int       seed;

  float     step_rate;

  float     eps_converge,
            eps_converge_beg,
            eps_converge_end,
            eps_zero;

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

  float   elapsed_time;

  int64_t constraints;

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

// Belief propagation - experiements

typedef struct _bp_expr_type {

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

    op.verbose = VB_NONE;

    default_opts();

  };

  //------------------------ high level API

  int       default_opts ();

  void      reset ();

  int       init( int, int, int,
              std::vector< std::string  >           tile_name_list,
              std::vector< float >                  tile_weight_list,
              std::vector< std::vector < float > >  rule_list );

  int       start();
  int       finish();

  int       RealizePre();
  int       RealizeIter();
  int       RealizeStep();
  int       RealizePost();
  int       Realize();

  int       CheckConstraints ( int64_t p );
  int       CheckConstraints ();

  void      SetVis (int viz_opt);

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
  int32_t tileName2ID (std::string &tile_name);
  int32_t tileName2ID (char *);

  // used for visualization
  void  ComputeDiffMUField ();
  void  ComputeBeliefField ();
  int   ComputeTilecountField ();


  // non "strict" bp functions but helpful still
  //
  int   CullBoundary();
  int   cellConstraintPropagate();
  void  cellFillVisited(uint64_t vtx, int32_t note_idx);
  int   cellFillSingle(uint64_t vtx, int32_t note_idx);

  int   tileIdxCollapse(uint64_t pos, int32_t tile_idx);
  int   tileIdxRemove(uint64_t pos, int32_t tile_idx);

  // note_idx is the 'plane' of BUF_NOTE to unwind
  //
  void  unfillVisited(int32_t note_idx);
  int   removeTileIdx(int64_t anch_cell, int32_t anch_tile_idx);
  int   sanityAccessed();

  //----------------------- visualization
  Vector4DF getVisSample ( int64_t v );



  //------------------------ memory management

  void          AllocBuf (int id, char dt, uint64_t cntx=1, uint64_t cnty=1, uint64_t cntz=1 );     // new function
  void          ZeroBuf (int id);

  int64_t       getNeighbor(uint64_t j, int nbr);        // 3D spatial neighbor function
  int64_t       getNeighbor(uint64_t j, Vector3DI jp, int nbr);        // 3D spatial neighbor function
  Vector3DI     getVertexPos(int64_t j);
  int64_t       getVertex(int x, int y, int z);
  int           getTilesAtVertex ( int64_t vtx );
  int           getOppositeDir(int nbr)  { return m_dir_inv[nbr]; }

  //----------------------- new accessor functions

  inline void*  getPtr(int id, int x=0, int y=0, int z=0)     {return (void*) m_buf[id].getPtr (x, y, z);}     // caller does type casting

  inline int32_t getValI(int id, int x=0, int y=0, int z=0)            {return *(int32_t*) m_buf[id].getPtr (x, y, z);}
  inline int64_t getValL(int id, int x=0, int y=0, int z=0)            {return *(int64_t*) m_buf[id].getPtr (x, y, z);}
  inline float   getValF(int id, int x=0, int y=0, int z=0)            {return *(float*) m_buf[id].getPtr (x, y, z);}

  inline void   SetValI(int id, int32_t val, int x, int y=0, int z=0)     {*(int32_t*) m_buf[id].getPtr(x, y, z) = val;}
  inline void   SetValL(int id, int64_t val, int x, int y=0, int z=0)     {*(int64_t*) m_buf[id].getPtr(x, y, z) = val;}
  inline void   SetValF(int id, float val, int x, int y=0, int z=0)     {*(float*)   m_buf[id].getPtr(x, y, z) = val;}

  inline int    getNumNeighbors(int j)        {return (m_bpres.z==1) ? 4 : 6;}
  // inline int    getNumNeighbors(int j)        {return 6;}

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

  //----------------------- LEGACY accessor functions

  //  belief prop residue access functions (int64_t)
  //  index heap - ih
  //
  /* inline int64_t* getPtr_ih(int32_t id, int32_t n, int64_t j, int32_t a)                { return  (int64_t*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
  inline int64_t  getVal_ih(int32_t id, int32_t n, int64_t j, int32_t a)                { return *(int64_t*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
  inline void     SetVal_ih(int32_t id, int32_t n, int64_t j, int32_t a, int64_t val )  { *(int64_t*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ) = val; }

  inline int64_t* getPtr_ih(int32_t id, int64_t idx)                { return  (int64_t*) m_buf[id].getPtr ( idx ); }
  inline int64_t  getVal_ih(int32_t id, int64_t idx)                { return *(int64_t*) m_buf[id].getPtr ( idx ); }
  inline void     SetVal_ih(int32_t id, int64_t idx, int64_t val )  { *(int64_t*) m_buf[id].getPtr ( idx ) = val; }

  inline float* getPtr_ihf(int32_t id, int64_t idx)             { return  (float*) m_buf[id].getPtr ( idx ); }
  inline float  getVal_ihf(int32_t id, int64_t idx)             { return *(float*) m_buf[id].getPtr ( idx ); }
  inline void   SetVal_ihf(int32_t id, int64_t idx, float val ) { *(float*) m_buf[id].getPtr ( idx ) = val; } */

/* inline int32_t getValNote(int id, int i, int a)                { return *(int32_t *) m_buf[id].getPtr ( uint64_t(i*m_num_verts+ a) ); }
   inline void    SetValNote(int id, int i, int a, int32_t val)   { *(int32_t*) m_buf[id].getPtr ( (i*m_num_verts + a) ) = val; }  */


  //-------------------------- wave function collapse
  int   wfc();
  int   wfc_start();
  int   wfc_step(int64_t it);

  //-------------------------- backtracking
  int cellConstraintPropagate_lookahead(int64_t, int32_t);
  int btPush(int64_t, int64_t, int64_t);
  int btUnwind(int64_t);



  //-------------------------- debugging functions

  // helper arrays and functions for ease of testing and simple use
  //
  void  debugPrint();
  void  debugPrintC();
  void  debugPrintS();
  void  debugPrintMU();
  void  debugInspect (Vector3DI pos, int tile);

  // run time statistics and other information
  //
  // void    UpdateRunTimeStat(int64_t num_step);


  //------------------------- member variables

  // primary data stored in buffers
  DataPtr       m_buf[ BUF_MAX ];

  // problem size
  int64_t       m_num_verts;    // Xi = 0..X (graph domain)
  int64_t       m_num_values;   //  B = 0..Bm-1 (value domain)
  Vector3DI     m_bpres;        // 3D spatial belief prop res

  Vector3DI     m_res;          // volume res

  std::vector< std::string > m_tile_name;
  std::vector< std::string > m_dir_desc;
  int           m_dir_inv[6];

  int           nbr_lookup[6][512];

  uint64_t      m_note_n[2];
  int64_t       m_note_plane;

  int64_t       m_svd_nsv[6];

  Mersenne      m_rand;

  // parameters/options
  bp_opt_t      op;

  // statistics
  bp_stat_t     st;

  // experiments
  bp_expr_t     expr;

};


#endif