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

#define BELIEF_PROPAGATION_VERSION "0.5.0"

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

#define ALG_CELL_ANY            32
#define ALG_CELL_MIN_ENTROPY    33

#define ALG_TILE_MAX_BELIEF     34

#define ALG_RUN_VANILLA         35
#define ALG_RUN_RESIDUAL        36


#define BUF_VOL         0     // volume: n^3
#define BUF_G           1     // beliefprop, G(a) vector
#define BUF_H           2     // beliefprop, H(a) vector
#define BUF_F           3     // beliefprop, F(a,b) vector - assume independent of i,j
#define BUF_MU          4     // beliefprop, mu{i,j}(a,b) vector
#define BUF_MU_NXT      5     // beliefprop, mu'{i,j}(a,b) vector
#define BUF_BELIEF      6     // Belief array
#define BUF_TILE_IDX    7     // volume 'dynamic' array of current tile indexes per cell
#define BUF_TILE_IDX_N  8     // volume number of tiles at cell position
#define BUF_CONSIDER    9     // volume consider/scan (of size cell count (i32))
#define BUF_VISITED     10    // volume visited/picked (of size cell count (i32))
#define BUF_NOTE        11    // volume to keep track of which cells have been processed to  (of size cell count (i32))
#define BUF_VIZ         12    // volume for vizualization

#define BUF_MU_RESIDUE  13

#define BUF_SVD_U       14
#define BUF_SVD_Vt      15
#define BUF_SVD_VEC     16

// auxiliary buffers for residual belief propagaion
//
// BUF_RESIDUE_HEAP         : heap of absolute differences of mu and mu_nxt (float)
// BUF_RESIDUE_HEAP_CELL_BP : back pointer of heap value location in CELL_HEAP (in64_t)
// BUF_RESIDUE_CELL_HEAP    : mapping of cell (and direction, value) to heap position (in64_t).
//                            That is, mapping of mu index to position in heap
//
// All sizes should be (Vol)*(2*D)*(B).
// That is, {volume} x {#neighbors} x {#values} : (dim[0]*dim[1]*dim[2]*6*B).
//
// All these structures are for book keeping so that we can get the maximum difference of
// mu and mu_nxt in addition to allowing arbitrary updates on other cells.
//
// The basic residual bp update step will fetch the maximum difference, copy the mu value
// from the mu_nxt buffer to the mu buffer, then update neighboring cells by updating their
// mu_nxt values, updating the residue_heap along the way.
//
#define BUF_RESIDUE_HEAP          17
#define BUF_RESIDUE_HEAP_CELL_BP  18
#define BUF_RESIDUE_CELL_HEAP     19

class BeliefPropagation {
public:
  BeliefPropagation() {
    m_seed = 17;
    m_verbose = 0;
    m_eps_converge = (1.0/(1024.0));
    //m_eps_converge = (1.0/(1024.0*1024.0));
    //m_eps_zero = (1.0/(1024.0*1024.0));
    m_eps_zero = (1.0/(1024.0*1024.0*1024.0*1024.0));
    m_max_iteration = 1024;

    //m_step_cb = 10;
    m_step_cb = 1;
    m_state_info_d = -1;
    m_state_info_iter = 0;

    m_rate = 0.98;

    m_use_svd = 0;
    m_use_checkerboard = 0;

    m_index_heap_size = 0;

    m_stat_enabled = 1;
    m_stat_avg_iter = 0.0;

    // unused...
    m_stat_second_moment_iter = 0.0;

    m_stat_cur_iter = 0;
    m_stat_max_iter = 0;
    m_stat_num_culled = 0;
    m_stat_num_collapsed = 0;

    // unused...
    m_stat_num_chosen = 0;

    m_eps_converge_beg = m_eps_converge;
    m_eps_converge_end = m_eps_converge;

    m_viz_opt = VIZ_NONE;

    m_alg_cell_opt = ALG_CELL_MIN_ENTROPY;
    m_alg_tile_opt = ALG_TILE_MAX_BELIEF;
    m_alg_run_opt = ALG_RUN_VANILLA;

  };

  bool _init();

  int init_CSV(int, std::string &, std::string &);
  int init_CSV(int, int, int, std::string &, std::string &);
  int init_F_CSV(std::string &, std::string &);

  int init_SVD(void);

  //DEBUG
  //DEBUG
  int  init(int);
  int  init(int, int, int);
  //DEBUG
  //DEBUG

  void  init_dir_desc();

  //----------------------- visualization
  Vector4DF getSample ( int buf, int64_t v );


  //---

  //------------------------ memory management  
  void     Restart();
  void     ZeroBPVec (int id);
  void     AllocBPVec (int id, int cnt);                  // vector alloc
  void     AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals);  // matrix alloc
  void     AllocBPMtx_i64 (int32_t id, int32_t nbrs, uint64_t verts, uint32_t vals);
  void     AllocBPMap (int id, int nbrs, int vals);
  void     AllocViz (int id, uint64_t cnt );
  void     AllocTileIdx (int, int, int);
  void     AllocTileIdxN(int, int );
  void     AllocVeci32(int, int);
  void     AllocVeci32(int, int, int);
  void     AllocSVD(int, int, int, int);
  int64_t  getNeighbor(uint64_t j, int nbr);        // 3D spatial neighbor function
  int64_t  getNeighbor(uint64_t j, Vector3DI jp, int nbr);        // 3D spatial neighbor function
  Vector3DI  getVertexPos(int64_t j);
  int64_t  getVertex(int x, int y, int z);
  int      getTilesAtVertex ( int64_t vtx );
  int      getOppositeDir(int nbr)  { return m_dir_inv[nbr]; }

  //----------------------- accessor functions
  inline int      getNumNeighbors(int j)        {return 6;}
  inline int      getNumValues(int j)          {return m_num_values;}
  inline int      getNumVerts()            {return m_num_verts;}
  // G and H vectors, size B
  inline float*  getPtr(int id, int a)                  {return  (float*) m_buf[id].getPtr (a);}            
  inline float   getVal(int id, int a)                  {return *(float*) m_buf[id].getPtr (a);}  
  inline void    SetVal(int id, int a, float val)       {*(float*) m_buf[id].getPtr(a) = val;}


#ifdef OPT_PTRS

  // Optimized: Closest values in memory are most used in inner loops
  // MU matrix
  // n=nbr (0-6), j=vertex (D), a=tile (B)
  inline float*  getPtr(int id, int nbr, int j, int a)              {return  (float*) m_buf[id].getPtr ( uint64_t(a*m_num_verts + j)*6 + nbr ); }  
  inline float   getVal(int id, int nbr, int j, int a)              {return *(float*) m_buf[id].getPtr ( uint64_t(a*m_num_verts + j)*6 + nbr ); }
  inline void    SetVal(int id, int nbr, int j, int a, float val )  {*(float*) m_buf[id].getPtr ( uint64_t(a*m_num_verts + j)*6 + nbr ) = val; }

  // belief mapping (F), BxB
  inline float*  getPtrF(int id, int a, int b, int n)      { return (float*) m_buf[id].getPtr ( (b*6 + n)*m_num_values + a ); }  
  inline float   getValF(int id, int a, int b, int n)      { return *(float*) m_buf[id].getPtr ( (b*6 + n)*m_num_values + a ); } 
  inline void    SetValF(int id, int a, int b, int n, float val ) { *(float*) m_buf[id].getPtr ( (b*6 + n)*m_num_values + a ) = val; }

#else
  // MU matrix
  inline float*  getPtr(int id, int n, int j, int a)                { return  (float*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
  inline float   getVal(int id, int n, int j, int a)                { return *(float*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
  inline void    SetVal(int id, int n, int j, int a, float val )    { *(float*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ) = val; }
  
  // Belief mapping (F), BxB
  inline float*  getPtrF(int id, int a, int b, int n)               { return  (float*) m_buf[id].getPtr ( (b*m_num_values + a)*6 + n ); }
  inline float   getValF(int id, int a, int b, int n)               { return *(float*) m_buf[id].getPtr ( (b*m_num_values + a)*6 + n ); }
  inline void    SetValF(int id, int a, int b, int n, float val )   { *(float*) m_buf[id].getPtr ( (b*m_num_values + a)*6 + n) = val; }
#endif

  inline int32_t getVali(int id, int i)                { return *(int32_t *) m_buf[id].getPtr (i); }
  inline void    SetVali(int id, int i, int32_t val)   { *(int32_t *) m_buf[id].getPtr (i) = val;  }
  inline int32_t getVali(int id, int i, int a)                { return *(int32_t *) m_buf[id].getPtr ( uint64_t(i*m_num_values + a) ); }
  inline void    SetVali(int id, int i, int a, int32_t val)   { *(int32_t*) m_buf[id].getPtr ( (i*m_num_values + a) ) = val; }


  inline int32_t getValNote(int id, int i, int a)                { return *(int32_t *) m_buf[id].getPtr ( uint64_t(i*m_num_verts+ a) ); }
  inline void    SetValNote(int id, int i, int a, int32_t val)   { *(int32_t*) m_buf[id].getPtr ( (i*m_num_verts + a) ) = val; }

  //  belief prop residue access functions (int64_t)
  //  index heap - ih
  //
  inline int64_t* getPtr_ih(int32_t id, int32_t n, int64_t j, int32_t a)                { return  (int64_t*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
  inline int64_t  getVal_ih(int32_t id, int32_t n, int64_t j, int32_t a)                { return *(int64_t*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
  inline void     SetVal_ih(int32_t id, int32_t n, int64_t j, int32_t a, int64_t val )  { *(int64_t*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ) = val; }

  inline int64_t* getPtr_ih(int32_t id, int64_t idx)                { return  (int64_t*) m_buf[id].getPtr ( idx ); }
  inline int64_t  getVal_ih(int32_t id, int64_t idx)                { return *(int64_t*) m_buf[id].getPtr ( idx ); }
  inline void     SetVal_ih(int32_t id, int64_t idx, int64_t val )  { *(int64_t*) m_buf[id].getPtr ( idx ) = val; }

  inline float* getPtr_ihf(int32_t id, int64_t idx)             { return  (float*) m_buf[id].getPtr ( idx ); }
  inline float  getVal_ihf(int32_t id, int64_t idx)             { return *(float*) m_buf[id].getPtr ( idx ); }
  inline void   SetVal_ihf(int32_t id, int64_t idx, float val ) { *(float*) m_buf[id].getPtr ( idx ) = val; }

  // residual belief propagation helper functions
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


  //---

  int   start();
 
  // legacy
  int   single_realize (int64_t it);
  int   single_realize_cb (int64_t it, void (*cb)(void *));
  //int   single_realize_lest_belief_cb (int64_t it, void (*cb)(void *));

  // core methods
  int   RAMA_single_realize_max_belief_cb(int64_t it, void (*cb)(void *));

  int   single_realize_max_belief_cb(int64_t it, void (*cb)(void *));
  int   single_realize_min_entropy_max_belief_cb(int64_t it, void (*cb)(void *));

  // min belief algorithm
  int   single_realize_min_belief_cb(int64_t it, void (*cb)(void *));
  
  // experimental
  int   single_realize_min_entropy_min_belief_cb(int64_t it, void (*cb)(void *));
  int   single_realize_residue_cb(int64_t it, void (*cb)(void *));


  int   _pick_tile(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   _pick_tile_max_belief(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   _pick_tile_min_belief(int64_t anch_cell, int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);
  int   _pick_tile_pdf(int64_t anch_cell, int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);

  //----

  int32_t     m_run_opt;

  int32_t     m_viz_opt;
  int32_t     m_alg_cell_opt;
  int32_t     m_alg_tile_opt;
  int32_t     m_alg_run_opt;

  int64_t     m_run_iter;
  int         RealizePre();
  int         RealizeRun();
  int         RealizeStep();
  int         RealizePost();
  int         Realize();

  //----

  int    realize();
  int    wfc();
  int    wfc_start();
  int    wfc_step(int64_t it);

  float  step(int update_mu);
  //float  step_residue(float *max_diff, int64_t *max_residue_cell, int64_t *max_residue_tile_idx, int64_t *max_dir_idx);
  float  step_residue(int32_t idir, int64_t cell, int32_t tile);

  float   BeliefProp();
  float   BeliefProp_svd ();

  float   BeliefProp_cell_residue(int64_t);
  float   BeliefProp_cell_residue_svd(int64_t);

  void    UpdateMU ();

  float    getVertexBelief ( uint64_t j );
  float    _getVertexBelief ( uint64_t j );

  void    cellUpdateBelief(int64_t anch_cell);
  int     chooseMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int     chooseMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);

  int     chooseMaxEntropy(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int     chooseMinEntropyMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int     chooseMinEntropyMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);

  void    WriteBoundaryMU ();
  void    WriteBoundaryMUbuf(int buf_id);
  void    TransferBoundaryMU (int src_id, int dst_id);

  float   MaxDiffMU();
  float   MaxDiffMUCellTile(float *max_diff, int64_t *max_cell, int64_t *max_tile_idx, int64_t *max_dir_idx);
  void    ComputeDiffMUField ();

  void    ConstructF ();
  void    ConstructGH ();
  void    ConstructMU ();
  void    NormalizeMU ();
  void    NormalizeMU (int id);

  void    NormalizeMU_cell_residue (int id, int64_t cell);

  void    ConstructTileIdx();

  uint64_t  m_num_verts;    // Xi = 0..X (graph domain)
  uint64_t  m_num_values;    //  B = 0..Bm-1 (value domain)
  Vector3DI m_bpres;      // 3D spatial belief prop res

  Vector3DI m_res;        // volume res

  DataPtr  m_buf[128];      // data buffers (CPU & GPU)

  // SVD number of non singular values in each direction
  //
  int       m_use_svd;
  int64_t   m_svd_nsv[6];

  int       m_use_checkerboard;

  bool      m_run_cuda=0;
  int       m_seed;
  Mersenne  m_rand;

  // helper arrays and functions for ease of testing and simple use
  //
  void debugPrint();
  void debugPrintC();
  void debugPrintS();
  void debugPrintMU();

  std::vector< std::string > m_tile_name;
  std::vector< std::string > m_dir_desc;
  int m_dir_inv[6];

  void filterKeep(uint64_t pos, std::vector<int32_t> &tile_id);
  void filterDiscard(uint64_t pos, std::vector<int32_t> &tile_id);
  int32_t tileName2ID (std::string &tile_name);
  int32_t tileName2ID (char *);

  // non "strict" bp functions but helpful still
  //
  int CullBoundary();
  int _CullBoundary();
  void ConstructConstraintBuffers();
  int cellConstraintPropagate();
  void cellFillAccessed(uint64_t vtx, int32_t note_idx);
  int cellFillSingle(uint64_t vtx, int32_t note_idx);

  int tileIdxCollapse(uint64_t pos, int32_t tile_idx);
  int tileIdxRemove(uint64_t pos, int32_t tile_idx);

  // note_idx is the 'plane' of BUF_NOTE to unwind
  //
  void unfillAccessed(int32_t note_idx);
  int removeTileIdx(int64_t anch_cell, int32_t anch_tile_idx);
  int sanityAccessed();


  uint64_t m_note_n[2];

  int64_t m_grid_note_idx;

  float m_rate;

  int filter_constraint(std::vector< std::vector< int32_t > > &constraint_list);

  int m_verbose;

  float m_eps_converge;

  float m_eps_converge_beg,
        m_eps_converge_end;

  float m_eps_zero;

  

  void gp_state_print();

  int64_t   m_step_cb;
  float     m_state_info_d;
  int64_t   m_state_info_iter;

  int64_t   m_step_iter;
  int64_t   m_max_iteration;

  int64_t   m_index_heap_size;

  // run time statistics and other information
  //

  void    UpdateRunTimeStat(int64_t num_step);
  int32_t m_stat_enabled;
  double  m_stat_avg_iter,
          m_stat_second_moment_iter;
  int64_t m_stat_cur_iter,
          m_stat_max_iter,
          m_stat_num_culled,
          m_stat_num_collapsed,
          m_stat_num_chosen;

};

int _read_line(FILE *fp, std::string &line);
int _read_name_csv(std::string &fn, std::vector<std::string> &name, std::vector<float> &weight);
int _read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule);
int _read_constraint_csv(std::string &fn, std::vector< std::vector<int32_t> > &admissible_tile);


#endif
