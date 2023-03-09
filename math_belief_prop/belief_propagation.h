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

#define BELIEF_PROPAGATION_VERSION "0.3.1"

#define RUN_OPT_PTRS
#define RUN_OPT_MUPTR
#define RUN_OPT_FH
#define RUN_OPT_MUBOUND


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
  };

  //----------------------- belief propagation (mid-level)
  bool  _init();

  int   init_CSV (int, std::string &, std::string &);
  int   init_CSV (int, int, int, std::string &, std::string &);
  int   init_F_CSV(std::string &, std::string &);

  int   init_SVD(void);

  //DEBUG
  //DEBUG
  int   init(int);
  int   init(int, int, int);
  //DEBUG
  //DEBUG
    
  int   start();

  int   realize();

  int   filter_constraint(std::vector< std::vector< int32_t > > &constraint_list);

  void  gp_state_print();
 
  // legacy
  int   single_realize (int64_t it);
  int   single_realize_cb (int64_t it, void (*cb)(void *));
  //int single_realize_lest_belief_cb (int64_t it, void (*cb)(void *));

  // core methods
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

  void  init_dir_desc();
  float step(int update_mu);
  float step_residue(float *max_diff, int64_t *max_residue_cell, int64_t *max_residue_tile_idx, int64_t *max_dir_idx);

  
  //----------------------- belief propagation (low-level)
  float BeliefProp();
  float BeliefProp_cell(int64_t);
  float BeliefProp_svd ();

  void  UpdateMU ();

  float getVertexBelief ( uint64_t j );
  float _getVertexBelief ( uint64_t j );

  void  cellUpdateBelief(int64_t anch_cell);
  int   chooseMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   chooseMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);

  int   chooseMaxEntropy(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   chooseMinEntropyMaxBelief(int64_t *max_cell, int32_t *max_tile, int32_t *max_tile_idx, float *max_belief);
  int   chooseMinEntropyMinBelief(int64_t *min_cell, int32_t *min_tile, int32_t *min_tile_idx, float *min_belief);

  void  WriteBoundaryMU ();
  void  WriteBoundaryMUbuf(int buf_id);
  void  TransferBoundaryMU (int src_id, int dst_id);
  float MaxDiffMU();
  float MaxDiffMUCellTile(float *max_diff, int64_t *max_cell, int64_t *max_tile_idx, int64_t *max_dir_idx);
  void  ComputeDiffMUField ();
  void  NormalizeMU ();
  void  NormalizeMU (int id);

  void  filterKeep(uint64_t pos, std::vector<int32_t> &tile_id);
  void  filterDiscard(uint64_t pos, std::vector<int32_t> &tile_id);
  int32_t tileName2ID (std::string &tile_name);
  int32_t tileName2ID (char *);

  // non "strict" bp functions but helpful still
  //
  int   CullBoundary();
  int   _CullBoundary();
  void  ConstructConstraintBuffers();
  int   cellConstraintPropagate();
  void  cellFillAccessed(uint64_t vtx, int32_t note_idx);
  int   cellFillSingle(uint64_t vtx, int32_t note_idx);

  int   tileIdxCollapse(uint64_t pos, int32_t tile_idx);
  int   tileIdxRemove(uint64_t pos, int32_t tile_idx);

  // note_idx is the 'plane' of BUF_NOTE to unwind
  //
  void  unfillAccessed(int32_t note_idx);
  int   removeTileIdx(int64_t anch_cell, int32_t anch_tile_idx);
  int   sanityAccessed();



  //----------------------- visualization
  Vector4DF getVisSample ( int buf, int64_t v );



  //------------------------ memory management  
  void     Restart();
  void     ConstructF ();
  void     ConstructGH ();
  void     ConstructMU ();
  void     ConstructTileIdx();
  void     ZeroBPVec (int id);
  void     AllocBPVec (int id, int cnt);                  // vector alloc
  void     AllocBPMtx (int id, int nbrs, uint64_t verts, uint64_t vals);  // matrix alloc
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
  inline int    getNumNeighbors(int j)        {return 6;}
  inline int    getNumValues(int j)          {return m_num_values;}
  inline int    getNumVerts()            {return m_num_verts;}
  // G and H vectors, size B
  inline float* getPtr(int id, int a)                  {return  (float*) m_buf[id].getPtr (a);}            
  inline float  getVal(int id, int a)                  {return *(float*) m_buf[id].getPtr (a);}  
  inline void   SetVal(int id, int a, float val)       {*(float*) m_buf[id].getPtr(a) = val;}

#ifdef RUN_OPT_PTRS
  // Optimized: Closest values in memory are most used in inner loops
  // MU matrix
  // n=nbr (0-6), j=vertex (D), a=tile (B)
  inline float* getPtr(int id, int nbr, int j, int a)              {return  (float*) m_buf[id].getPtr ( uint64_t(a*m_num_verts + j)*6 + nbr ); }  
  inline float  getVal(int id, int nbr, int j, int a)              {return *(float*) m_buf[id].getPtr ( uint64_t(a*m_num_verts + j)*6 + nbr ); }
  inline void   SetVal(int id, int nbr, int j, int a, float val )  {*(float*) m_buf[id].getPtr ( uint64_t(a*m_num_verts + j)*6 + nbr ) = val; }

  // belief mapping (F), BxB
  inline float* getPtrF(int id, int a, int b, int n)      { return (float*) m_buf[id].getPtr ( (b*6 + n)*m_num_values + a ); }  
  inline float  getValF(int id, int a, int b, int n)      { return *(float*) m_buf[id].getPtr ( (b*6 + n)*m_num_values + a ); } 
  inline void   SetValF(int id, int a, int b, int n, float val ) { *(float*) m_buf[id].getPtr ( (b*6 + n)*m_num_values + a ) = val; }

#else
  // MU matrix
  inline float*  getPtr(int id, int n, int j, int a)                {return  (float*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
  inline float   getVal(int id, int n, int j, int a)                {return *(float*) m_buf[id].getPtr ( uint64_t(n*m_num_verts + j)*m_num_values + a ); }
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


  //-------------------------- wave function collapse
  int   wfc();
  int   wfc_start();
  int   wfc_step(int64_t it);

  
  
  //-------------------------- debugging functions

  // helper arrays and functions for ease of testing and simple use
  //
  void debugPrint();
  void debugPrintC();
  void debugPrintS();



  //------------------------- member variables

  // bp data stored in dataptr buffers
  DataPtr   m_buf[128];     

  int64_t   m_num_verts;    // Xi = 0..X (graph domain)
  int64_t   m_num_values;   //  B = 0..Bm-1 (value domain)
  Vector3DI m_bpres;        // 3D spatial belief prop res

  Vector3DI m_res;          // volume res

  std::vector< std::string > m_tile_name;
  std::vector< std::string > m_dir_desc;
  int m_dir_inv[6];


  // SVD number of non singular values in each direction
  //
  int       m_use_svd;
  int64_t   m_svd_nsv[6];
  int       m_use_checkerboard;

  bool      m_run_cuda=0;
  int       m_seed;
  Mersenne  m_rand;

  uint64_t m_note_n[2];

  int64_t m_grid_note_idx;

  float m_rate;

  

  int m_verbose;

  float m_eps_converge;
  float m_eps_zero;

  
  int64_t   m_step_cb;
  float     m_state_info_d;
  int64_t   m_state_info_iter;

  int64_t   m_step_iter;
  int64_t   m_max_iteration;

};

int _read_line(FILE *fp, std::string &line);
int _read_name_csv(std::string &fn, std::vector<std::string> &name, std::vector<float> &weight);
int _read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule);
int _read_constraint_csv(std::string &fn, std::vector< std::vector<int32_t> > &admissible_tile);


#endif
