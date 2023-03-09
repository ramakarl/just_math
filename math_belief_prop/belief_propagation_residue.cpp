//--------------------------------------------------------------------------------
// JUST MATH:
// Belief Propagation - helper functions for residual belief propagation

//--------------------------------------------------------------------------------
// Copyright 2023 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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
//
// Sample utils
//
#include <algorithm>
#include "mersenne.h"
#include "dataptr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "belief_propagation.h"

int64_t BeliefPropagation::getMuIdx( int32_t idir, int64_t cell, int32_t tile ) {
  return (int64_t)((idir*m_num_verts + cell)*m_num_values + tile);
}

// (idir * m_num_verts * m_num_values) + (cell * m_num_values) + tile
//
int64_t BeliefPropagation::getMuPos( int64_t mu_idx, int32_t *idir, int64_t *cell, int32_t *tile ) {
  int64_t _idir,
          _cell,
          _tile;
  int64_t q = 0;

  _idir = mu_idx / (m_num_verts * m_num_values);
  q += _idir * m_num_verts * m_num_values;
  _cell = (mu_idx - q) / m_num_values;
  q += _cell * m_num_values;
  _tile = (mu_idx - q);

  *idir = (int32_t)_idir;
  *cell = (int64_t)_cell;
  *tile = (int32_t)_tile;

  return 0;
}

void BeliefPropagation::indexHeap_swap(int64_t heap_idx_a, int64_t heap_idx_b) {
  int64_t cell_idx_a,
          cell_idx_b;
  float val_a,
        val_b;

  //val_a       = getVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_a );
  //cell_idx_a  = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_a );

  val_a       = getValF( BUF_RESIDUE_HEAP,         heap_idx_a );
  cell_idx_a  = getValL( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_a );

  //val_b       = getVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_b );
  //cell_idx_b  = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_b );

  val_b       = getValF( BUF_RESIDUE_HEAP,         heap_idx_b );
  cell_idx_b  = getValL( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_b );

  //SetVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_a, val_b );
  //SetVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_a, cell_idx_b );

  SetValF( BUF_RESIDUE_HEAP,         val_b,       heap_idx_a );
  SetValL( BUF_RESIDUE_HEAP_CELL_BP, cell_idx_b,  heap_idx_a );

  //SetVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_b, val_a );
  //SetVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_b, cell_idx_a );

  SetValF( BUF_RESIDUE_HEAP,         val_a,       heap_idx_b );
  SetValL( BUF_RESIDUE_HEAP_CELL_BP, cell_idx_a,  heap_idx_b );

  //SetVal_ih ( BUF_RESIDUE_CELL_HEAP,  cell_idx_a, heap_idx_b );
  //SetVal_ih ( BUF_RESIDUE_CELL_HEAP,  cell_idx_b, heap_idx_a );

  SetValL( BUF_RESIDUE_CELL_HEAP,  heap_idx_b, cell_idx_a );
  SetValL( BUF_RESIDUE_CELL_HEAP,  heap_idx_a, cell_idx_b );

}

static inline int64_t _lchild(int64_t _ii) { return ((2*(_ii))+1); }
static inline int64_t _rchild(int64_t _ii) { return ((2*(_ii))+2); }
static inline int64_t _parent(int64_t _ii) { return (_ii-1)/2; }

// initialize index heap with difference of MU and MU_NXT buffers
//
void BeliefPropagation::indexHeap_init() {
  int64_t mu_idx, cell, mu_n,
          n_tile_idx, tile_idx, tile_id;
  int32_t idir, tile;
  float mu_cur, mu_nxt, mu_del;

  m_index_heap_size = 0;

  mu_n = 6*m_num_verts*m_num_values;

  for (mu_idx=0; mu_idx<mu_n; mu_idx++) { indexHeap_push( 0.0 ); }

  for (idir=0; idir<6; idir++) {
    for (cell=0; cell<m_num_verts; cell++) {
      //n_tile_idx = getVali( BUF_TILE_IDX_N, cell );
      n_tile_idx = getValI( BUF_TILE_IDX_N, cell );
      for (tile_idx=0; tile_idx<n_tile_idx; tile_idx++) {
        //tile = getVali( BUF_TILE_IDX, cell, tile_idx );
        tile = getValI( BUF_TILE_IDX, cell, tile_idx );

        //mu_cur = getVal( BUF_MU,      idir, cell, tile );
        //mu_nxt = getVal( BUF_MU_NXT,  idir, cell, tile );

        mu_cur = getValF( BUF_MU,      idir, tile, cell );
        mu_nxt = getValF( BUF_MU_NXT,  idir, tile, cell );

        mu_del = fabs(mu_cur - mu_nxt);

        indexHeap_update_mu_pos(idir, cell, tile, mu_del);
      }
    }
  }

}

int32_t BeliefPropagation::indexHeap_push(float val) {
  int64_t mu_n, idx_par, idx, t_idx;
  float f, val_par, val_cur;

  mu_n = 6*m_num_verts*m_num_values;

  if (m_index_heap_size >= mu_n) { return -1; }

  //SetVal_ihf( BUF_RESIDUE_HEAP,         m_index_heap_size, val );
  //SetVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, m_index_heap_size, m_index_heap_size );
  //SetVal_ih ( BUF_RESIDUE_CELL_HEAP,    m_index_heap_size, m_index_heap_size );

  SetValF( BUF_RESIDUE_HEAP,         val, m_index_heap_size );
  SetValL( BUF_RESIDUE_HEAP_CELL_BP, m_index_heap_size, m_index_heap_size );
  SetValL( BUF_RESIDUE_CELL_HEAP,    m_index_heap_size, m_index_heap_size );

  idx = m_index_heap_size;
  while (idx > 0) {
    idx_par = (idx-1)/2;

    //val_par = getVal_ihf( BUF_RESIDUE_HEAP, idx_par );
    //val_cur = getVal_ihf( BUF_RESIDUE_HEAP, idx );

    val_par = getValF( BUF_RESIDUE_HEAP, idx_par );
    val_cur = getValF( BUF_RESIDUE_HEAP, idx );

    if ( val_par < val_cur ) {
      indexHeap_swap( idx_par, idx );
    }
    idx = idx_par;
  }

  m_index_heap_size++;

  return 0;
}

void BeliefPropagation::indexHeap_update_mu_idx(int64_t mu_idx, float val) {
  int64_t heap_idx;
  //heap_idx = getVal_ih( BUF_RESIDUE_CELL_HEAP, mu_idx );
  heap_idx = getValL( BUF_RESIDUE_CELL_HEAP, mu_idx );
  indexHeap_update( heap_idx, val );
}

void BeliefPropagation::indexHeap_update_mu_pos(int32_t idir, int64_t cell, int32_t tile, float val) {
  int64_t heap_idx, mu_idx;
  mu_idx = getMuIdx(idir, cell, tile);
  //heap_idx = getVal_ih( BUF_RESIDUE_CELL_HEAP, mu_idx );
  heap_idx = getValL( BUF_RESIDUE_CELL_HEAP, mu_idx );
  indexHeap_update( heap_idx, val );
}

// Alter value at heap_idx location.
// First, bubble up change, then start pushing down.
//
void BeliefPropagation::indexHeap_update(int64_t heap_idx, float val) {

  int64_t max_heap_idx,
          heap_idx_par,
          heap_idx_lc,
          heap_idx_rc;
  float par_val,
        child_val,
        max_heap_val;

  int64_t heap_n;

  heap_n = m_index_heap_size;
  //heap_n = 6*m_num_verts*m_num_values;

  //SetVal_ihf( BUF_RESIDUE_HEAP, heap_idx, val );
  SetValF( BUF_RESIDUE_HEAP, val, heap_idx );

  // bubble up value until we can't anymore
  //
  while (heap_idx > 0) {
    heap_idx_par = _parent(heap_idx);

    //par_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_par );
    par_val = getValF( BUF_RESIDUE_HEAP, heap_idx_par );

    if ( val <= par_val ) { break; }
    indexHeap_swap( heap_idx_par, heap_idx );
    heap_idx = heap_idx_par;
  }

  // push down value
  //
  heap_idx_par = heap_idx;
  max_heap_idx = heap_idx_par;

  while ( heap_idx_par < heap_n ) {

    // Take the maximum of the three of current entry (parent),
    // left child and right child.
    // The maximum entry will be guaranteed to be larger than
    // or equal to the other two, letting us put it in the
    // current (parent) position.
    // Whichever entry we swapped it with, 'recur' on it
    // to keep pushing it down.
    // Since all we're doing is swapping entries, worst case
    // we get down to a leaf node and the last entry is smaller
    // than its parent.
    // If we don't swap the current entry (parent) with any
    // of the children, we've reached a consistent heap.
    //

    heap_idx_lc = _lchild(heap_idx_par);
    heap_idx_rc = _rchild(heap_idx_par);

    max_heap_idx = heap_idx_par;
    //max_heap_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_par );
    max_heap_val = getValF( BUF_RESIDUE_HEAP, heap_idx_par );

    if (heap_idx_lc < heap_n) {
      //child_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_lc );
      child_val = getValF( BUF_RESIDUE_HEAP, heap_idx_lc );
      if (child_val > max_heap_val) {
        max_heap_idx = heap_idx_lc;
        max_heap_val = child_val;
      }
    }

    if (heap_idx_rc < heap_n) {
      //child_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_rc );
      child_val = getValF( BUF_RESIDUE_HEAP, heap_idx_rc );
      if (child_val > max_heap_val) {
        max_heap_idx = heap_idx_rc;
        max_heap_val = child_val;
      }
    }

    if (heap_idx_par == max_heap_idx) { break; }

    indexHeap_swap( heap_idx_par, max_heap_idx );
    heap_idx_par = max_heap_idx;

  }

}

int64_t BeliefPropagation::indexHeap_peek(int64_t *mu_idx, float *val) {
  int64_t _mu_idx, _heap_idx;
  float _f;

  //_f         = getVal_ihf( BUF_RESIDUE_HEAP,         0 );
  //_mu_idx    = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, 0 );

  _f         = getValF( BUF_RESIDUE_HEAP,         0 );
  _mu_idx    = getValL( BUF_RESIDUE_HEAP_CELL_BP, 0 );

  if (mu_idx) { *mu_idx = _mu_idx; }
  if (val)    { *val = _f; }

  return _mu_idx;
}

int64_t BeliefPropagation::indexHeap_peek_mu_pos(int32_t *idir, int64_t *cell, int32_t *tile_val, float *val) {
  int64_t _mu_idx, _heap_idx;
  float _f;

  //_f         = getVal_ihf( BUF_RESIDUE_HEAP,         0 );
  //_mu_idx    = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, 0 );

  _f         = getValF( BUF_RESIDUE_HEAP,         0 );
  _mu_idx    = getValL( BUF_RESIDUE_HEAP_CELL_BP, 0 );

  getMuPos(_mu_idx, idir, cell, tile_val);

  if (val)    { *val = _f; }

  return _mu_idx;
}




//----
//----
//


void BeliefPropagation::indexHeap_debug_print(void) {
  int64_t i, p2, p2_c;

  printf("indexHeap m_index_heap_size: %i (/%i)\n",
      (int)m_index_heap_size, (int)(6*m_num_values*m_num_verts));

  printf("indexHeap cell_heap:\n");

  p2 = 16;
  p2_c = p2;
  for (i=0; i<m_index_heap_size; i++) {
    if (p2_c==0) {
      printf("\n   ");
      p2_c = p2;
    }
    //printf(" %i", (int)getVal_ih( BUF_RESIDUE_CELL_HEAP, i ) );
    printf(" %i", (int)getValL( BUF_RESIDUE_CELL_HEAP, i ) );
    p2_c--;
  }
  printf("\n");

  printf("indexHeap heap[heap_cell_bp]:\n");
  p2 = 1;
  p2_c = 1;
  for (i=0; i<m_index_heap_size; i++) {
    if (p2_c==0) {
      printf("\n   ");
      p2 *= 2;
      p2_c = p2;
    }
    printf(" %f[%i]",
        (float)getValF( BUF_RESIDUE_HEAP, i ),
        (int)getValL( BUF_RESIDUE_HEAP_CELL_BP, i ) );
        //(float)getVal_ihf( BUF_RESIDUE_HEAP, i ),
        //(int)getVal_ih( BUF_RESIDUE_HEAP_CELL_BP, i ) );
    p2_c--;
  }
  printf("\n");

}

//----

int32_t BeliefPropagation::indexHeap_mu_consistency(void) {
  int64_t i, idx_bp, idx, idx_child;
  int32_t err_code=0;
  float f_d, val_a, val_b;

  float mu_cur_val,
        mu_nxt_val,
        mu_diff,
        rz_diff;

  int64_t heap_n,
          heap_idx,
          mu_idx;

  int64_t idir, cell,
          tile, tile_idx, n_tile_idx;
  int64_t _cell_pos;
  int32_t _idir, 
          _tile_val;

  heap_n = m_index_heap_size;

  for (idir=0; idir<6; idir++) {
    for (cell=0; cell<m_num_verts; cell++) {
      //n_tile_idx = getVali( BUF_TILE_IDX_N, cell );
      n_tile_idx = getValI( BUF_TILE_IDX_N, cell );
      for (tile_idx=0; tile_idx<n_tile_idx; tile_idx++) {
        //tile = getVali( BUF_TILE_IDX, cell, tile_idx );
        tile = getValI( BUF_TILE_IDX, tile_idx, cell );

        mu_idx = getMuIdx(idir, cell, tile);

        //heap_idx = getVal_ih( BUF_RESIDUE_CELL_HEAP, mu_idx );
        heap_idx = getValL( BUF_RESIDUE_CELL_HEAP, mu_idx );

        //mu_cur_val = getVal( BUF_MU,      idir, cell, tile );
        //mu_nxt_val = getVal( BUF_MU_NXT,  idir, cell, tile );
        mu_cur_val = getValF( BUF_MU,      idir, tile, cell );
        mu_nxt_val = getValF( BUF_MU_NXT,  idir, tile, cell );

        mu_diff = fabs(mu_cur_val - mu_nxt_val);

        //rz_diff = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx );
        rz_diff = getValF( BUF_RESIDUE_HEAP, heap_idx );

        if (fabs(mu_diff - rz_diff) > m_eps_zero) {
          return -1;
        }

      }
    }
  }

  return 0;
}

int32_t BeliefPropagation::indexHeap_consistency(void) {
  int64_t i, idx_bp, idx, idx_child;
  std::vector<int> tbuf;
  int32_t err_code=0;
  float f_d, val_a, val_b;

  int64_t heap_n;

  int64_t idir, cell_pos, tile_val;
  int64_t _cell_pos;
  int32_t _idir, 
          _tile_val;

  heap_n = m_index_heap_size;

  for (i=0; i<heap_n; i++) { tbuf.push_back(0); }

  //---

  // do some sanity checks on getMuIdx conversion to make
  // sure it's 1-1 and onto
  //
  for (idir=0; idir<6; idir++) {
    for (cell_pos=0; cell_pos<m_num_verts; cell_pos++) {
      for (tile_val=0; tile_val<m_num_values; tile_val++) {

        idx = getMuIdx( idir, cell_pos, tile_val );

        if ((idx < 0) || (idx >= heap_n)) { return -1; }
        if (tbuf[idx]!=0) { return -2; }

        tbuf[idx] = 1;

        getMuPos( idx, &_idir, &_cell_pos, &_tile_val);

        if ((idir != _idir) ||
            (cell_pos != _cell_pos) ||
            (tile_val != _tile_val)) {
          return -100;
        }

      }
    }
  }
  for (i=0; i<heap_n; i++) { if (tbuf[i] != 1) { return -3; } }

  //---

  // check heap_cell_bp is 1-1 and onto
  //
  for (i=0; i<heap_n; i++) { tbuf[i] = 0; }
  for (i=0; i<heap_n; i++) {
    //idx = getVal_ih( BUF_RESIDUE_HEAP_CELL_BP, i );
    idx = getValL( BUF_RESIDUE_HEAP_CELL_BP, i );
    if ((idx < 0) || (idx >= heap_n)) { return -4; }
    if (tbuf[idx] !=0 ) { return -5; }
    tbuf[idx] = 1;
  }
  for (i=0; i<heap_n; i++) { if (tbuf[i] != 1) { return -6; } }

  //---

  // check to make sure idx_heap is 1-1 and onto
  //
  for (i=0; i<heap_n; i++) { tbuf[i] = 0; }
  for (i=0; i<heap_n; i++) {
    //idx = getVal_ih( BUF_RESIDUE_CELL_HEAP, i );
    idx = getValL( BUF_RESIDUE_CELL_HEAP, i );
    if ((idx < 0) || (idx >= heap_n)) { return -7; }
    if (tbuf[idx] !=0 ) { return -8; }
    tbuf[idx] = 1;
  }
  for (i=0; i<heap_n; i++) { if (tbuf[i] != 1) { return -9; } }

  //---

  // check idx_heap maps to entry that points back
  // to it.
  //
  for (i=0; i<heap_n; i++) {
    //idx = getVal_ih( BUF_RESIDUE_CELL_HEAP, i );
    //if (i != getVal_ih( BUF_RESIDUE_HEAP_CELL_BP, idx )) { return -10; }
    idx = getValL( BUF_RESIDUE_CELL_HEAP, i );
    if (i != getValL( BUF_RESIDUE_HEAP_CELL_BP, idx )) { return -10; }
  }

  //---

  // check heap property is consistent
  //
  for (i=0; i<heap_n; i++) {
    idx_child = _lchild(i);
    if (idx_child < 0) { return -11; }
    if (idx_child < heap_n) {
      //val_a = getVal_ihf( BUF_RESIDUE_HEAP, i );
      //val_b = getVal_ihf( BUF_RESIDUE_HEAP, idx_child );
      val_a = getValF( BUF_RESIDUE_HEAP, i );
      val_b = getValF( BUF_RESIDUE_HEAP, idx_child );
      if ( val_a < val_b ) { return -12; }
    }

    idx_child = _rchild(i);
    if (idx_child < 0) { return -13; }
    if (idx_child < heap_n) {
      //val_a = getVal_ihf( BUF_RESIDUE_HEAP, i );
      //val_b = getVal_ihf( BUF_RESIDUE_HEAP, idx_child );
      val_a = getValF( BUF_RESIDUE_HEAP, i );
      val_b = getValF( BUF_RESIDUE_HEAP, idx_child );
      if ( val_a < val_b ) { return -14;
      }
    }

  }

  return 0;
}

