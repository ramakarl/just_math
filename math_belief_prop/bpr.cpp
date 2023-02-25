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
// Sample utils
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
int64_t BeliefPropagation::getMuPos( int64_t idx, int32_t *idir, int64_t *cell, int32_t *tile ) {
  int64_t _idir,
          _cell,
          _tile;
  int64_t q = 0;

  _idir = idx / (m_num_verts * m_num_values);
  q += _idir * m_num_verts * m_num_values;
  _cell = (idx - q) / m_num_values;
  q += _cell * m_num_values;
  _tile = (idx - q);

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

  val_a       = getVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_a );
  cell_idx_a  = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_a );

  val_b       = getVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_b );
  cell_idx_b  = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_b );

  SetVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_a, val_b );
  SetVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_a, cell_idx_b );

  SetVal_ihf( BUF_RESIDUE_HEAP,         heap_idx_b, val_a );
  SetVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_b, cell_idx_a );

  //cell_idx_a = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_a );
  //cell_idx_b = getVal_ih ( BUF_RESIDUE_HEAP_CELL_BP, heap_idx_b );

  //SetVal_ih ( BUF_RESIDUE_CELL_HEAP,  cell_idx_a, heap_idx_a );
  //SetVal_ih ( BUF_RESIDUE_CELL_HEAP,  cell_idx_b, heap_idx_b );

  SetVal_ih ( BUF_RESIDUE_CELL_HEAP,  cell_idx_a, heap_idx_b );
  SetVal_ih ( BUF_RESIDUE_CELL_HEAP,  cell_idx_b, heap_idx_a );

}

static inline int64_t _lchild(int64_t _ii) { return ((2*(_ii))+1); }
static inline int64_t _rchild(int64_t _ii) { return ((2*(_ii))+2); }
static inline int64_t _parent(int64_t _ii) { return (_ii-1)/2; }

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

  heap_n = 6*m_num_verts*m_num_values;

  SetVal_ihf( BUF_RESIDUE_HEAP, heap_idx, val );

  // bubble up value until we can't anymore
  //
  while (heap_idx > 0) {
    heap_idx_par = _parent(heap_idx);

    par_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_par );

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
    max_heap_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_par );

    if (heap_idx_lc < heap_n) {
      child_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_lc );
      if (child_val > max_heap_val) {
        max_heap_idx = heap_idx_lc;
        max_heap_val = child_val;
      }
    }

    if (heap_idx_rc < heap_n) {
      child_val = getVal_ihf( BUF_RESIDUE_HEAP, heap_idx_rc );
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

