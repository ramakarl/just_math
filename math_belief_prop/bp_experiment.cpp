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

#include "bp_experiment.h"

int run_experiment(BeliefPropagation &bpc) {
  int ret = 0;

  if (bpc.op.experiment_idx == 0) {
    ret = bp_experiment_tile_fix(bpc);
  }

  return ret;
}


int bp_experiment_tile_fix(BeliefPropagation &bpc) {

  int32_t n_tile,
          tile_idx,
          tile_val,
          x,y,z,
          pos_side[3],
          pos_corner[3],
          pos_center[3],
          pos_edge[3],
          cur_x, cur_y, cur_z;
  int64_t cell_center,
          cell_corner,
          cell_side,
          cell_edge,
          cur_cell;

  int32_t _ix,_iy,_iz, n_orig, n_cur;
  int64_t _cell;
  float dist,
        min_dist, max_dist;


  std::vector< std::vector< std::vector< int32_t > > > grid_sum;
  std::vector< std::vector< int32_t > > _vv;
  std::vector< int32_t > _v;

  bpc.op.verbose = VB_SUPPRESS;

  bp_restart(bpc);

  for (x=0; x<bpc.m_bpres.x; x++) {
    _vv.clear();
    grid_sum.push_back( _vv );
    for (y=0; y<bpc.m_bpres.y; y++) {
      _v.clear();
      grid_sum[x].push_back(_v);
      for (z=0; z<bpc.m_bpres.z; z++) {
        grid_sum[x][y].push_back(0);
      }
    }
  }

  x = bpc.m_bpres.x/2;
  y = bpc.m_bpres.y/2;
  z = bpc.m_bpres.z/2;
  cell_center = bpc.getVertex(x,y,z);
  pos_center[0] = x;
  pos_center[1] = y;
  pos_center[2] = z;

  x = 0; y = 0; z = 0;
  cell_corner = bpc.getVertex(x,y,z);
  pos_corner[0] = x;
  pos_corner[1] = y;
  pos_corner[2] = z;


  x = 0; y = 0; z = 0;
  if ( (bpc.m_bpres.x >= bpc.m_bpres.z) &&
       (bpc.m_bpres.y >= bpc.m_bpres.z) ) {
    x = bpc.m_bpres.x/2;
    y = bpc.m_bpres.y/2;
    z = 0;
  }
  else if ( (bpc.m_bpres.x >= bpc.m_bpres.y) &&
            (bpc.m_bpres.z >= bpc.m_bpres.y) ) {
    x = bpc.m_bpres.x/2;
    y = 0;
    z = bpc.m_bpres.z/2;
  }
  else if ( (bpc.m_bpres.y >= bpc.m_bpres.x) &&
            (bpc.m_bpres.z >= bpc.m_bpres.x) ) {
    x = 0;
    y = bpc.m_bpres.y/2;
    z = bpc.m_bpres.z/2;
  }
  cell_side = bpc.getVertex(x,y,z);
  pos_side[0] = x;
  pos_side[1] = y;
  pos_side[2] = z;


  x = 0; y = 0; z = 0;
  if( (bpc.m_bpres.x >= bpc.m_bpres.y) &&
       (bpc.m_bpres.x >= bpc.m_bpres.z) ) {
    x = bpc.m_bpres.x/2;
    y = 0;
    z = 0;
  }
 
  else if ( (bpc.m_bpres.y >= bpc.m_bpres.x) &&
            (bpc.m_bpres.y >= bpc.m_bpres.z) ) {
    x = 0;
    y = bpc.m_bpres.y/2;
    z = 0;
  }

  else if ( (bpc.m_bpres.z >= bpc.m_bpres.x) &&
            (bpc.m_bpres.z >= bpc.m_bpres.y) ) {
    x = 0;
    y = 0;
    z = bpc.m_bpres.z/2;
  }
  cell_edge = bpc.getVertex(x,y,z);
  pos_edge[0] = x;
  pos_edge[1] = y;
  pos_edge[2] = z;



  cur_x = pos_center[0];
  cur_y = pos_center[1];
  cur_z = pos_center[2];
  cur_cell = cell_center;

  /*
  cur_x = pos_corner[0];
  cur_y = pos_corner[1];
  cur_z = pos_corner[2];
  cur_cell = cell_corner;

  cur_x = pos_side[0];
  cur_y = pos_side[1];
  cur_z = pos_side[2];
  cur_cell = cell_side;

  cur_x = pos_edge[0];
  cur_y = pos_edge[1];
  cur_z = pos_edge[2];
  cur_cell = cell_edge;
  */

  n_tile = bpc.getValI( BUF_TILE_IDX_N, cur_cell );
  for (tile_idx=0; tile_idx<n_tile; tile_idx++) {

    bpc._saveTileIdx();

    tile_val = bpc.getValI( BUF_TILE_IDX, tile_idx, cur_cell );
    bpc.CollapseAndPropagate( cur_cell, tile_val, tile_idx );

    printf("# [%i][%i][%i]{%i} tile:%i (idx:%i)\n",
        (int)cur_x,(int)cur_y,(int)cur_z, (int)cur_cell, (int)tile_val, (int)tile_idx);
    //bpc.debugPrintTerse();

    min_dist = -1.0;
    max_dist = -1.0;

    for (_iz=0; _iz<bpc.m_bpres.z; _iz++) {
      for (_iy=0; _iy<bpc.m_bpres.y; _iy++) {
        for (_ix=0; _ix<bpc.m_bpres.x; _ix++) {

          _cell = bpc.getVertex(_ix,_iy,_iz);

          n_orig  = bpc.getValI( BUF_SAVE_TILE_IDX_N, _cell );
          n_cur   = bpc.getValI( BUF_TILE_IDX_N, _cell );

          if (n_cur != n_orig) {

            grid_sum[_ix][_iy][_iz] += (n_orig - n_cur);
            //grid_sum[_ix][_iy][_iz] ++;

            dist = sqrt( (float)(_ix - cur_x)*(_ix - cur_x) + (float)(_iy - cur_y)*(_iy - cur_y) + (float)(_iz - cur_z)*(_iz - cur_z) );
            //printf("%f %i\n", (float)dist, (int)(n_orig - n_cur) );
            //printf("%f %i\n", (float)dist, 1);
            //printf("%i %i %i %i\n", (int)_ix, (int)_iy, (int)_iz, (int)(n_orig - n_cur));
          }

        }
      }
    }
    //printf("\n\n");

    bpc._restoreTileIdx();

  }

  /*
  for (z=0; z<bpc.m_bpres.z; z++) {
    for (y=0; y<bpc.m_bpres.y; y++) {
      for (x=0; x<bpc.m_bpres.x; x++) {

        if (grid_sum[x][y][z] > 0) {
          printf("%i %i %i %i\n", (int)x, (int)y, (int)z, (int)grid_sum[x][y][z]);
        }

      }
    }
  }
  */

  int lf=0;
  for (x=0; x<bpc.m_bpres.x; x++) {
    lf=0;
    for (y=0; y<bpc.m_bpres.y; y++) {

      if (grid_sum[x][y][z] > 0) {
        printf("%i %i %i\n", (int)x, (int)y, (int)grid_sum[x][y][0]);
        lf=1;
      }

    }
    if (lf) { printf("\n"); }
  }

  return 0;
}


