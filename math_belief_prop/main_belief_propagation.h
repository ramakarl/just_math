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


#ifndef MAIN_BELIEF_PROPAGATION_HPP
#define MAIN_BELIEF_PROPAGATION_HPP

typedef struct _opt_t {
  float alpha;
  int alg_idx;
  std::string fn_name;
  std::string fn_rule;

  std::string tileset_fn,
              tilemap_fn,
              tileobj_fn,
              outstl_fn;
  int32_t tileset_stride_x,
          tileset_stride_y;
  int32_t tileset_margin,
          tileset_spacing;
  int32_t tileset_width,
          tileset_height;

  int tiled_reverse_y;

} opt_t;

extern opt_t g_opt;




// tests..
//
int test0();
int test1();
int test2();
int test3();
int test4_();
int test4();
int test5();
int test5_1();
int test_cull0();
int test_cull1();
int test_cull2();
int test_cull3();
int test_cull4();
int test6();
int test_realize0();
int test_realize1();
int test_realize2(int x, int y, int z);
int test_wfc0(int x, int y, int z);

int test_step0();
int test_step1();

int run_test(int test_num);

#endif

