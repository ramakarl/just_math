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

#include <algorithm>
#include "mersenne.h"
#include "dataptr.h"

#include "belief_propagation.h"
#include "main_belief_propagation.h"

extern opt_t g_opt;



//------------------------------------//
//  _            _   _                //
// | |_ ___  ___| |_(_)_ __   __ _    //
// | __/ _ \/ __| __| | '_ \ / _` |   //
// | ||  __/\__ \ |_| | | | | (_| |   //
//  \__\___||___/\__|_|_| |_|\__, |   //
//                           |___/    //
//------------------------------------//

// custom size (basic test)
//
int test0() {
  int ret;

  BeliefPropagation bp;
  ret = bp.init_CSV(4,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  //bp.init(4,3,1);
  bp.debugPrint();
  return 0;
}

// test filterDiscard
//
int test1() {
  int ret;
  std::vector<int32_t> discard_list;

  discard_list.push_back(35);
  discard_list.push_back(36);
  discard_list.push_back(37);
  discard_list.push_back(38);
  discard_list.push_back(39);
  discard_list.push_back(40);
  discard_list.push_back(41);
  discard_list.push_back(42);
  discard_list.push_back(43);
  discard_list.push_back(44);

  BeliefPropagation bp;
  //bp.init(4,3,1);
  ret = bp.init_CSV(4,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  bp.filterDiscard(6, discard_list);
  bp.debugPrint();
  return 0;
}

// test filterKeep
//
int test2() {
  int ret;
  std::vector<int32_t> keep_list;

  keep_list.push_back(1);
  keep_list.push_back(2);
  keep_list.push_back(3);
  keep_list.push_back(4);
  keep_list.push_back(5);
  keep_list.push_back(6);

  BeliefPropagation bp;
  //bp.init(4,3,1);
  ret = bp.init_CSV(4,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  bp.filterKeep(6, keep_list);
  bp.debugPrint();
  return 0;
}

int test3() {
  int ret;
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(4,3,1);
  ret = bp.init_CSV(4,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"+000") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(3,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(3,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(3,0,0), keep_list);

  //---

  bp.NormalizeMU();

  bp.debugPrint();

  return 0;
}

int test4_() {
  int ret;

  float maxdiff;
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(4,3,1);
  ret = bp.init_CSV(4,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"+000") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(3,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(3,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(3,0,0), keep_list);

  //---

  bp.NormalizeMU();

  printf("---\nBEFORE:\n");
  bp.debugPrint();

  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[0] got diff: %f\n", maxdiff);

  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[1] got diff: %f\n", maxdiff);

  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[2] got diff: %f\n", maxdiff);

  printf("\n\n---\nAFTER:\n");
  bp.debugPrint();
  printf("---\n\n");

  return 0;
}

int test4() {
  int ret;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  float maxdiff;
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,1);
  ret = bp.init_CSV(3,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  bp.m_rate = 1.0;

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);

  //---

  bp.NormalizeMU();

  printf("---\nBEFORE:\n");
  bp.debugPrint();

  bp.WriteBoundaryMU();
  //bp.WriteBoundaryMUbuf(BUF_MU_NXT);
  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[0] got diff: %f\n", maxdiff);

  printf("\n\n---\niter %i:\n", 0);
  bp.debugPrint();
  printf("---\n\n");



  bp.WriteBoundaryMU();
  //bp.WriteBoundaryMUbuf(BUF_MU_NXT);
  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();


  printf("[1] got diff: %f\n", maxdiff);

  printf("\n\n---\niter %i:\n", 1);
  bp.debugPrint();
  printf("---\n\n");

  //---

  bp.WriteBoundaryMU();
  //bp.WriteBoundaryMUbuf(BUF_MU_NXT);
  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[2] got diff: %f\n", maxdiff);

  printf("\n\n---\niter %i:\n", 2);
  bp.debugPrint();
  printf("---\n\n");

  //---

  bp.WriteBoundaryMU();
  //bp.WriteBoundaryMUbuf(BUF_MU_NXT);
  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[3] got diff: %f\n", maxdiff);

  printf("\n\n---\niter %i:\n", 3);
  bp.debugPrint();
  printf("---\n\n");

  //---

  bp.WriteBoundaryMU();
  //bp.WriteBoundaryMUbuf(BUF_MU_NXT);
  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[4] got diff: %f\n", maxdiff);

  printf("\n\n---\niter %i:\n", 4);
  bp.debugPrint();
  printf("---\n\n");

  //---

  bp.WriteBoundaryMU();
  //bp.WriteBoundaryMUbuf(BUF_MU_NXT);
  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[5] got diff: %f\n", maxdiff);

  printf("\n\n---\niter %i:\n", 5);
  bp.debugPrint();
  printf("---\n\n");

  //---

  printf("\n\n---\nAFTER:\n");
  bp.debugPrint();
  printf("---\n\n");

  return 0;
}

// test run until converged
//
int test5() {
  int ret;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  int iter, max_iter=100;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,1);
  ret = bp.init_CSV(3,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  bp.m_eps_converge = 1.0/1024.0;


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);

  //---

  for (iter=0; iter<max_iter; iter++) {
    bp.NormalizeMU();

    printf("---\nBEFORE:\n");
    bp.debugPrint();

    bp.WriteBoundaryMU();
    //bp.WriteBoundaryMUbuf(BUF_MU_NXT);
    bp.BeliefProp();
    bp.NormalizeMU(BUF_MU_NXT);
    bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );

    maxdiff = bp.MaxDiffMU();
    bp.UpdateMU();

    if (fabs(maxdiff) < _eps) { break; }
  }

  printf("count: %i\n", iter);
  bp.debugPrint();

  return 0;
}

// test run until converged
//
int test5_1() {
  int ret;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,1);
  ret = bp.init_CSV(3,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);

  //---

  bp.NormalizeMU();
  for (iter=0; iter<max_iter; iter++) {

    maxdiff = bp.step(1);

    /*
    bp.NormalizeMU();

    printf("---\nBEFORE:\n");
    bp.debugPrint();

    bp.BeliefProp();
    bp.NormalizeMU(BUF_MU_NXT);
    maxdiff = bp.MaxDiffMU();
    bp.UpdateMU();
    */

    if (fabs(maxdiff) < _eps) { break; }
  }

  printf("count: %i\n", iter);
  bp.debugPrint();

  return 0;
}



// cull 2x2 grid with 0,0 fixed with r003
// result should be:
//
//  (0,1)r000  (1,1)r001
//  (0,0)r003  (1,0)r002
//
//
int test_cull0() {

  int ret = 0;
  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(2,2,1);
  ret = bp.init_CSV(2,2,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);
  bp.cellFillAccessed(0, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  ret = bp.cellConstraintPropagate();

  printf("ret: %i\n", ret);
  bp.debugPrint();

  return 0;
}

// cull 3x3 grid with 0,0 fixed with r003
//
//
//
int test_cull1() {

  int ret = 0;
  int iter=0, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,1);
  ret = bp.init_CSV(3,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);
  bp.cellFillAccessed(0, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  ret = bp.cellConstraintPropagate();

  printf("ret: %i\n", ret );
  bp.debugPrint();

  return 0;
}

// cull 3x3 grid with 0,0 fixed with r003
//
//
//
int test_cull2() {

  int ret = 0;
  int iter=0, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,1);
  ret = bp.init_CSV(3,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);
  bp.cellFillAccessed(0, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);
  bp.cellFillAccessed(4, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);
  bp.cellFillAccessed(8, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  ret = bp.cellConstraintPropagate();

  printf("ret: %i\n", ret );
  bp.debugPrint();

  return 0;
}

// cull 3x3x2 grid with 0,0 fixed with r003
//
//
//
int test_cull3() {

  int ret = 0;
  int iter=0, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,2);
  ret = bp.init_CSV(3,3,2, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);
  bp.cellFillAccessed(0, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);
  bp.cellFillAccessed(0, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);
  bp.cellFillAccessed(4, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(1,1,1), keep_list);
  bp.cellFillAccessed(4, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,1), keep_list);
  bp.cellFillAccessed(8, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,1), keep_list);
  bp.cellFillAccessed(8, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  ret = bp.cellConstraintPropagate();

  printf("ret: %i\n", ret );
  bp.debugPrint();

  return 0;
}

// cull 2x2x1 to test an issue with directionality of
// rule constraints.
//
// Set up with `.`, `.r` and `^`. All `^` should be culled.
//
//
//
int test_cull4() {

  int ret = 0;
  int iter=0, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(2,2,1);
  ret = bp.init_CSV(2,2,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"^012") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);
  bp.cellFillAccessed(0, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"^011") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);
  bp.cellFillAccessed(1, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"^013") );
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);
  bp.cellFillAccessed(2, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"^010") );
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);
  bp.cellFillAccessed(3, bp.m_grid_note_idx);
  bp.unfillAccessed(bp.m_grid_note_idx);

  ret = bp.cellConstraintPropagate();

  printf("ret: %i\n", ret );
  bp.debugPrint();

  return 0;
}

// cull boundary
//
int test6() {
  int ret;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,1);
  ret = bp.init_CSV(3,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  bp.CullBoundary();

  bp.debugPrint();
  return 0;

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);

  //---

  for (iter=0; iter<max_iter; iter++) {
    bp.NormalizeMU();

    printf("---\nBEFORE:\n");
    bp.debugPrint();

    bp.BeliefProp();
    bp.NormalizeMU(BUF_MU_NXT);
    maxdiff = bp.MaxDiffMU();
    bp.UpdateMU();

    if (fabs(maxdiff) < _eps) { break; }
  }

  printf("count: %i\n", iter);
  bp.debugPrint();

  return 0;
}

int test_realize0() {
  int ret;
  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.m_seed = 18;

  //bp.init(2,2,1);
  //bp.init(3,3,1);
  ret = bp.init_CSV(2,2,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //bp.debugPrintC();
  //bp.debugPrintS();

  ret = bp.realize();

  //bp.CullBoundary();

  printf("got: %i\n", ret);

  bp.debugPrint();
  return 0;
}

int test_realize1() {
  int ret;
  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.m_seed = 18;

  //bp.init(3,3,1);
  //bp.init(3,3,1);
  ret = bp.init_CSV(3,3,1, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }


  //bp.debugPrintC();
  //bp.debugPrintS();

  ret = bp.realize();

  //bp.CullBoundary();

  printf("got: %i\n", ret);

  bp.debugPrint();
  return 0;
}

int test_realize2(int x, int y, int z) {
  int ret;
  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  //bp.init(x,y,z);
  ret = bp.init_CSV(x,y,z, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  ret = bp.realize();

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);

  bp.debugPrint();
  return 0;
}


int test_step0(void) {
  int ret;
  int iter, max_iter=1024;
  float maxdiff, _eps = (1.0/(1024.0));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  int x,y,z;
  int64_t n_it, it;

  x = 4;
  y = 4;
  z = 4;

  bp.m_seed = 0;

  ret = bp.init_CSV(x,y,z, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  ret = bp.start();
  if (ret<0) { return ret; }

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {
    ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  bp.debugPrint();

  return ret;
}

// test run until converged
//
int test_step1() {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  x = 3;
  y = 3;
  z = 1;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  int iter, max_iter=100;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  //bp.init(3,3,1);
  ret = bp.init_CSV(x,y,z, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  bp.m_eps_converge = 1.0/1024.0;

  bp.m_verbose = 3;


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);

  //---
  bp.m_seed = 0;

  ret = bp.start();
  if (ret<0) { return ret; }

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {
    ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  bp.debugPrint();

  return ret;
}


// test run until converged
// with SVD optimization turned on
//
int test_step2() {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  x = 3;
  y = 3;
  z = 1;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  int iter, max_iter=100;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.m_use_svd = 1;

  ret = bp.init_CSV(x,y,z, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  bp.m_eps_converge = 1.0/1024.0;

  bp.m_verbose = 3;


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T003") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"T000") );
  bp.filterKeep( bp.getVertex(1,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)"T002") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|001") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"|000") );
  keep_list.push_back( bp.tileName2ID((char *)"T001") );
  bp.filterKeep( bp.getVertex(2,1,0), keep_list);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);

  //---
  bp.m_seed = 0;

  ret = bp.start();
  if (ret<0) { return ret; }

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {
    ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  bp.debugPrint();

  return ret;
}



int test_wfc0(int x, int y, int z) {
  int ret;
  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  //bp.init(x,y,z);
  ret = bp.init_CSV(x,y,z, g_opt.fn_name, g_opt.fn_rule);
  if (ret<0) { return ret; }

  ret = bp.wfc();

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);

  bp.debugPrint();
  return 0;
}



void _debugstate() {
  int a, b, i, j, k, d;

  BeliefPropagation bp;
  bp.init(3,3,1);

  for (a=0; a<bp.m_num_values; a++) {
    for (b=0; b<bp.m_num_values; b++) {
      for (i=0; i<6; i++) {
        printf("%s(%i) -(%s(%i))-> %s(%i): %f\n",
            bp.m_tile_name[a].c_str(), a,
            bp.m_dir_desc[i].c_str(), i,
            bp.m_tile_name[b].c_str(), b,
            bp.getValF(BUF_F, a, b, i));
      }
    }

  }
}

int run_test(int test_num) {
  switch(test_num) {
    case 0:
      test0();
      break;
    case 1:
      test1();
      break;
    case 2:
      test2();
      break;
    case 3:
      test3();
      break;
    case 4:
      test4();
      break;
    case 5:
      test5();
      break;
    case 6:
      test6();
      break;

    case 7:
      test_cull0();
      break;
    case 8:
      test_cull1();
      break;
    case 9:
      test_cull2();
      break;
    case 10:
      test_cull3();
      break;
    case 11:
      test_cull4();
      break;

    case 12:
      test_realize0();
      break;
    case 13:
      test_realize1();
      break;
    case 14:
      test_realize2(4,4,4);
      break;

    case 15:
      test_wfc0(4,4,4);
      break;

    case 16:
      test_step0();
      break;

    case 17:
      test_step1();
      break;

    case 18:
      test_step2();
      break;

    default:
      return -1;

  }

  return 0;
}

