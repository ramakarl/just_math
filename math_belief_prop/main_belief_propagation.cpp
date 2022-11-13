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

#include "belief_propagation.h"

#include "pd_getopt.h"
extern char *optarg;

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
  BeliefPropagation bp;
  bp.init(4,3,1);
  bp.debugPrint();
  return 0;
}

// test filterDiscard
//
int test1() {
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
  bp.init(4,3,1);

  bp.filterDiscard(6, discard_list);
  bp.debugPrint();
  return 0;
}

// test filterKeep
//
int test2() {
  std::vector<int32_t> keep_list;

  keep_list.push_back(1);
  keep_list.push_back(2);
  keep_list.push_back(3);
  keep_list.push_back(4);
  keep_list.push_back(5);
  keep_list.push_back(6);

  BeliefPropagation bp;
  bp.init(4,3,1);

  bp.filterKeep(6, keep_list);
  bp.debugPrint();
  return 0;
}

int test3() {
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  bp.init(4,3,1);

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

  float maxdiff;
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;
  bp.init(4,3,1);

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
  bp.init(3,3,1);

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

// test run until converged
//
int test5() {

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
  bp.init(3,3,1);

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

// test run until converged
//
int test5_1() {

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
  bp.init(3,3,1);

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

    maxdiff = bp.step();

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
  bp.init(2,2,1);

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
  bp.init(3,3,1);

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
  bp.init(3,3,1);

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
  bp.init(3,3,2);

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
  bp.init(2,2,1);

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
  bp.init(3,3,1);

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

  bp.init(2,2,1);
  //bp.init(3,3,1);

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

  bp.init(3,3,1);
  //bp.init(3,3,1);

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

  bp.init(x,y,z);
  ret = bp.realize();

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);

  bp.debugPrint();
  return 0;
}



int test_wfc0(int x, int y, int z) {
  int ret;
  int iter, max_iter=10;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.init(x,y,z);
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

    default:
      return -1;

  }

  return 0;
}

//--------------------------------------//
//  _____     _                         //
// |___ /  __| |    _ __  _ __   __ _   //
//   |_ \ / _` |   | '_ \| '_ \ / _` |  //
//  ___) | (_| |   | |_) | | | | (_| |  //
// |____/ \__,_|   | .__/|_| |_|\__, |  //
//                 |_|          |___/   //
//--------------------------------------//


#include "camera3d.h"
#include "file_png.h"

uchar*    m_img;
DataPtr   m_vol[4];

void alloc_img (int xres, int yres) {
  // RGB, 3 bytes/pix
  int sz = xres * yres * 3;
  m_img = (uchar*) malloc ( sz );
  // default gray
  memset( m_img, 128, sz);
}

void set_pixel (uchar* img, int x, int y, int xres, int yres, uchar r, uchar g, uchar b) {
  int px = 3*(y*xres+x);
  *(m_img + px + 0) = r;
  *(m_img + px + 1) = g;
  *(m_img + px + 2) = b;
}

void alloc_volume (int id, Vector3DI res, int chan) {
  uint64_t cnt = res.x*res.y*res.z;
  m_vol[id].Resize( chan*sizeof(float), cnt, 0x0, DT_CPU );
  memset( (void *)(m_vol[id].getPtr(0)), 0, sizeof(float)*cnt);
}

Vector4DF getVoxel4 ( int id, int x, int y, int z, Vector3DI vres ) {
  Vector4DF* dat = (Vector4DF*) m_vol[id].getPtr ( (z*vres.y + y)*vres.x + x );
  return *dat;
}

Vector3DF intersectLineBox(Vector3DF p1, Vector3DF p2, Vector3DF bmin, Vector3DF bmax) {

  // p1 = ray position, p2 = ray direction
  //
  register float ht[8];
  ht[0] = (bmin.x - p1.x)/p2.x;
  ht[1] = (bmax.x - p1.x)/p2.x;
  ht[2] = (bmin.y - p1.y)/p2.y;
  ht[3] = (bmax.y - p1.y)/p2.y;
  ht[4] = (bmin.z - p1.z)/p2.z;
  ht[5] = (bmax.z - p1.z)/p2.z;
  ht[6] = fmax(fmax(fmin(ht[0], ht[1]), fmin(ht[2], ht[3])), fmin(ht[4], ht[5]));
  ht[7] = fmin(fmin(fmax(ht[0], ht[1]), fmax(ht[2], ht[3])), fmax(ht[4], ht[5]));
  ht[6] = (ht[6] < 0 ) ? 0.0 : ht[6];
  return Vector3DF( ht[6], ht[7], (ht[7]<ht[6] || ht[7]<0) ? -1 : 0 );
}

void raycast_cpu ( Vector3DI vres, Camera3D* cam, int id, uchar* img, int xres, int yres, Vector3DF vmin, Vector3DF vmax ) {
  Vector3DF rpos, rdir;
  Vector4DF clr;

  Vector3DF wp, dwp, p, dp, t;
  Vector3DF vdel = vres;
  Vector4DF val;
  int iter;
  float alpha, k;
  float pStep = 0.1;          // volume quality   - lower=better (0.01), higher=worse (0.1)
  float kDensity = 3.0;       // volume density   - lower=softer, higher=more opaque
  float kIntensity = 16.0;    // volume intensity - lower=darker, higher=brighter
  float kWidth = 3.0;         // transfer func    - lower=broader, higher=narrower (when sigmoid transfer enabled)

  // for each pixel in image..
  for (int y=0; y < yres; y++) {
    for (int x=0; x < xres; x++) {

      // background color
      clr.Set(0,0,0,0);

      // get camera ray
      rpos = cam->getPos();
      rdir = cam->inverseRay ( x, y, xres, yres );
      rdir.Normalize();

      // intersect with volume box
      t = intersectLineBox ( rpos, rdir, vmin, vmax );
      if ( t.z >= 0 ) {
        // hit volume, start raycast...
        wp = rpos + rdir * (t.x + pStep);                     // starting point in world space
        dwp = (vmax-vmin) * rdir * pStep;                     // ray sample stepping in world space
        p = Vector3DF(vres) * (wp - vmin) / (vmax-vmin);    // starting point in volume
        dp = rdir * pStep;                // step delta along ray

        // accumulate along ray
        for (iter=0; iter < 512 && clr.w < 0.99 && p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < vres.x && p.y < vres.y && p.z < vres.z; iter++) {
          val = getVoxel4 ( 0, p.x, p.y, p.z, vres ); // get voxel value
          alpha = 1.0 / (1+exp(-(val.w-1.0)*kWidth)); // opacity = sigmoid transfer - accentuates boundaries at 0.5
          clr += Vector4DF(val.x,val.y,val.z, 0) * (1-clr.w) * alpha * kIntensity * pStep;  // accumulate color
          clr.w += alpha * kDensity * pStep;          // attenuate alpha
          p += dp;                           // next sample
        }
        if (clr.x > 1.0) clr.x = 1;
        if (clr.y > 1.0) clr.y = 1;
        if (clr.z > 1.0) clr.z = 1;
        clr *= 255.0;
      }
      // set pixel
      set_pixel(img, x, y, xres, yres, clr.x, clr.y, clr.z );
    }
  }
}

void visualize_belief ( BeliefPropagation& src, int bp_id, int vol_id, Vector3DI vres ) {

   Vector4DF* vox = (Vector4DF*) m_vol[ vol_id ].getPtr (0);
   float maxv;

   // printf ( "  visualize: vol %p, verts %d, res %dx%dx%d\n", vox, src.getNumVerts(), vres.x, vres.y, vres.z);

   // map belief to RGBA voxel
   for ( uint64_t j=0; j < src.getNumVerts(); j++ ) {
     src.getVertexBelief (j);

     // red
     maxv = 0.0;
     for (int k=1; k <= 30; k++) {
        maxv = std::max(maxv, src.getVal( bp_id, k ));
     }
     vox->x = maxv;

     // green
     maxv = 0.0;
     for (int k=31; k <= 60; k++) {
        maxv = std::max(maxv, src.getVal( bp_id, k ));
     }
     vox->y = maxv;

     // blue
     maxv = 0.0;
     for (int k=61; k <= 90; k++) {
        maxv = std::max(maxv, src.getVal( bp_id, k ));
     }
     vox->z = maxv;

     vox->w = std::max(vox->x, std::max(vox->y, vox->z));
     vox++;
   }
}

//------------//
//       _ _  //
//   ___| (_) //
//  / __| | | //
// | (__| | | //
//  \___|_|_| //
//            //
//------------//


// DEBUG MAIN
//


/*
bool handle_args ( int& arg, int argc, char **argv, char& ch, char*& optarg ) {
   if (arg >= argc) return false;

   char dash = argv[arg][0];
   if (dash=='-') {
     ch = argv[arg][1];
     optarg = argv[ arg+1 ];
     printf ( "arg: %d, ch: %c, opt: %s\n", arg, ch, optarg);
     arg += 2;
   } else {
     ch = argv[arg][0];
     optarg = 0;
     printf ( "arg: %d, ch: %c, no opt\n", arg, ch);
     arg++;
   }
   return (arg <= argc);
}
*/

// #include "getopt.h"

void show_usage(FILE *fp) {
  fprintf(fp, "usage:\n");
  fprintf(fp, "\n");
  fprintf(fp, "    bpc [-h] [-v] [-N <name_file>] [-R <rule_file] [-C <fn>] [-T <test#>] [-D <#>] [-X <#>] [-Y <#>] [-Z <#>]\n");
  fprintf(fp, "\n");
  fprintf(fp, "  -N <fn>  CSV name file\n");
  fprintf(fp, "  -R <fn>  CSV rule file\n");
  fprintf(fp, "  -C <fn>  constrained realization file\n");
  fprintf(fp, "  -W       run 'wave function collapse' instead of belief propagation\n");
  fprintf(fp, "  -D <#>   set X,Y,Z = D\n");
  fprintf(fp, "  -X <#>   set X\n");
  fprintf(fp, "  -Y <#>   set Y\n");
  fprintf(fp, "  -Z <#>   set Z\n");
  fprintf(fp, "  -T <#>   run test number\n");
  fprintf(fp, "  -S <#>   seed\n");
  fprintf(fp, "  -d       debug print\n");

  fprintf(fp, "  -V <#>   set verbosity level (default 0)\n");
  fprintf(fp, "  -e <#>   set convergence epsilon\n");
  fprintf(fp, "  -z <#>   set zero epsilon\n");
  fprintf(fp, "  -I <#>   set max step iteration\n");

  fprintf(fp, "  -v       show version\n");
  fprintf(fp, "  -h       help (this screen)\n");
  fprintf(fp, "\n");
}

void show_version(FILE *fp) {
  fprintf(fp, "bp version: %s\n", BELIEF_PROPAGATION_VERSION);
}

int main(int argc, char **argv) {
  int i, j, k, idx, ret;
  char ch;
  char* optarg;

  char *name_fn = NULL, *rule_fn = NULL, *constraint_fn = NULL;
  std::string name_fn_str, rule_fn_str, constraint_fn_str;

  int test_num = -1;
  int X=0, Y=0, Z=0, D=0;

  int wfc_flag = 0;
  int raycast = 0;
  int debug_print = 0;
  int seed = 0;

  int iresx=0, iresy=0;
  Vector3DI vres (X, Y, Z);
  Camera3D cam;
  const char VOL=0;

  std::string base_png = "out";
  char imgfile[512] = {0};

  float eps_zero = -1.0, eps_converge = -1.0;
  int max_iter = -1;

  std::vector< std::vector< int32_t > > constraint_list;

  BeliefPropagation bpc;

  int arg=1;

  //while ( handle_args ( arg, argc, argv, ch, optarg ) ) {
  while ((ch=pd_getopt(argc, argv, "hvdVre:z:I:N:R:C:T:WD:X:Y:Z:S:")) != EOF) {
    switch (ch) {
      case 'h':
        show_usage(stdout);
        exit(0);
        break;
      case 'v':
        show_version(stdout);
        exit(0);
        break;
      case 'd':
        debug_print = 1;
        break;
      case 'V':
        bpc.m_verbose = atoi(optarg);
        break;
      case 'r':
        raycast = 1;
        iresx = atoi(optarg);
        iresy = iresx;
        printf ("raycast enabled\n");
        break;


      case 'e':
        eps_converge = atof(optarg);
        if (eps_converge > 0.0) {
          bpc.m_eps_converge = eps_converge;
        }
        break;
      case 'z':
        eps_zero = atof(optarg);
        if (eps_zero > 0.0) {
          bpc.m_eps_zero = eps_zero;
        }
        break;
      case 'I':
        max_iter = atoi(optarg);
        if (max_iter > 0) {
          bpc.m_max_iteration = (int64_t)max_iter;
        }
        break;

      case 'N':
        name_fn = strdup(optarg);
        break;
      case 'R':
        rule_fn = strdup(optarg);
        break;
      case 'C':
        constraint_fn = strdup(optarg);
        break;

      case 'S':
        seed = atoi(optarg);
        bpc.m_seed = seed;
        break;

      case 'T':
        test_num = atoi(optarg);
        break;

      case 'D':
        D = atoi(optarg);
        break;
      case 'X':
        X = atoi(optarg);
        break;
      case 'Y':
        Y = atoi(optarg);
        break;
      case 'Z':
        Z = atoi(optarg);
        break;

      case 'W':
        wfc_flag = 1;
        break;

      default:
        show_usage(stderr);
        exit(-1);
    }
  }

 if ((!name_fn) || (!rule_fn)) {
    printf("\nprovide name file and rule file CSV\n\n");
    show_usage(stderr);
    exit(-1);
  }

  if (D>0) {
    X = D;
    Y = D;
    Z = D;
  }

  if ((X<=0) || (Y<=0) || (Z<=0)) {
    printf("dimensions must all be >0 (%i,%i,%i)\n", X,Y,Z);
    show_usage(stderr);
    exit(-1);
  }

  name_fn_str = name_fn;
  rule_fn_str = rule_fn;
  if (constraint_fn) {
    constraint_fn_str = constraint_fn;
    _read_constraint_csv(constraint_fn_str, constraint_list);
    printf ( "reading constraints file. %s, %d\n", constraint_fn_str.c_str(), (int) constraint_list.size() );
  }

  printf ( "bpc init csv.\n" );
  ret = bpc.init_CSV(X,Y,Z,name_fn_str, rule_fn_str);
  if (ret<0) {
    printf("error loading CSV\n");
    exit(-1);
  }

  if (constraint_fn) {
    printf ( "filter constraints.\n" );
    bpc.filter_constraint(constraint_list);
  }

  if (debug_print) {
    bpc.debugPrint();
    exit(0);
  }

  if (test_num >= 0) {
    run_test(test_num);
    exit(0);
  }

  // prepare raycast [optional]
  //
  if (raycast) {
    printf ( "preparing raycast.\n" );
    alloc_img (iresx, iresy);
    alloc_volume (VOL, vres, 4);
    cam.setOrbit ( 30, 20, 0, vres/2.0f, 50, 1 );
    printf ( "prepare raycast done. vol: %d,%d,%d  img: %d,%d\n", vres.x, vres.y, vres.z, iresx, iresy );
  }

  if (wfc_flag) {

    printf ( "wfc realize.\n" );
    ret = bpc.wfc();
    printf("# wfc got: %i\n", ret);
    bpc.debugPrint();

  }
  else {

    printf ( "bpc realize.\n" );
    ret = bpc.start();

    for (int64_t it=0; it < bpc.m_num_verts; it++) {
      ret = bpc.single_realize(it);
      if (ret<=0) { break; }

      if ( raycast )  {
        visualize_belief ( bpc, BUF_BELIEF, VOL, vres );
        raycast_cpu ( vres, &cam, VOL, m_img, iresx, iresy, Vector3DF(0,0,0), Vector3DF(vres) );
        snprintf ( imgfile, 511, "%s%04d.png", base_png.c_str(), (int) it );
        printf ( "  output: %s\n", imgfile );
        save_png ( imgfile, m_img, iresx, iresy, 3 );
      }
    }

    printf("# bp realize got: %i\n", ret);

    printf("####################### DEBUG PRINT\n" );
    bpc.debugPrint();

  }

  if (name_fn) { free(name_fn); }
  if (rule_fn) { free(rule_fn); }
  if (constraint_fn) { free(constraint_fn); }

  return 0;
}
