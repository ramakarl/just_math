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

// todo:
//  - writeboundary should be tested, init BUF_MU/BUF_MU_NXT with 0 values, write boundary, confirm they're what you think they are
//  - RealizeStep and RealizeRun should be tested

#include <algorithm>
#include "mersenne.h"
#include "dataptr.h"

#include "belief_propagation.h"
#include "main_belief_propagation.h"
#include "bp_helper.h"


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
int test0(BeliefPropagation &_bp) {

  int ret;

  BeliefPropagation bp;

  ret = bp_init_CSV ( bp, 4, 3, 1, _bp.op.name_fn, _bp.op.rule_fn);
  if (ret<0) { return ret; }

  bp.debugPrint();
  return 0;
}

// test filterDiscard
//
int test1(BeliefPropagation &_bp) {
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

  ret = bp_init_CSV ( bp, 4, 3, 1, _bp.op.name_fn, _bp.op.rule_fn);

  if (ret<0) { return ret; }

  bp.filterDiscard(6, discard_list);
  bp.debugPrint();
  return 0;
}

// test filterKeep
//
int test2(BeliefPropagation &_bp) {
  int ret;
  std::vector<int32_t> keep_list;

  keep_list.push_back(1);
  keep_list.push_back(2);
  keep_list.push_back(3);
  keep_list.push_back(4);
  keep_list.push_back(5);
  keep_list.push_back(6);

  BeliefPropagation bp;

  ret = bp_init_CSV ( bp, 4, 3, 1, _bp.op.name_fn, _bp.op.rule_fn);

  if (ret<0) { return ret; }


  bp.filterKeep(6, keep_list);
  bp.debugPrint();
  return 0;
}

int test3(BeliefPropagation &_bp) {
  int ret;
  std::vector<int32_t> keep_list;

  BeliefPropagation bp;

  ret = bp_init_CSV ( bp, 4, 3, 1, _bp.op.name_fn, _bp.op.rule_fn);

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

int test4_(BeliefPropagation &_bp) {
  int ret;

  float maxdiff;
  std::vector<int32_t> keep_list;

  BeliefPropagation bp;

  ret = bp_init_CSV ( bp, 4, 3, 1, _bp.op.name_fn, _bp.op.rule_fn);

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

int test4(BeliefPropagation &_bp) {
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

  ret = bp_init_CSV ( bp, 4, 3, 1, _bp.op.name_fn, _bp.op.rule_fn);

  if (ret<0) { return ret; }

  bp.op.step_rate = 1.0;

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

  bp.WriteBoundaryMUbuf(BUF_MU);
  bp.BeliefProp();
  bp.NormalizeMU(BUF_MU_NXT);
  bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );
  maxdiff = bp.MaxDiffMU();
  bp.UpdateMU();

  printf("[0] got diff: %f\n", maxdiff);

  printf("\n\n---\niter %i:\n", 0);
  bp.debugPrint();
  printf("---\n\n");



  bp.WriteBoundaryMUbuf(BUF_MU);
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

  bp.WriteBoundaryMUbuf(BUF_MU);
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

  bp.WriteBoundaryMUbuf(BUF_MU);
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

  bp.WriteBoundaryMUbuf(BUF_MU);
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

  bp.WriteBoundaryMUbuf(BUF_MU);
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
int test5(BeliefPropagation &_bp) {
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

  ret = bp_init_CSV ( bp, 3, 3, 1, _bp.op.name_fn, _bp.op.rule_fn);
  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;

  bp.start();

  // for some reason, this small size has problems
  // if the rate isn't set to maximum (from default
  // .98)
  //
  bp.op.step_rate = 1.0;


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

  bp.RealizePre();
  for (iter=0; iter<max_iter; iter++) {
    ret = bp.RealizeStep();
    if (ret==0) { break; }

    /*
    bp.NormalizeMU();

    printf("---\nBEFORE:\n");
    bp.debugPrint();

    bp.WriteBoundaryMUbuf(BUF_MU);
    bp.BeliefProp();
    bp.NormalizeMU(BUF_MU_NXT);
    bp.TransferBoundaryMU( BUF_MU, BUF_MU_NXT );

    maxdiff = bp.MaxDiffMU();
    bp.UpdateMU();

    if (fabs(maxdiff) < _eps) { break; }
    */
  }
  bp.NormalizeMU(BUF_MU_NXT);

  printf("AFTER: %i\n", iter);
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


  ret = bp_init_CSV ( bp, 3, 3, 1, bp.op.name_fn, bp.op.rule_fn);
  if (ret<0) { return ret; }

  bp.start();

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

  //bp.NormalizeMU();


  bp.RealizePre();
  for (iter=0; iter<max_iter; iter++) {

    ret = bp.RealizeStep();
    if (ret == 0) { break; }
    //maxdiff = bp.step(1);

    /*
    bp.NormalizeMU();

    printf("---\nBEFORE:\n");
    bp.debugPrint();

    bp.BeliefProp();
    bp.NormalizeMU(BUF_MU_NXT);
    maxdiff = bp.MaxDiffMU();
    bp.UpdateMU();
    */

    //if (fabs(maxdiff) < _eps) { break; }
  }
  //bp.RealizePost();

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

  ret = bp_init_CSV ( bp, 2, 2, 1,  bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  bp.cellFillVisited (0, bp.m_note_plane);
  bp.unfillVisited (bp.m_note_plane);

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

  ret = bp_init_CSV ( bp, 3, 3, 1,  bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  bp.cellFillVisited (0, bp.m_note_plane);
  bp.unfillVisited (bp.m_note_plane);

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

  ret = bp_init_CSV ( bp, 3, 3, 1, bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  bp.cellFillVisited (0, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  bp.cellFillVisited (4, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,0), keep_list);

  bp.cellFillVisited (8, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

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


  ret = bp_init_CSV ( bp, 3, 3, 2, bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  bp.cellFillVisited(0, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  bp.filterKeep( bp.getVertex(2,0,0), keep_list);

  bp.cellFillVisited(0, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  bp.cellFillVisited(4, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  bp.filterKeep( bp.getVertex(1,1,1), keep_list);

  bp.cellFillVisited(4, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,2,1), keep_list);

  bp.cellFillVisited(8, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(2,2,1), keep_list);

  bp.cellFillVisited(8, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

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

  ret = bp_init_CSV ( bp, 2, 2, 1, bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }


  //--

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)"r003") );
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"^012") );
  bp.filterKeep( bp.getVertex(0,0,0), keep_list);

  bp.cellFillVisited(0, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"r002") );
  keep_list.push_back( bp.tileName2ID((char *)"^011") );
  bp.filterKeep( bp.getVertex(1,0,0), keep_list);

  bp.cellFillVisited(1, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"^013") );
  keep_list.push_back( bp.tileName2ID((char *)"r000") );
  bp.filterKeep( bp.getVertex(0,1,0), keep_list);

  bp.cellFillVisited(2, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

  keep_list.clear();
  keep_list.push_back( bp.tileName2ID((char *)".000") );
  keep_list.push_back( bp.tileName2ID((char *)"^010") );
  keep_list.push_back( bp.tileName2ID((char *)"r001") );
  bp.filterKeep( bp.getVertex(1,1,0), keep_list);

  bp.cellFillVisited(3, bp.m_note_plane);
  bp.unfillVisited(bp.m_note_plane);

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

  ret = bp_init_CSV ( bp, 3, 3, 1, bp.op.name_fn, bp.op.rule_fn);

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

  bp.op.seed = 18;

  ret = bp_init_CSV ( bp, 2, 2, 1, bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }


  //bp.debugPrintC();
  //bp.debugPrintS();

  // ret = bp.realize();

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

  bp.op.seed = 18;


  ret = bp_init_CSV ( bp, 3, 3, 1, bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }


  //bp.debugPrintC();
  //bp.debugPrintS();

  // ret = bp.realize();

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


  ret = bp_init_CSV ( bp, x, y, z, bp.op.name_fn, bp.op.rule_fn);

  if (ret<0) { return ret; }

  // ret = bp.realize();

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);

  bp.debugPrint();
  return 0;
}


int test_step0(BeliefPropagation &_bp) {

  int ret;
  float maxdiff, _eps = (1.0/(1024.0));
  std::vector<int32_t> keep_list;

  BeliefPropagation bp;

  ret = bp_init_CSV ( bp, 4, 4, 4, _bp.op.name_fn, _bp.op.rule_fn );

  if (ret<0) { return ret; }

  ret = bp.start();
  if (ret<0) { return ret; }

  int n_it = bp.m_num_verts * bp.m_num_values;

  for (int it=0; it<n_it; it++) {

    ret = bp.RealizePre();
    if (ret < 0) { break; }

    ret = bp.RealizeIter();
    if (ret < 0) { break; }

    ret = bp.RealizePost();
    if (ret < 0) { break; }

     //ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    //if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", bp.op.X, bp.op.Y, bp.op.Z, ret);
  bp.debugPrint();

  return ret;
}

// test run until converged
//
int test_step1(BeliefPropagation &_bp) {
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

  ret = bp_init_CSV ( bp, x, y, z, _bp.op.name_fn, _bp.op.rule_fn);

  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;

  bp.op.verbose = 3;

  bp.op.seed = 0;

  ret = bp.start();
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

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {

    ret = bp.RealizePre();
    if (ret < 0) { break; }

    ret = bp.RealizeIter();
    if (ret < 0) { break; }

    ret = bp.RealizePost();
    if (ret <= 0) { break; }

    // ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    //if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  bp.debugPrint();

  return ret;
}


// test run until converged
// with SVD optimization turned on
//
int test_step2(BeliefPropagation &_bp) {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  x = 3;
  y = 3;
  z = 1;

  int iter, max_iter=100;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.op.use_svd = 1;

  ret = bp_init_CSV( bp, x,y,z, _bp.op.name_fn, _bp.op.rule_fn);

  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;

  bp.op.verbose = 3;

  //---
  bp.op.seed = 0;

  ret = bp.start();
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

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {

    ret = bp.RealizePre();
    if (ret < 0) { break; }

    ret = bp.RealizeIter();
    if (ret < 0) { break; }

    ret = bp.RealizePost();
    if (ret <= 0) { break; }


    // ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    //if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  bp.debugPrint();

  return ret;
}


// test checkerboard on simple example
//
int test_step3(BeliefPropagation &_bp) {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  x = 3;
  y = 3;
  z = 1;

  int iter, max_iter=100;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.op.use_svd = 0;
  bp.op.use_checkerboard = 1;

  ret = bp_init_CSV( bp, x,y,z, _bp.op.name_fn, _bp.op.rule_fn );
  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;

  bp.op.verbose = 3;

  //--

  bp.op.seed = 0;

  ret = bp.start();
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

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {

    ret = bp.RealizePre();
    if (ret < 0) { break; }

    ret = bp.RealizeIter();
    if (ret < 0) { break; }

    ret = bp.RealizePost();
    if (ret <= 0) { break; }

    // ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    //if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  bp.debugPrint();

  return ret;
}



// test checkerboard on simple example
// with svd
//
int test_step4(BeliefPropagation &_bp) {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  x = 3;
  y = 3;
  z = 1;

  int iter, max_iter=100;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.op.use_svd = 1;
  bp.op.use_checkerboard = 1;

  ret = bp_init_CSV( bp, x,y,z, _bp.op.name_fn, _bp.op.rule_fn );
  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;

  bp.op.verbose = 3;


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
  bp.op.seed = 0;

  ret = bp.start();
  if (ret<0) { return ret; }

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {
    // ret = bp.single_realize_max_belief_cb(max_iter, NULL);
    if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  bp.debugPrint();

  return ret;
}

// populate BUF_MU and BUF_MU_NXT
// with random values, add the differences
// to the indexHeap structures/buffers (through
// indexHeap_init) and then run consistency
// checks on the heap.
//
int test_residual0() {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  int64_t idx, n, _cell;
  int32_t _tile, _idir;
  float _a, _b;

  BeliefPropagation bp;

  ret = bp_init_CSV( bp, 3, 3, 1, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  n = bp.m_num_values * bp.m_num_verts * 6;

  for (idx=0; idx<n; idx++) {
    bp.getMuPos( idx, &_idir, &_cell, &_tile );

    _a = bp.m_rand.randF();
    _b = bp.m_rand.randF();

    bp.SetValF( BUF_MU,     _a, _idir, _tile, _cell );
    bp.SetValF( BUF_MU_NXT, _b, _idir, _tile, _cell );
  }

  bp.indexHeap_init();

  //bp.indexHeap_debug_print();

  ret = bp.indexHeap_consistency();
  printf("indexHeap_consistency got: %i\n", (int)ret);

  return ret;
}

// populate BUF_MU and BUF_MU_NXT
// with random values, add the differences
// to the indexHeap structures/buffers (through
// indexHeap_init) and then run consistency
// checks on the heap.
//
int test_residual1() {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  int64_t idx, n, _cell;
  int32_t _tile, _idir;
  float _a, _b;

  int _buf;

  BeliefPropagation bp;

  ret = bp_init_CSV( bp, 3, 3, 1, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  n = bp.m_num_values * bp.m_num_verts * 6;

  for (idx=0; idx<n; idx++) {
    bp.getMuPos( idx, &_idir, &_cell, &_tile );

    _a = bp.m_rand.randF();
    _b = bp.m_rand.randF();

    bp.SetValF( BUF_MU,     _a, _idir, _tile, _cell );
    bp.SetValF( BUF_MU_NXT, _b, _idir, _tile, _cell );
  }

  bp.indexHeap_init();

  n = bp.op.index_heap_size;

  n_it = n*10;
  for (it=0; it<n_it; it++) {
    do {
      idx = (int64_t)(bp.m_rand.randF() * (float)n);
    } while (idx==n);
    _a = bp.m_rand.randF();
    bp.indexHeap_update( idx, _a );

    ret = bp.indexHeap_consistency();

    if (ret < 0) {
      printf("[%i/%i] indexHeap_consistency got: %i\n", (int)it, (int)n_it, (int)ret);
      return ret;
    }

  }

  ret = bp.indexHeap_consistency();
  printf("indexHeap_consistency got: %i\n", (int)ret);

  return ret;
}

int test_residual2() {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  int64_t idx, n, _cell, mu_idx;
  int32_t _tile, _idir;
  float _a, _b;

  int _buf;

  BeliefPropagation bp;

  ret = bp_init_CSV( bp, 3, 3, 1, bp.op.name_fn, bp.op.rule_fn  );
  if (ret<0) { return ret; }

  n = bp.m_num_values * bp.m_num_verts * 6;

  for (idx=0; idx<n; idx++) {
    bp.getMuPos( idx, &_idir, &_cell, &_tile );

    _a = bp.m_rand.randF();
    _b = bp.m_rand.randF();

    bp.SetValF( BUF_MU,     _a, _idir, _tile, _cell );
    bp.SetValF( BUF_MU_NXT, _b, _idir, _tile, _cell );
  }

  bp.indexHeap_init();

  n = bp.op.index_heap_size;

  n_it = n*10;
  for (it=0; it<n_it; it++) {
    do {
      mu_idx = (int64_t)(bp.m_rand.randF() * (float)n);
    } while (mu_idx==n);
    _a = bp.m_rand.randF();
    _buf = ((bp.m_rand.randF() < 0.5) ? BUF_MU : BUF_MU_NXT);

    ret = bp.getMuPos( mu_idx, &_idir, &_cell, &_tile );
    if (ret < 0) {
      printf("ERROR!\n");
      return ret;
    }

    bp.SetValF( _buf, _a, _idir, _tile, _cell );

    _a = bp.getValF( BUF_MU,     _idir, _tile, _cell );
    _b = bp.getValF( BUF_MU_NXT, _idir, _tile, _cell );

    bp.indexHeap_update( mu_idx, fabs(_a-_b) );

    ret = bp.indexHeap_consistency();
    if (ret < 0) {
      printf("[%i/%i] indexHeap_consistency got: %i\n", (int)it, (int)n_it, (int)ret);
      return ret;
    }

  }

  //bp.indexHeap_debug_print();

  ret = bp.indexHeap_consistency();
  printf("indexHeap_consistency got: %i\n", (int)ret);

  return ret;
}

int test_residual3() {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  int64_t idx, n, _cell, mu_idx;
  int32_t _tile, _idir;
  float _a, _b;

  int _buf;

  BeliefPropagation bp;

  ret = bp_init_CSV( bp, 3, 2, 1, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  n = bp.m_num_values * bp.m_num_verts * 6;

  for (idx=0; idx<n; idx++) {
    bp.getMuPos( idx, &_idir, &_cell, &_tile );

    _a = bp.m_rand.randF();
    _b = bp.m_rand.randF();

    bp.SetValF( BUF_MU,     _a, _idir, _tile, _cell );
    bp.SetValF( BUF_MU_NXT, _b, _idir, _tile, _cell );
  }

  bp.indexHeap_init();

  n = bp.op.index_heap_size;

  n_it = n*n*10;
  for (it=0; it<n_it; it++) {
    do {
      mu_idx = (int64_t)(bp.m_rand.randF() * (float)n);
    } while (mu_idx==n);
    _a = bp.m_rand.randF();
    _buf = ((bp.m_rand.randF() < 0.5) ? BUF_MU : BUF_MU_NXT);

    ret = bp.getMuPos( mu_idx, &_idir, &_cell, &_tile );
    if (ret < 0) {
      printf("ERROR!\n");
      return ret;
    }

    if ((_idir < 0) ||
        (_idir >= 6) ||
        (_cell < 0) ||
        (_cell >= bp.m_num_verts) ||
        (_tile < 0) ||
        (_tile >= bp.m_num_values)) {
      printf("!!!!! mu_idx:%i -> (%i,%i,%i)\n",
          (int)mu_idx,
          (int)_idir, (int)_cell, (int)_tile);
      return -1;
    }


    bp.SetValF( _buf, _a, _idir, _tile, _cell );

    _a = bp.getValF( BUF_MU,     _idir, _tile, _cell );
    _b = bp.getValF( BUF_MU_NXT, _idir, _tile, _cell );

    bp.indexHeap_update_mu_idx( mu_idx, fabs(_a-_b) );

    if ((it%1000)==0) {

      ret = bp.indexHeap_consistency();
      if (ret < 0) {
        printf("[%i/%i] indexHeap_consistency got: %i\n", (int)it, (int)n_it, (int)ret);
        return ret;
      }

    }

  }

  //bp.indexHeap_debug_print();

  ret = bp.indexHeap_consistency();
  printf("indexHeap_consistency got: %i\n", (int)ret);

  return ret;
}

int test_residual4() {
  int ret;
  int64_t it, n_it;
  int x, y, z;

  int64_t idx, n, _cell, mu_idx;
  int32_t _tile, _idir;
  float _a, _b;

  int _buf;

  BeliefPropagation bp;

  ret = bp_init_CSV( bp, 3, 2, 1, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  n = bp.m_num_values * bp.m_num_verts * 6;

  for (idx=0; idx<n; idx++) {
    bp.getMuPos( idx, &_idir, &_cell, &_tile );

    _a = bp.m_rand.randF();
    _b = bp.m_rand.randF();

    bp.SetValF( BUF_MU,     _a, _idir, _tile, _cell );
    bp.SetValF( BUF_MU_NXT, _b, _idir, _tile, _cell );
  }

  bp.indexHeap_init();

  n = bp.op.index_heap_size;

  n_it = n*n*10;
  for (it=0; it<n_it; it++) {
    do {
      mu_idx = (int64_t)(bp.m_rand.randF() * (float)n);
    } while (mu_idx==n);
    _a = bp.m_rand.randF();
    _buf = ((bp.m_rand.randF() < 0.5) ? BUF_MU : BUF_MU_NXT);

    ret = bp.getMuPos( mu_idx, &_idir, &_cell, &_tile );
    if (ret < 0) {
      printf("ERROR!\n");
      return ret;
    }

    if ((_idir < 0) ||
        (_idir >= 6) ||
        (_cell < 0) ||
        (_cell >= bp.m_num_verts) ||
        (_tile < 0) ||
        (_tile >= bp.m_num_values)) {
      printf("!!!!! mu_idx:%i -> (%i,%i,%i)\n",
          (int)mu_idx,
          (int)_idir, (int)_cell, (int)_tile);
      return -1;
    }


    bp.SetValF( _buf, _a, _idir, _tile, _cell );

    _a = bp.getValF( BUF_MU,     _idir, _tile, _cell );
    _b = bp.getValF( BUF_MU_NXT, _idir, _tile, _cell );

    bp.indexHeap_update( mu_idx, fabs(_a-_b) );

    if ((it%1000)==0) {

      ret = bp.indexHeap_consistency();
      if (ret < 0) {
        printf("[%i/%i] indexHeap_consistency got: %i\n", (int)it, (int)n_it, (int)ret);
        return ret;
      }

    }

  }

  //bp.indexHeap_debug_print();

  ret = bp.indexHeap_consistency();
  printf("indexHeap_consistency got: %i\n", (int)ret);

  return ret;
}

int test_residual5() {
  int ret, _buf;
  int64_t it, n_it;
  int x, y, z;

  int64_t idx, n, _cell, mu_idx, heap_idx;
  int32_t _tile, _idir;
  float _a, _b, mu_max_diff, f, mu_buf_max;

  int64_t mu_max_idir,
          mu_max_cell,
          mu_max_tile,
          ih_idx_max;

  float _eps;
  int mu_pos_found=0;

  BeliefPropagation bp;

  _eps = bp.op.eps_zero;

  ret = bp_init_CSV( bp, 3, 2, 1, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  n = bp.m_num_values * bp.m_num_verts * 6;

  for (idx=0; idx<n; idx++) {
    bp.getMuPos( idx, &_idir, &_cell, &_tile );

    _a = bp.m_rand.randF();
    _b = bp.m_rand.randF();

    bp.SetValF( BUF_MU,     _a, _idir, _tile, _cell );
    bp.SetValF( BUF_MU_NXT, _b, _idir, _tile, _cell );
  }

  bp.indexHeap_init();

  n = bp.op.index_heap_size;

  n_it = n*n*10;
  for (it=0; it<n_it; it++) {
    do {
      mu_idx = (int64_t)(bp.m_rand.randF() * (float)n);
    } while (mu_idx==n);
    _a = bp.m_rand.randF();
    _buf = ((bp.m_rand.randF() < 0.5) ? BUF_MU : BUF_MU_NXT);

    ret = bp.getMuPos( mu_idx, &_idir, &_cell, &_tile );
    if (ret < 0) {
      printf("ERROR!\n");
      return ret;
    }

    if ((_idir < 0) ||
        (_idir >= 6) ||
        (_cell < 0) ||
        (_cell >= bp.m_num_verts) ||
        (_tile < 0) ||
        (_tile >= bp.m_num_values)) {
      printf("!!!!! mu_idx:%i -> (%i,%i,%i)\n",
          (int)mu_idx,
          (int)_idir, (int)_cell, (int)_tile);
      return -1;
    }

    bp.SetValF( _buf, _a, _idir, _tile, _cell );

    _a = bp.getValF( BUF_MU,     _idir, _tile, _cell );
    _b = bp.getValF( BUF_MU_NXT, _idir, _tile, _cell );

    bp.indexHeap_update_mu_idx( mu_idx, fabs(_a-_b) );

  }


  mu_buf_max = -1.0;
  for (_idir=0; _idir<6; _idir++) {
    for (_cell=0; _cell<bp.m_num_verts; _cell++) {
      for (_tile=0; _tile<bp.m_num_values; _tile++) {

        _a = bp.getValF( BUF_MU,     _idir, _tile, _cell );
        _b = bp.getValF( BUF_MU_NXT, _idir, _tile, _cell );

        f = fabs(_a-_b);

        if (mu_buf_max < f) {
          mu_buf_max = f;
        }

      }
    }
  }

  bp.indexHeap_peek_mu_pos( &_idir, &_cell, &_tile, &_a);
  mu_max_idir = _idir;
  mu_max_cell = _cell;
  mu_max_tile = _tile;
  mu_max_diff = _a;

  //printf("mu_max (idir:%i,cell:%i,tile:%i,val:%f)\n",
  //    (int)mu_max_idir, (int)mu_max_cell, (int)mu_max_tile, (float)mu_max_diff);

  if ( fabs(mu_max_diff - mu_buf_max) > _eps ) {
    printf("observed maximum in MU/MU_NXT buf (%f) disagrees with indexHeap maximum (%f)\n",
        (float)mu_buf_max, (float)mu_max_diff);

    ret = bp.indexHeap_consistency();
    printf("indexHeap_consistency got: %i\n", (int)ret);


    return -103;
  }

  //if ( fabs(mu_max_diff - mu_buf_max) < _eps) { printf("yes, they're close\n"); }

  mu_pos_found = 0;
  for (_idir=0; _idir<6; _idir++) {
    for (_cell=0; _cell<bp.m_num_verts; _cell++) {
      for (_tile=0; _tile<bp.m_num_values; _tile++) {
        _a = bp.getValF( BUF_MU,     _idir, _tile, _cell );
        _b = bp.getValF( BUF_MU_NXT, _idir, _tile, _cell );

        f = fabs(_a-_b);

        if ( (fabs(mu_max_diff - f) <= _eps) ||
             (fabs(f - mu_max_diff) <= _eps) ) {

          //printf(">>max<< %i %i %i (%f,%f,%f)\n", (int)_idir, (int)_cell, (int)_tile, f, (float)mu_max_diff, (float)fabs(mu_max_diff-f));

          if ((_idir == mu_max_idir) &&
              (_cell == mu_max_cell) &&
              (_tile == mu_max_tile)) {

            //printf("!!found!! %i %i %i\n", (int)_idir, (int)_cell, (int)_tile);

            mu_pos_found = 1;
            break;

          }

        }

      }
      if (mu_pos_found) { break; }
    }
    if (mu_pos_found) { break; }
  }

  if (mu_pos_found==0) {
    printf("did not retrieve maximum mu position. Expected (idir:%i,cell:%i,tile:%i) (%f), observied max: %f\n",
      (int)mu_max_idir, (int)mu_max_cell, (int)mu_max_tile, (float)mu_max_diff, (float)mu_buf_max);

    ret = bp.indexHeap_consistency();
    printf("indexHeap_consistency got: %i\n", (int)ret);

    return -101;
  }

  //bp.indexHeap_debug_print();

  ret = bp.indexHeap_consistency();
  printf("indexHeap_consistency got: %i\n", (int)ret);

  return ret;
}

// test residual bp run until converged
//
int test_residual6() {
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

  int iter, max_iter=100000;
  float maxdiff;
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.op.use_svd = 0;

  ret = bp_init_CSV( bp, x,y,z, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;
  bp.op.step_rate = 1.0;

  bp.op.verbose = 3;


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
  bp.op.seed = 0;

  ret = bp.start();
  if (ret<0) { return ret; }


  //ret = bp.single_realize_residue_cb(max_iter, NULL);

  int32_t idir, tile;
  int64_t cell, mu_idx;
  float f_residue, d;
  int _verbose = 1;

  int32_t c_0, c_1;

  // after we've propagated constraints, BUF_MU
  // needs to be renormalized
  //
  bp.WriteBoundaryMUbuf(BUF_MU);
  bp.WriteBoundaryMUbuf(BUF_MU_NXT);
  bp.NormalizeMU();
  bp.NormalizeMU(BUF_MU_NXT);
  d = bp.step(1);
  d = bp.step(0);
  bp.indexHeap_init();

  if (_verbose > 0) {
    printf("--MU>>>\n");
    bp.debugPrintMU();

    c_0 = bp.indexHeap_consistency();
    c_1 = bp.indexHeap_mu_consistency();

    printf("  consist:(%i,%i))\n", (int)c_0, (int)c_1);
    printf("--MU<<<\n");
  }

  // iterate bp until converged or max iteration count tripped
  //
  for (iter=1; iter<max_iter; iter++) {

    mu_idx = bp.indexHeap_peek_mu_pos( &idir, &cell, &tile, &f_residue);
    if (f_residue < bp.op.eps_converge) { break; }

    if (_verbose > 0) {
      printf("\n");
      printf("  [it:%i,step:%i] [idir:%i,cell:%i,tile:%i](mu_idx:%i) residue %f\n",
          (int)iter, (int)max_iter,
          (int)idir, (int)cell, (int)tile, (int)mu_idx, (float)f_residue);
    }

    d = bp.step_residue( idir, cell, tile );

    if (_verbose > 0) {
      c_0 = bp.indexHeap_consistency();
      c_1 = bp.indexHeap_mu_consistency();

      printf("  consist:(%i,%i))\n", (int)c_0, (int)c_1);
    }


  }

  printf("---\n");

  //printf("(%i,%i,%i) got: %i\n", x, y, z, ret);
  //bp.debugPrint();
  bp.debugPrintMU();

  printf("---\n");

  return ret;
}


// test residual bp run until converged
//
int test_residual7(BeliefPropagation &_bp) {
  int ret;
  int64_t it, n_it;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  int iter, max_iter=1000;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.op.use_svd = 0;

  ret = bp_init_CSV( bp, 3, 3, 1, _bp.op.name_fn, _bp.op.rule_fn );
  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;
  bp.op.step_rate = 1.0;

  bp.op.verbose = 3;

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
  bp.op.seed = 0;

  ret = bp.start();
  if (ret<0) { return ret; }

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {
    // ret = bp.single_realize_residue_cb(max_iter, NULL);
    if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", bp.op.X, bp.op.Y, bp.op.Z, ret);
  bp.debugPrint();

  return ret;
}

// test residual bp run until converged
//
int test_residual8() {
  int ret;
  int64_t it, n_it;

  // expect:
  //
  // 0,1,0: 2/5 |000, 3/5 T003
  // 2,1,0: 2/5 |000, 3/5 T001
  // 1,2,0: 2/5 |001, 3/5 T000
  // 1,1,0: 1/5 all
  //

  int iter, max_iter=1000;
  float maxdiff, _eps = (1.0/(1024*1024));
  std::vector<int32_t> keep_list;
  BeliefPropagation bp;

  bp.op.use_svd = 1;

  ret = bp_init_CSV( bp, 3,3,1, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  bp.op.eps_converge = 1.0/1024.0;
  bp.op.step_rate = 1.0;

  bp.op.verbose = 3;


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
  bp.op.seed = 0;

  ret = bp.start();
  if (ret<0) { return ret; }

  n_it = bp.m_num_verts * bp.m_num_values;
  for (it=0; it<n_it; it++) {
    // ret = bp.single_realize_residue_cb(max_iter, NULL);
    if (ret<=0) { break; }
  }

  printf("(%i,%i,%i) got: %i\n", bp.op.X, bp.op.Y, bp.op.Z, ret);
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
  ret = bp_init_CSV( bp, x,y,z, bp.op.name_fn, bp.op.rule_fn );
  if (ret<0) { return ret; }

  ret = bp.wfc();

  printf("(%i,%i,%i) got: %i\n", x, y, z, ret);

  bp.debugPrint();
  return 0;
}

int run_test(BeliefPropagation &bp, int test_num) {

  switch(test_num) {
    case 0:
      test0(bp);
      break;
    case 1:
      test1(bp);
      break;
    case 2:
      test2(bp);
      break;
    case 3:
      test3(bp);
      break;
    case 4:
      test4(bp);
      break;
    case 5:
      test5(bp);
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
      test_step0(bp);
      break;

    case 17:
      test_step1(bp);
      break;

    case 18:
      test_step2(bp);
      break;

    case 19:
      test_step3(bp);
      break;

    case 20:
      test_step4(bp);
      break;

    case 21:
      test_residual0();
      break;
    case 22:
      test_residual1();
      break;
    case 23:
      test_residual2();
      break;
    case 24:
      test_residual3();
      break;
    case 25:
      test_residual4();
      break;
    case 26:
      test_residual5();
      break;
    case 27:
      test_residual6();
      break;
    case 28:
      test_residual7(bp);
      break;
    case 29:
      test_residual8();
      break;

    default:
      return -1;

  }

  return 0;
}

