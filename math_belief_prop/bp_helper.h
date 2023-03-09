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

#ifndef BP_HELPER
#define BP_HELPER

#include <vector>
#include <string>

#include "belief_propagation.h"
#include "main_belief_propagation.h"

#include "string_helper.h"


typedef struct constraint_op_type {
  char op;
  std::vector< int > dim_range;
  std::vector< int > tile_range;
} constraint_op_t;

int parse_range(std::vector<int> &range, std::string &s, std::vector<int> &dim);
int parse_bracket_range(std::vector<int> &range, std::string &s, std::vector<int> &dim);
int parse_constraint_dsl(std::vector< constraint_op_t > &op_list, std::string &s, std::vector< int > dim, std::vector< std::string > name);

int _read_line(FILE *fp, std::string &line);
int _read_name_csv(std::string &fn, std::vector<std::string> &name, std::vector<float> &weight);
int _read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule);

int init_CSV(
    BeliefPropagation &bp,
    int X, int Y, int Z,
    std::string &name_fn,
    std::string &rule_fn);

int _parse_constraint_dsl (std::vector< constraint_op_t > &op_list, std::string &s, std::vector< int > dim, std::vector< std::string > name);

int parse_frange(std::vector<float> &range, std::string &s);
int parse_range(std::vector<int> &range, std::string &s, std::vector<int> &dim);
int parse_bracket_range(std::vector<int> &range, std::string &s, std::vector<int> &dim);
int parse_constraint_dsl(std::vector< constraint_op_t > &op_list, std::string &s, std::vector< int > dim, std::vector< std::string > name);
void debug_constraint_op_list(std::vector< constraint_op_t > &op_list);
int constrain_bp(BeliefPropagation &bp, std::vector< constraint_op_t > &op_list);
void stl_print(FILE *, std::vector< float > &, float, float, float);
int write_bp_stl(opt_t &opt, BeliefPropagation &bp, std::vector< std::vector< float > > tri_lib);
int write_tiled_json(opt_t &opt, BeliefPropagation &bpc);
int grid_obj2stl_out(std::string ofn, BeliefPropagation &bp, std::vector< std::vector< float > > tri);
int load_obj2tri(std::string inputfile, std::vector< float > &tri);
//void stl_print(FILE *fp, std::vector< float > &tri, float dx=0.0, float dy=0.0, float dz=0.0);
void stl_print(FILE *fp, std::vector< float > &tri, float dx, float dy, float dz);
int load_obj_stl_lib(std::string fn, std::vector< std::vector< float > > &tris);

extern opt_t g_opt;

#endif

