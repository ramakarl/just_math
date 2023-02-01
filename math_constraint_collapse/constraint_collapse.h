//--------------------------------------------------------------------------------
// JUST MATH:
// Constraint Collapse
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

#ifndef DEF_CONSTRAINT_COLLAPSE
#define DEF_CONSTRAINT_COLLAPSE

#include <algorithm>
#include "mersenne.h"
#include "dataptr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <vector>
#include <string>

#define CONSTRAINT_COLLAPSE_VERSION "0.0.1"

#define BUF_T           0    // tile vector
#define BUF_C           1    // constraint vector
#define BUF_R           2    // resolved vector
#define BUF_F           3    // rule list

class ConstraintCollapse {
public:
  ConstraintCollapse() {
    m_seed = 117;    
  };

  int init (int, int, int, std::string& rule_fn, std::string& name_fn);
  void start ();
  void single_step ();
  void randomize ();
  void decimate ();
  void write_admissible ();
  int check_constraints ();
  int check_constraints ( int64_t p );
  void fix_constraints (bool stuck);
  void check_r ();
  
  int GetVertexConstraints ( int64_t p );
  int CountBits ( int mask );
  void GetMaxResolved ( int64_t p, int prev, int next,  float& r_fixed, float& r_max);

  void Restart();
  void AllocBuf (int id, int cnt);              
  void AllocF (int id, int nbrs, int vals);
  void ZeroBuf (int id);
 
  int64_t    getNeighbor(uint64_t j, int nbr);        // 3D spatial neighbor function
  Vector3DI  getVertexPos(int64_t j);
  int64_t    getVertex(int x, int y, int z);
  int64_t    getFace (int64_t a, int nbr );
  Vector4DF  getSample ( int buf, int64_t v );  // for visualization
  inline int getNumNeighbors(int j)  {return 6;}     
  inline int getNumValues()         {return m_num_values;}
  inline int getNumVerts()          {return m_num_verts;}

  // buffer access
  inline float getVal(int id, int a) {return *(float*) m_buf[id].getPtr (a);}
  inline void  SetVal(int id, int a, float val)  {*(float*) m_buf[id].getPtr(a) = val;}
  inline float getValF(int id, int a, int b, int n)      { return *(float*) m_buf[id].getPtr ( (b*m_num_values + a)*6 + n ); }  // belief mapping (f), BxB
  inline void  SetValF(int id, int a, int b, int n, float val ) { *(float*) m_buf[id].getPtr ( (b*m_num_values + a)*6 + n ) = val; }

  // read helpers
  int read_F_CSV(std::string& rule_fn, std::string& name_fn );
  int read_constraints (std::string &fn );
  int filter_constraint(std::vector< std::vector< int32_t > > &constraint_list);

  uint64_t  m_num_verts;    // Xi = 0..X (graph domain)
  uint64_t  m_num_face;
  uint64_t  m_num_values;   // B = 0..Bm-1 (value domain)  
  Vector3DI m_res;          // volume res

  DataPtr  m_buf[8];      // data buffers (CPU & GPU)  

  std::vector< std::string > m_tile_name;   // tile names
  std::vector< std::vector< int32_t > > m_admissible;
  int m_dir_inv[6];

  bool      m_run_cuda=0;
  int       m_seed;
  Mersenne  m_rand;
  int       m_flip;
  float     m_resolve;
  int       m_cnt;
  int       m_stuck_cnt;
  int       m_step;
  float     m_temp;
  
};

int _read_line(FILE *fp, std::string &line);
int _read_name_csv(std::string &fn, std::vector<std::string> &name);
int _read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule);


#endif
