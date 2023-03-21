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

#define CONSTRAINT_COLLAPSE_VERSION "0.0.2"

#define BUF_T           0    // selected tiles
#define BUF_C           1    // constraints
#define BUF_E           2    // vertex constraints (extended)
#define BUF_R           3    // tile confidence (R=resolved)
#define BUF_F           4    // rule list, 6xB
#define BUF_G           5    // tile weights, 1xB

#define BUF_MAX         6

typedef struct lsa_opt_type {
    
    int         seed;
    
    int         status;

    Vector3DI   res;            // volume res

    std::string name_fn,
                rule_fn,
                tilemap_fn,
                constraint_fn,
                tileset_fn;  

    int         tileset_width, tileset_height,
                tileset_stride_x, tileset_stride_y;

    float       decay;          // accepted neighbor decay rate
    float       border_r;       // resolved rate at border
    float       noise_r;        // noise for decrease in confidence
    float       noise_flip;     // flip noise

    int         cur_step;

    int         cur_run, 
                max_run;

    bool        use_cuda;

    int         m_step;    

    float       elapsed_time;
    float       success_time;
    float       fail_time;
    float       max_time;

    float       temperature;
    int         constrained_cnt;
    int         stuck_cnt;  

} lsa_opt_t;

typedef struct _lsa_expr_type {

    int         num_expr;
    int         num_run;

    Vector3DI   grid_min, grid_max;

} lsa_expr_t;


class ConstraintCollapse {
public:
  ConstraintCollapse() {    
      default_ops ();
  };

  void default_ops ();
  void reset_stats ();
  void reset ();
  int init (int, int, int, std::string& rule_fn, std::string& name_fn);  
  void start ();
  int step ();  
  void randomize ();
  void decimate ();
  void write_admissible ();
  int check_constraints ();
  int check_constraints ( int64_t p );
  void fix_constraints (bool stuck);
  void generate_noise ();
  void check_r ();
  void print_map();

  std::string getStatMsg ();
  std::string getStatCSV ();
  
  int GetVertexConstraints ( int64_t p );
  int CountBits ( int mask );
  void GetMaxResolved ( int64_t p, int prev, int next, float r_edge, float& r_fixed, float& r_max);
  void RandomizeVert ( Vector3DI ki );

  void Restart();
  
  void AllocBuf (int id, char dt, uint64_t resx=1, uint64_t resy=1, uint64_t resz=1 );  
  void ZeroBuf (int id);
 
  int64_t    getNeighbor(uint64_t j, int nbr);        // 3D spatial neighbor function
  Vector3DI  getVertexPos(int64_t j);
  int64_t    getVertex(int x, int y, int z);
  int64_t    getFace (int64_t a, int nbr );
  Vector4DF  getSample ( int buf, int64_t v );  // for visualization
  inline int getNumNeighbors()      {return (op.res.z==1) ? 4 : 6;}
  inline int getNumValues()         {return m_num_values;}
  inline int getNumVerts()          {return m_num_verts;}

  // buffer access
  inline int32_t getValI(int id, int x=0, int y=0, int z=0)            {return *(int32_t*) m_buf[id].getPtr (x, y, z);}
  inline int64_t getValL(int id, int x=0, int y=0, int z=0)            {return *(int64_t*) m_buf[id].getPtr (x, y, z);}
  inline float   getValF(int id, int x=0, int y=0, int z=0)            {return *(float*) m_buf[id].getPtr (x, y, z);}

  inline void   SetValI(int id, int32_t val, int x, int y=0, int z=0)     {*(int32_t*) m_buf[id].getPtr(x, y, z) = val;}
  inline void   SetValL(int id, int64_t val, int x, int y=0, int z=0)     {*(int64_t*) m_buf[id].getPtr(x, y, z) = val;}
  inline void   SetValF(int id, float val, int x, int y=0, int z=0)     {*(float*)   m_buf[id].getPtr(x, y, z) = val;}

  // read helpers
  int read_F_CSV(std::string& rule_fn, std::string& name_fn );
  int read_constraints (std::string &fn );
  int filter_constraint(std::vector< std::vector< int32_t > > &constraint_list);

  uint64_t      m_num_verts;    // Xi = 0..X (graph domain)
  uint64_t      m_num_face;
  uint64_t      m_num_values;   // B = 0..Bm-1 (value domain)  
  
  DataPtr       m_buf[ BUF_MAX ];    // data buffers (CPU & GPU)  

  std::vector< std::string > m_tile_name;   // tile names
  std::vector< std::vector< int32_t > > m_admissible;
  int m_dir_inv[6];

  Mersenne      m_rand;

  lsa_opt_t     op;

  lsa_expr_t    expr;

  int           m_flip;
  float         m_resolve;

  Vector3DF     m_clr[512];  
};

int _read_line(FILE *fp, std::string &line);
//int _read_name_csv(std::string &fn, std::vector<std::string> &name,  std::vector< float >& tile_weight);
int _read_name_csv(std::string &fn, std::vector<std::string> &name, std::vector< float > &weight );
int _read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule );


#endif
