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
#include "main_belief_propagation.h"

#include "string_helper.h"
//#include "mesh.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "pd_getopt.h"
extern char *optarg;

opt_t g_opt;



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

Camera3D  m_cam;
Vector3DI m_vres;

const char VIZ_VOL=0;

int m_iresx,
    m_iresy;


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

//WIP
//
void visualize_belief ( BeliefPropagation& src, int bp_id, int vol_id, Vector3DI vres );

BeliefPropagation *g_bpc;
void bp_cb( void * dat ) {
  char imgfile[512];
  std::string base_png = "out";
  int it=0;
  static int base_it = -1;

  m_iresx = 512;
  m_iresy = 512;

  if (g_bpc->m_state_info_iter==0) { base_it++; }

  it = (int)g_bpc->m_state_info_iter;

  //printf("... %i\n", (int)g_bpc->m_state_info_iter); fflush(stdout);

  //visualize_belief ( g_bpc, BUF_BELIEF, VIZ_VOL, g_bpc->m_vres );
  visualize_belief ( *g_bpc, BUF_BELIEF, VIZ_VOL, m_vres );

  //raycast_cpu ( g_bpc->m_vres, &m_cam, VIZ_VOL, g_bpc->m_img, g_bpc->m_iresx, g_bpc->m_iresy, Vector3DF(0,0,0), Vector3DF(g_bpc->m_vres) );
  raycast_cpu ( m_vres, &m_cam, VIZ_VOL, m_img, m_iresx, m_iresy, Vector3DF(0,0,0), Vector3DF(m_vres) );

  //snprintf ( imgfile, 511, "%s%04d.png", base_png.c_str(), (int) it );
  snprintf ( imgfile, 511, "%s%04d.%04d.png", base_png.c_str(), (int) base_it, (int) it );

  printf ( "  output: %s\n", imgfile );
  save_png ( imgfile, m_img, m_iresx, m_iresy, 3 );

}

void bp_cb_0(void *dat) {
  printf("... %i\n", (int)g_bpc->m_state_info_iter); fflush(stdout);
}

void bp_cb_V2(void *dat) {
  static int base_it = -1;

  if (g_bpc->m_state_info_iter==0) { base_it++; }
  if ( (g_bpc->m_state_info_iter % 100) == 0) {
    printf("# [base_it:%i.m_state_info_iter:%i]\n", base_it, (int)g_bpc->m_state_info_iter);
  }
}

void bp_cb_v1(void *dat) {
  char imgfile[512];
  std::string base_png = "out";
  int it=0;
  static int base_it = -1;

  int64_t cell_idx, val_idx,
          n_dir, val_idx_n,
          nei_cell_idx,
          dir_idx;
  float residue,
        max_residue,
        t_f;

  int vol_id = VIZ_VOL,
      val;

  Vector4DF* vox = (Vector4DF*) m_vol[ vol_id ].getPtr (0);

  if (g_bpc->m_state_info_iter==0) { base_it++; }

  it = (int)g_bpc->m_state_info_iter;

  n_dir = g_bpc->getNumNeighbors(0);

  max_residue = 0.0;
  for (cell_idx=0; cell_idx < g_bpc->getNumVerts(); cell_idx++) {

    val_idx_n = g_bpc->getVali( BUF_TILE_IDX_N, cell_idx );
    for (val_idx=0; val_idx < val_idx_n; val_idx++) {

      val = g_bpc->getVali( BUF_TILE_IDX, cell_idx, val_idx );

      for (dir_idx=0; dir_idx < n_dir; dir_idx++) {
        nei_cell_idx = g_bpc->getNeighbor( cell_idx, dir_idx );
        if (nei_cell_idx < 0) { continue; }

        residue = g_bpc->getVal( BUF_MU_RESIDUE, dir_idx, cell_idx, val );

        if (max_residue < residue) { max_residue = residue; }
      }

    }
  }

  if (max_residue < (1/((1024.0*1024.0)))) { max_residue = 1.0; }

  for (cell_idx=0; cell_idx < g_bpc->getNumVerts(); cell_idx++) {

    t_f = 0.0;
    residue = -1.0;

    val_idx_n = g_bpc->getVali( BUF_TILE_IDX_N, cell_idx );
    for (val_idx=0; val_idx < val_idx_n; val_idx++) {

      val = g_bpc->getVali( BUF_TILE_IDX, cell_idx, val_idx );

      for (dir_idx=0; dir_idx < n_dir; dir_idx++) {
        nei_cell_idx = g_bpc->getNeighbor( cell_idx, dir_idx );
        if (nei_cell_idx < 0) { continue; }

        t_f = g_bpc->getVal( BUF_MU_RESIDUE, dir_idx, cell_idx, val );
        if (t_f > residue) { residue = t_f; }
      }

    }

    residue = (residue/max_residue);

    //printf("%i: %f / %f\n", (int)cell_idx, residue, max_residue);

    //residue = powf( residue, 0.5 );
    residue = powf( residue, g_opt.alpha );

    //printf("%f\n", residue);

    //if (residue < 0.0) { residue = -residue; }
    if (residue > 1.0) { residue = 1.0; }

    vox->x = residue;
    vox->y = 0.0;
    vox->z = 0.0;
    //vox->w = residue;
    vox->w = 0.25;

    vox++;
  }

  //raycast_cpu ( g_bpc->m_vres, &m_cam, VOL, g_bpc->m_img, g_bpc->m_iresx, g_bpc->m_iresy, Vector3DF(0,0,0), Vector3DF(g_bpc->m_vres) );
  raycast_cpu ( m_vres, &m_cam, VIZ_VOL, m_img, m_iresx, m_iresy, Vector3DF(0,0,0), Vector3DF(m_vres) );

  //snprintf ( imgfile, 511, "%s%04d.png", base_png.c_str(), (int) it );
  snprintf ( imgfile, 511, "%s%04d.%04d.png", base_png.c_str(), (int) base_it, (int) it );
  printf ( "  output: %s\n", imgfile );
  save_png ( imgfile, m_img, m_iresx, m_iresy, 3 );

}

void visualize_belief ( BeliefPropagation& src, int bp_id, int vol_id, Vector3DI vres ) {

  Vector4DF* vox = (Vector4DF*) m_vol[ vol_id ].getPtr (0);
  float maxv;

  int N = (int)(src.m_tile_name.size());;

  int r_l = 1,
      r_u = (N-1)/3;
  int g_l = r_u+1,
      g_u = 2*(N-1)/3;
  int b_l = g_u,
      b_u = N-1;

  // map belief to RGBA voxel
  //
  for ( uint64_t j=0; j < src.getNumVerts(); j++ ) {
    src.getVertexBelief (j);

    // red
    //
    maxv = 0.0;
    for (int k=r_l; k <= r_u; k++) {
      maxv = std::max(maxv, src.getVal( bp_id, k ));
    }
    vox->x = maxv;

    // green
    //
    maxv = 0.0;
    for (int k=g_l; k <= g_u; k++) {
      maxv = std::max(maxv, src.getVal( bp_id, k ));
    }
    vox->y = maxv;

    // blue
    //
    maxv = 0.0;
    for (int k=b_l; k <= b_u; k++) {
      maxv = std::max(maxv, src.getVal( bp_id, k ));
    }
    vox->z = maxv;

    vox->w = std::max(vox->x, std::max(vox->y, vox->z));
    vox++;
  }

}

void visualize_dmu ( BeliefPropagation& src, int bp_id, int vol_id, Vector3DI vres ) {

   Vector4DF* vox = (Vector4DF*) m_vol[ vol_id ].getPtr (0);
   float maxv;

   int N = (int)(src.m_tile_name.size());;

   int r_l = 1,
       r_u = (N-1)/3;
   int g_l = r_u+1,
       g_u = 2*(N-1)/3;
   int b_l = g_u,
       b_u = N-1;

   // map belief to RGBA voxel
   //
   for ( uint64_t j=0; j < src.getNumVerts(); j++ ) {
     src.getVertexBelief (j);

     // red
     //
     maxv = 0.0;
     for (int k=r_l; k <= r_u; k++) {
        maxv = std::max(maxv, src.getVal( bp_id, k ));
     }
     vox->x = maxv;

     // green
     //
     maxv = 0.0;
     for (int k=g_l; k <= g_u; k++) {
        maxv = std::max(maxv, src.getVal( bp_id, k ));
     }
     vox->y = maxv;

     // blue
     //
     maxv = 0.0;
     for (int k=b_l; k <= b_u; k++) {
        maxv = std::max(maxv, src.getVal( bp_id, k ));
     }
     vox->z = maxv;

     vox->w = std::max(vox->x, std::max(vox->y, vox->z));
     vox++;
   }

}

//-------------------------------------------------//
//                      _             _       _    //
//   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_  //
//  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __| //
// | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_  //
//  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__| //
//                                                 //
//-------------------------------------------------//

// Split string on separator
//   e.g. "object:car, other".. left='object:car', right='other'
bool strSplit_ ( std::string str, std::string sep, std::string& left, std::string& right ) {
  std::string result;
  size_t f1, f2;

  //f1 = str.find_first_not_of ( sep );
  //if ( f1 == std::string::npos ) f1 = 0;
  f1 = 0;
  f2 = str.find_first_of ( sep, f1 );
  if ( f2 != std::string::npos) {
    left = str.substr ( f1, f2-f1 );
    right = str.substr ( f2+1 );
    return true;
  }
  left = "";
  right = str;
  return false;
}


int parse_frange(std::vector<float> &range, std::string &s) {
  int err_code = 0, iret;
  float val;

  std::string cur_s = s;
  std::string innard;
  std::string csep = ":";
  std::string l_innard, r_innard;
  size_t pos=0;

  bool r;

  range.clear();
  range.push_back(0);
  range.push_back(0);

  if (s.size()==0) { return -1; }

  r = strSplit_( s, csep, l_innard, r_innard );
  if (!r) {
    iret = strToF(r_innard, val);
    if (iret<0) { return -1; }
    range[0] = val;
    range[1] = val;
    return 0;
  }

  if (l_innard.size() > 0) {
    iret = strToF(l_innard, val);
    if (iret < 0) { return -1; }
    range[0] = val;
  }

  if (r_innard.size() > 0) {
    iret = strToF(r_innard, val);
    if (iret < 0) { return -1; }
    range[1] = val;
  }

  return err_code;
}

int parse_range(std::vector<int> &range, std::string &s, std::vector<int> &dim) {
  int err_code = 0;
  int val, iret;

  std::string cur_s = s;
  std::string innard;
  std::string csep = ":";
  std::string l_innard, r_innard;
  size_t pos=0;

  bool r;

  range.clear();
  range.push_back(0);
  range.push_back(0);
  if (dim.size() > 0) { range[1] = dim[0]; }

  if (s.size()==0) { return -1; }

  r = strSplit_( s, csep, l_innard, r_innard );
  if (!r) {

    iret = strToI(r_innard, val);
    if (iret<0) { return -1; }
    if (val<0) {
      if (dim.size() > 0) {
        val = dim[0] + val;
      }
    }
    range[0] = val;
    range[1] = range[0]+1;
    return 0;
  }

  if (l_innard.size() > 0) {
    iret = strToI(l_innard, val);

    if (iret < 0) { return -1; }
    if (val < 0) {
      if (dim.size() > 0) {
        val = dim[0] + val;
      }
    }
    range[0] = val;
  }

  if (r_innard.size() > 0) {
    iret = strToI(r_innard, val);

    if (iret < 0) { return -1; }
    if (val < 0) {
      if (dim.size() > 0) {
        val = dim[0] + val;
      }
    }
    range[1] = val+1;
  }

  return err_code;
}

int parse_bracket_range(std::vector<int> &range, std::string &s, std::vector<int> &dim) {
  int dim_idx=0;
  int err_code = 0;
  int val, iret;

  std::string cur_s = s;
  std::string innard;
  std::string lsep = "[", rsep = "]", csep = ":";
  std::string l_innard, r_innard;
  size_t pos=0;

  bool r;

  range.clear();

  for (dim_idx=0; dim_idx<dim.size(); dim_idx++) {
    range.push_back(0);
    range.push_back(dim[dim_idx]);
  }

  if (s.size()==0) { return 0; }

  for (dim_idx=0; dim_idx<dim.size(); dim_idx++) {

    r = strGet( cur_s, lsep, rsep, innard, pos );
    if (!r) {
      err_code = -1-dim_idx;
      break;
    }

    // bad parse or not enclosed in brackets
    //
    if (innard.size()==0) { err_code = -4; break; }
    if ((innard[0] != lsep[0]) ||
        (innard[ innard.size()-1 ] != rsep[0])) {
      err_code = -5;
      break;
    }

    // lop off first range
    //
    cur_s  = cur_s.substr(pos + innard.size());

    if (innard.size()==2) { continue; }

    // lop off left and right separator
    //
    innard = innard.substr(1, innard.size()-2);

    r = strSplit_( innard, csep, l_innard, r_innard );
    if (!r) {
      iret = strToI(r_innard, val);
      if (iret<0) { return -1; }
      if (val<0) { val = dim[dim_idx] + val; }
      range[2*dim_idx] = val;
      range[2*dim_idx+1] = range[2*dim_idx]+1;
      continue;
    }

    if (l_innard.size() > 0) {
      iret = strToI(l_innard, val);
      if (iret < 0) { return -1; }
      if (val < 0) { val = dim[dim_idx] + val; }
      range[2*dim_idx] = val;
    }

    if (r_innard.size() > 0) {
      iret = strToI(r_innard, val);
      if (iret < 0) { return -1; }
      if (val < 0) { val = dim[dim_idx] + val; }
      range[2*dim_idx+1] = val;
    }


  }

  return err_code;
}

int parse_constraint_dsl(std::vector< constraint_op_t > &op_list, std::string &s, std::vector< int > dim, std::vector< std::string > name) {
  int i, n, r;
  std::vector< std::string > raw_tok;
  std::string ws_sep = " \n\t";
  std::vector< std::string > tok;
  std::vector< int > tiledim;

  constraint_op_t op;

  std::string srange;

  //tiledim.push_back(0);
  tiledim.push_back(name.size());

  op_list.clear();

  n = strSplitMultiple( s, ws_sep, raw_tok );

  for (i=0; i<n; i++) {
    if (raw_tok[i].size() > 0) {
      tok.push_back( raw_tok[i] );
    }
  }

  if ((tok.size()%2) != 0) { return -1; }

  for (i=0; i<tok.size(); i+=2) {

    op.dim_range.clear();
    op.tile_range.clear();

    srange = tok[i].substr(1);

    op.op = tok[i][0];
    r = parse_bracket_range(op.dim_range, srange, dim);
    if (r<0) { return -1; }

    r = parse_range(op.tile_range, tok[i+1], tiledim);
    if (r<0) { return -2; }

    op_list.push_back(op);
  }

  return 0;
}

void debug_constraint_op_list(std::vector< constraint_op_t > &op_list) {
  int i, j;
  for (i=0; i<op_list.size(); i++) {
    printf("op_list[%i] op:%c dim_range", i, op_list[i].op);
    for (j=0; j<op_list[i].dim_range.size(); j+=2) {
      printf("[%i:%i]",
          (int)op_list[i].dim_range[j],
          (int)op_list[i].dim_range[j+1]);
    }
    printf(" tile_range");
    for (j=0; j<op_list[i].tile_range.size(); j+=2) {
      printf("(%i:%i)",
          (int)op_list[i].tile_range[j],
          (int)op_list[i].tile_range[j+1]);
    }
    printf("\n");
  }
}

int constrain_bp(BeliefPropagation &bp, std::vector< constraint_op_t > &op_list) {
  int op_idx, i, j, k, n;
  int x,y,z,t;
  int64_t pos;

  std::vector<int32_t> v;

  debug_constraint_op_list(op_list);

  for (op_idx=0; op_idx<op_list.size(); op_idx++) {

    // discard
    //
    if (op_list[op_idx].op == 'd') {

      v.clear();
      for (t=op_list[op_idx].tile_range[0]; t<op_list[op_idx].tile_range[1]; t++) {
        v.push_back(t);
      }

      for (x=op_list[op_idx].dim_range[0]; x<op_list[op_idx].dim_range[1]; x++) {
        for (y=op_list[op_idx].dim_range[2]; y<op_list[op_idx].dim_range[3]; y++) {
          for (z=op_list[op_idx].dim_range[4]; z<op_list[op_idx].dim_range[5]; z++) {
            pos = bp.getVertex(x,y,z);
            bp.filterDiscard(pos, v);
          }
        }
      }

    }

    // force (only)
    //
    else if (op_list[op_idx].op == 'f') {

      v.clear();
      for (t=op_list[op_idx].tile_range[0]; t<op_list[op_idx].tile_range[1]; t++) {
        v.push_back(t);
      }

      for (x=op_list[op_idx].dim_range[0]; x<op_list[op_idx].dim_range[1]; x++) {
        for (y=op_list[op_idx].dim_range[2]; y<op_list[op_idx].dim_range[3]; y++) {
          for (z=op_list[op_idx].dim_range[4]; z<op_list[op_idx].dim_range[5]; z++) {
            pos = bp.getVertex(x,y,z);
            bp.filterKeep(pos, v);
          }
        }
      }

    }

    else if (op_list[op_idx].op == 'a') {
      // sorry
    }

    else {
      return -1;
    }

  }

  return 0;
}

//------------//
//       _ _  //
//   ___| (_) //
//  / __| | | //
// | (__| | | //
//  \___|_|_| //
//            //
//------------//

//void stl_print(FILE *fp, std::vector< float > &tri, float dx=0.0, float dy=0.0, float dz=0.0);
void stl_print(FILE *, std::vector< float > &, float, float, float);

int write_bp_stl(opt_t &opt, BeliefPropagation &bp, std::vector< std::vector< float > > tri_lib) {
  FILE *fp=stdout;

  int i, j, k, n;
  int ix, iy, iz;

  float stride_x = 1.0,
        stride_y = 1.0,
        stride_z = 1.0;

  float cx=0.0, cy=0.0, cz=0.0,
        dx, dy, dz,
        nx, ny, nz;
  int64_t pos;
  int32_t tile_id;


  fp = fopen(opt.outstl_fn.c_str(), "w");
  if (!fp) { return -1; }

  for (ix=0; ix<bp.m_res.x; ix++) {
    for (iy=0; iy<bp.m_res.y; iy++) {
      for (iz=0; iz<bp.m_res.z; iz++) {
        pos = bp.getVertex(ix, iy, iz);

        tile_id = bp.getVali( BUF_TILE_IDX, pos, 0 );

        dx = (float)ix*stride_x + cx;
        dy = (float)iy*stride_y + cy;
        dz = (float)iz*stride_z + cz;

        if (tile_id >= tri_lib.size()) {
          fprintf(stderr, "ERROR: tile_id %i, exceeds tri (%i)\n",
              (int)tile_id, (int)tri_lib.size());
          continue;
        }

        stl_print(fp, tri_lib[tile_id], dx, dy, dz);

      }
    }
  }

  fclose(fp);

  return 0;
}

int write_tiled_json(opt_t &opt, BeliefPropagation &bpc) {
  FILE *fp;
  int i, j, n, tileset_size;
  int64_t vtx;

  int sy, ey_inc;

  int tilecount = (int)bpc.m_tile_name.size();
  tilecount--;

  //opt.tileset_width = ceil( sqrt( ((double)bpc.m_tile_name.size()) - 1.0 ) );
  opt.tileset_width = ceil( sqrt( (double)tilecount ) );
  opt.tileset_height = opt.tileset_width;

  opt.tileset_width *= opt.tileset_stride_x;
  opt.tileset_height *= opt.tileset_stride_y;


  fp = fopen( opt.tilemap_fn.c_str(), "w");
  if (!fp) { return -1; }

  fprintf(fp, "{\n");
  fprintf(fp, "  \"backgroundcolor\":\"#ffffff\",\n");
  fprintf(fp, "  \"height\": %i,\n", (int)bpc.m_res.y);
  fprintf(fp, "  \"width\": %i,\n", (int)bpc.m_res.x);
  fprintf(fp, "  \"layers\": [{\n");

  fprintf(fp, "    \"data\": [");

  // tiled expects y to increment in the negative direction
  // so we need to reverse the y direction when exporting
  //


  if (opt.tiled_reverse_y) {

    for (i=(int)(bpc.m_res.y-1); i>=0; i--) {
      for (j=0; j<(int)bpc.m_res.x; j++) {
        vtx = bpc.getVertex(j, i, 0);

        fprintf(fp, " %i", (int)bpc.getVali( BUF_TILE_IDX, vtx, 0 ));
        if ((i==0) && (j==(bpc.m_res.x-1))) { fprintf(fp, "%s",  ""); }
        else                                { fprintf(fp, "%s", ","); }
      }
      fprintf(fp, "\n  ");
    }

  }
  else {
    for (i=0; i<(int)(bpc.m_res.y); i++) {
      for (j=0; j<(int)bpc.m_res.x; j++) {
        vtx = bpc.getVertex(j, i, 0);

        fprintf(fp, " %i", (int)bpc.getVali( BUF_TILE_IDX, vtx, 0 ));
        if ((i==(bpc.m_res.y-1)) && (j==(bpc.m_res.x-1))) { fprintf(fp, "%s",  ""); }
        else                                { fprintf(fp, "%s", ","); }
      }
      fprintf(fp, "\n  ");
    }

  }

  /*
  n = bpc.m_num_verts;
  for (i=0; i<n; i++) {
    if ((i%(int)bpc.m_res.x)==0) {
      fprintf(fp, "\n   ");
    }
    fprintf(fp, " %i%s", bpc.getVali( BUF_TILE_IDX, i, 0 ), (i<(n-1)) ? "," : "" );
  }
  */

  fprintf(fp, "\n    ],\n");
  fprintf(fp, "    \"name\":\"main\",\n");
  fprintf(fp, "    \"opacity\":1,\n");
  fprintf(fp, "    \"type\":\"tilelayer\",\n");
  fprintf(fp, "    \"visible\":true,\n");
  fprintf(fp, "    \"width\": %i,\n", (int)bpc.m_res.x);
  fprintf(fp, "    \"height\": %i,\n", (int)bpc.m_res.y);
  fprintf(fp, "    \"x\":0,\n");
  fprintf(fp, "    \"y\":0\n");

  fprintf(fp, "  }\n");

  fprintf(fp, "  ],\n");
  fprintf(fp, "  \"nextobjectid\": %i,\n", 1);
  fprintf(fp, "  \"orientation\": \"%s\",\n", "orthogonal");
  fprintf(fp, "  \"properties\": [ ],\n");
  fprintf(fp, "  \"renderorder\": \"%s\",\n", "right-down");
  fprintf(fp, "  \"tileheight\": %i,\n", (int)opt.tileset_stride_y);
  fprintf(fp, "  \"tilewidth\": %i,\n", (int)opt.tileset_stride_x);
  fprintf(fp, "  \"tilesets\": [{\n");

  fprintf(fp, "    \"firstgid\": %i,\n", 1);
  fprintf(fp, "    \"columns\": %i,\n", (int)bpc.m_res.x);
  fprintf(fp, "    \"name\": \"%s\",\n", "tileset");
  fprintf(fp, "    \"image\": \"%s\",\n", opt.tileset_fn.c_str());
  fprintf(fp, "    \"imageheight\": %i,\n", (int)opt.tileset_height);
  fprintf(fp, "    \"imagewidth\": %i,\n", (int)opt.tileset_width);
  fprintf(fp, "    \"margin\": %i,\n", (int)opt.tileset_margin);
  fprintf(fp, "    \"spacing\": %i,\n", (int)opt.tileset_spacing);
  //fprintf(fp, "    \"tilecount\": %i,\n", (int)(bpc.m_tile_name.size()-1));
  fprintf(fp, "    \"tilecount\": %i,\n", tilecount);
  fprintf(fp, "    \"tileheight\": %i,\n", (int)opt.tileset_stride_y);
  fprintf(fp, "    \"tilewidth\": %i\n", (int)opt.tileset_stride_x);

  fprintf(fp, "  }],\n");
  fprintf(fp, "  \"version\": %i\n", 1);
  fprintf(fp, "}\n");

  fclose(fp);

  return 0;
}

void show_usage(FILE *fp) {
  fprintf(fp, "usage:\n");
  fprintf(fp, "\n");
  fprintf(fp, "    bpc [-h] [-v] [-N <name_file>] [-R <rule_file] [-C <fn>] [-T <test#>] [-D <#>] [-X <#>] [-Y <#>] [-Z <#>]\n");
  fprintf(fp, "\n");
  fprintf(fp, "  -N <fn>  CSV name file\n");
  fprintf(fp, "  -R <fn>  CSV rule file\n");
  fprintf(fp, "  -C <fn>  constrained realization file\n");
  fprintf(fp, "  -J <dsl> constraint dsl to help populuate/cull initial grid\n");
  fprintf(fp, "  -e <#>   set convergence epsilon\n");
  fprintf(fp, "  -z <#>   set zero epsilon\n");
  fprintf(fp, "  -w <#>   set (update) rate\n");
  fprintf(fp, "  -I <#>   set max step iteration\n");
  fprintf(fp, "  -W       run 'wave function collapse' instead of belief propagation\n");
  fprintf(fp, "  -D <#>   set X,Y,Z = D\n");
  fprintf(fp, "  -X <#>   set X\n");
  fprintf(fp, "  -Y <#>   set Y\n");
  fprintf(fp, "  -Z <#>   set Z\n");
  fprintf(fp, "  -T <#>   run test number\n");
  fprintf(fp, "  -S <#>   seed\n");
  fprintf(fp, "  -G <#>   algorithm choice\n");
  fprintf(fp, "    0      fix maximum belief tile (default)\n");
  fprintf(fp, "    1      remove minimum belief tile\n");
  fprintf(fp, "    2      fix maximum belief tile in minimum entropy cell\n");
  fprintf(fp, "    3      remove min. belief tile from minimum entropy cell\n");
  fprintf(fp, "    4      use residue algorithm (schedule max residue updates until convergence)\n");
  fprintf(fp, "  -E       use SVD decomposition speedup (default off)\n");
  fprintf(fp, "  -B       use checkboard speedup (default off)\n");
  fprintf(fp, "  -A <#>   alpha (for visualization)\n");

  fprintf(fp, "  -M <fn>  output tilemap (JSON)\n");
  fprintf(fp, "  -Q <fn>  tileset filename (PNG)\n");
  fprintf(fp, "  -u       reverse y for tiled output (default 0)\n");
  fprintf(fp, "  -s <#>   png tile stride\n");
  fprintf(fp, "  -c <#>   cull tile id\n");

  //fprintf(fp, "  -d       debug print\n");

  fprintf(fp, "  -V <#>   set verbosity level (default 0)\n");
  fprintf(fp, "  -r       enable raycast visualization\n");

  fprintf(fp, "  -v       show version\n");
  fprintf(fp, "  -h       help (this screen)\n");
  fprintf(fp, "\n");
}

void show_version(FILE *fp) {
  fprintf(fp, "bp version: %s\n", BELIEF_PROPAGATION_VERSION);
}

/*
void fprint_obj(FILE *fp, tinyobj::ObjReader &reader, float dx, float dy, float dz) {
  int i, j, k;
  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();

  for (i=0; i<attrib.size(); i+=3) {
    fprintf(fp, "v %f %f %f\n",
        (float)(attrib.vertices[i+0] + dx),
        (float)(attrib.vertices[i+1] + dy),
        (float)(attrib.vertices[i+2] + dz));
  }

  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {

    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {

        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
        tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
        tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

        fprintf(fp, "%f %f %f\n", (float)(vx+dx), (float)(vy+dy), (float)(vz+dz));

        // Check if `normal_index` is zero or positive. negative = no normal data
        if (idx.normal_index >= 0) {
          tinyobj::real_t nx = attrib.normals[3*size_t(idx.normal_index)+0];
          tinyobj::real_t ny = attrib.normals[3*size_t(idx.normal_index)+1];
          tinyobj::real_t nz = attrib.normals[3*size_t(idx.normal_index)+2];
        }

        // Check if `texcoord_index` is zero or positive. negative = no texcoord data
        if (idx.texcoord_index >= 0) {
          tinyobj::real_t tx = attrib.texcoords[2*size_t(idx.texcoord_index)+0];
          tinyobj::real_t ty = attrib.texcoords[2*size_t(idx.texcoord_index)+1];
        }

        // Optional: vertex colors
        // tinyobj::real_t red   = attrib.colors[3*size_t(idx.vertex_index)+0];
        // tinyobj::real_t green = attrib.colors[3*size_t(idx.vertex_index)+1];
        // tinyobj::real_t blue  = attrib.colors[3*size_t(idx.vertex_index)+2];
      }
      index_offset += fv;

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }


}
*/

int grid_obj2stl_out(std::string ofn, BeliefPropagation &bp, std::vector< std::vector< float > > tri) {
  int i, j, k, n;

  float stride_x = 1.0,
        stride_y = 1.0,
        stride_z = 1.0;

  float cx = 0.0,
        cy = 0.0,
        cz = 0.0;

  float dx = 1.0,
        dy = 1.0,
        dz = 1.0;

  float nx, ny, nz;

  int ix, iy, iz;
  int64_t pos;
  int32_t tile_id;

  FILE *fp;

  fp = fopen(ofn.c_str(), "w");
  if (!fp) { return -1; }

  for (ix=0; ix<bp.m_res.x; ix++) {
    for (iy=0; iy<bp.m_res.y; iy++) {
      for (iz=0; iz<bp.m_res.z; iz++) {
        pos = bp.getVertex(ix, iy, iz);

        tile_id = bp.getVali( BUF_TILE_IDX, pos, 0 );

        dx = (float)ix*stride_x + cx;
        dy = (float)iy*stride_y + cy;
        dz = (float)iz*stride_z + cz;

        if (tile_id >= tri.size()) {
          fprintf(stderr, "ERROR: tile_id %i, exceeds tri (%i)\n",
              (int)tile_id, (int)tri.size());
          continue;
        }

        fprintf(fp, "solid\n");
        for (i=0; i<tri[tile_id].size(); i+=9) {
          fprintf(fp, "  facet normal %f %f %f\n",
              (float)nx, (float)ny, (float)nz);

          for (j=0; j<3; j++) {
            fprintf(fp, "    vertex %f %f %f\n",
              (float)tri[tile_id][i + 3*j + 0],
              (float)tri[tile_id][i + 3*j + 1],
              (float)tri[tile_id][i + 3*j + 2]);
          }

          fprintf(fp, "  endfacet\n");
        }
        fprintf(fp, "endsolid\n");

      }
    }
  }

  fclose(fp);

  return 0;
}

int load_obj2tri(std::string inputfile, std::vector< float > &tri) {

  tri.clear();

  //std::string inputfile = "./examples/.data/s000.obj";
  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = "./"; // Path to material files

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(inputfile, reader_config)) {
    if (!reader.Error().empty()) {
        std::cerr << "TinyObjReader[E]: " << reader.Error();
    }
    return -1;
  }

  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader[W]: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();
  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {

    //printf("#shape %i\n", (int)s);

    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      //printf("# face %i\n", (int)f);

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {

        //printf("#  vertex %i\n", (int)v);

        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
        tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
        tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

        tri.push_back(vx);
        tri.push_back(vy);
        tri.push_back(vz);

        // Check if `normal_index` is zero or positive. negative = no normal data
        if (idx.normal_index >= 0) {

          tinyobj::real_t nx = attrib.normals[3*size_t(idx.normal_index)+0];
          tinyobj::real_t ny = attrib.normals[3*size_t(idx.normal_index)+1];
          tinyobj::real_t nz = attrib.normals[3*size_t(idx.normal_index)+2];

          printf("!! normal: %f %f %f\n", nx, ny, nz);
        }

        // Check if `texcoord_index` is zero or positive. negative = no texcoord data
        if (idx.texcoord_index >= 0) {
          tinyobj::real_t tx = attrib.texcoords[2*size_t(idx.texcoord_index)+0];
          tinyobj::real_t ty = attrib.texcoords[2*size_t(idx.texcoord_index)+1];
        }

        // Optional: vertex colors
        // tinyobj::real_t red   = attrib.colors[3*size_t(idx.vertex_index)+0];
        // tinyobj::real_t green = attrib.colors[3*size_t(idx.vertex_index)+1];
        // tinyobj::real_t blue  = attrib.colors[3*size_t(idx.vertex_index)+2];
      }
      index_offset += fv;

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }

  return 0;

  /*
  tinyobj::attrib_t ax;
  std::vector< tinyobj::shape_t > sx;
  std::vector< tinyobj::material_t > mx;

  ax = attrib;
  sx = shapes;
  mx = materials;

  bool coordTransform = false;
  bool ignoreMaterial = true;

  //tinyobj::WriteObj( "./akok.0.obj", attrib, shapes, materials, coordTransform, ignoreMaterial);
  tinyobj::WriteObj( "./akok.0.obj", ax, sx, mx, coordTransform, ignoreMaterial);

  for (size_t i=0; i<ax.vertices.size(); i+=3) {
    ax.vertices[i+0] += 2.0;
    ax.vertices[i+1] += 0.0;
    ax.vertices[i+2] += 0.0;
  }

  //tinyobj::WriteObj( "./akok.1.obj", attrib, shapes, materials, coordTransform, ignoreMaterial);
  tinyobj::WriteObj( "./akok.1.obj", ax, sx, mx, coordTransform, ignoreMaterial);

  exit(-1);
  return;
  */

  /*
  bool br;
  MeshX mm;

  br = mm.LoadObj("./examples/.data/s000.obj", 1.0);
  if (br) { printf("load successful\n"); }
  else { printf("load failed\n"); }

  exit(-1);
  */

}

void stl_print(FILE *fp, std::vector< float > &tri, float dx=0.0, float dy=0.0, float dz=0.0) {
  int i, j;
  float nx=0.0, ny=0.0, nz=0.0;

  fprintf(fp, "solid\n");
  for (i=0; i<tri.size(); i+=9) {
    fprintf(fp, "  facet normal %f %f %f\n",
        (float)nx, (float)ny, (float)nz);

    fprintf(fp, "    outer loop\n");
    for (j=0; j<3; j++) {
      fprintf(fp, "      vertex %f %f %f\n",
        (float)tri[i + 3*j + 0] + dx,
        (float)tri[i + 3*j + 1] + dy,
        (float)tri[i + 3*j + 2] + dz);
    }

    fprintf(fp, "    endloop\n");
    fprintf(fp, "  endfacet\n");
  }
  fprintf(fp, "endsolid\n");
}

int load_obj_stl_lib(std::string fn, std::vector< std::vector< float > > &tris) {
  int i, j, k, ret;
  std::vector< std::string > obj_fns;
  std::vector< float > w;
  std::vector< float > tri;

  ret = _read_name_csv(fn, obj_fns, w);
  if (ret<0) { return ret; }

  for (i=0; i<obj_fns.size(); i++) {
    ret = load_obj2tri(obj_fns[i], tri);
    if (ret<0) { return ret; }
    tris.push_back(tri);
  }

  return 0;
}

int main(int argc, char **argv) {
  int i, j, k, idx, ret;
  char ch;

  char *name_fn = NULL, *rule_fn = NULL, *constraint_fn = NULL;
  std::string name_fn_str,
              rule_fn_str,
              constraint_fn_str;

  int test_num = -1;
  int X=0, Y=0, Z=0, D=0;

  int wfc_flag = 0;
  int raycast = 0;
  int debug_print = 0;
  int seed = 0;

  int iresx=0, iresy=0;
  Vector3DI vres (X, Y, Z);
  Camera3D cam;

  std::string base_png = "out";
  char imgfile[512] = {0};

  float eps_zero = -1.0,
        eps_converge = -1.0,
        step_factor = 1.0;
  std::string _eps_str;
  std::vector<float> eps_range;

  int max_iter = -1, it, n_it;

  std::vector< std::vector< int32_t > > constraint_list;

  std::vector< int32_t > cull_list;
  std::string constraint_commands;
  std::vector< constraint_op_t > constraint_op_list;

  std::vector< std::vector< float > > tri_shape_lib;

  BeliefPropagation bpc;

  eps_range.push_back( bpc.m_eps_converge );
  eps_range.push_back( bpc.m_eps_converge );

  int arg=1;

  void (*_cb_f)(void *) = NULL;

  g_bpc = &bpc;

  g_opt.tiled_reverse_y = 0;
  g_opt.alpha = 0.5;
  g_opt.alg_idx = 0;
  while ((ch=pd_getopt(argc, argv, "hvdV:r:e:z:I:N:R:C:T:WD:X:Y:Z:S:A:G:w:EBQ:M:s:c:uJ:L:")) != EOF) {
    switch (ch) {
      case 'h':
        show_usage(stdout);
        exit(0);
        break;
      case 'v':
        show_version(stdout);
        exit(0);
        break;

      //case 'd':
      //  debug_print = 1;
      //  break;

      case 'V':
        bpc.m_verbose = atoi(optarg);
        break;
      case 'r':
        raycast = 1;
        iresx = atoi(optarg);
        iresy = iresx;

        m_iresx = iresx;
        m_iresy = iresy;

        break;

      case 'A':
        g_opt.alpha = atof(optarg);
        break;
      case 'G':
        g_opt.alg_idx = atoi(optarg);
        break;

      case 'e':
        /*
        eps_converge = atof(optarg);
        if (eps_converge > 0.0) {
          bpc.m_eps_converge = eps_converge;
        }
        */

        _eps_str = optarg;

        ret = parse_frange( eps_range, _eps_str );
        if (ret < 0) {
          fprintf(stderr, "bad value for convergence epsilon\n");
          show_usage(stderr);
          exit(-1);
        }
        bpc.m_eps_converge_beg = eps_range[0];
        bpc.m_eps_converge_end = eps_range[1];

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
      case 'w':
        step_factor = atof(optarg);
        if (step_factor > 0.0) {
          bpc.m_rate = step_factor;
        }
        break;

      case 'N':
        name_fn = strdup(optarg);

        g_opt.fn_name = name_fn;
        break;
      case 'R':
        rule_fn = strdup(optarg);

        g_opt.fn_rule = rule_fn;
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

      case 'c':
        cull_list.push_back( (int32_t)atoi(optarg) );
        break;
      case 'J':
        constraint_commands = optarg;
        break;

      case 'W':
        wfc_flag = 1;
        break;

      case 'E':
        bpc.m_use_svd = 1;
        break;
      case 'B':
        bpc.m_use_checkerboard = 1;
        break;

      case 'L':
        g_opt.tileobj_fn = optarg;
        break;
      case 'Q':
        g_opt.tileset_fn = optarg;
        break;
      case 'M':
        g_opt.tilemap_fn = optarg;
        break;
      case 's':
        g_opt.tileset_stride_x = atoi(optarg);
        g_opt.tileset_stride_y = g_opt.tileset_stride_x;
        break;
      case 'u':
        g_opt.tiled_reverse_y = 1;
        break;

      default:
        show_usage(stderr);
        exit(-1);
        break;
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
    fprintf(stderr, "dimensions must all be >0 (%i,%i,%i)\n", X,Y,Z);
    show_usage(stderr);
    exit(-1);
  }

  name_fn_str = name_fn;
  rule_fn_str = rule_fn;

  if (g_opt.tileobj_fn.size() > 0) {
    ret=load_obj_stl_lib( g_opt.tileobj_fn, tri_shape_lib );
    if (ret<0) {
      fprintf(stderr, "ERROR: when trying to load '%s' (load_obj_stl_lib)\n", g_opt.tileobj_fn.c_str());
      exit(-1);
    }
  }


  if (constraint_commands.size() == 0) {
    if (constraint_fn) {
      constraint_fn_str = constraint_fn;
      _read_constraint_csv(constraint_fn_str, constraint_list);

      if (bpc.m_verbose > 0) {
        printf ( "reading constraints file. %s, %d\n", constraint_fn_str.c_str(), (int) constraint_list.size() );
      }
    }
  }

  if (bpc.m_verbose > 0) {
    printf ( "bpc init csv. (%s, %s)\n",
        name_fn_str.c_str(),
        rule_fn_str.c_str() );
    fflush(stdout);
  }

  ret = bpc.init_CSV(X,Y,Z,name_fn_str, rule_fn_str);

  if (ret<0) {
    fprintf(stderr, "error loading CSV\n"); fflush(stderr);
    exit(-1);
  }

  if (constraint_commands.size() > 0) {
    std::vector< int > dim;
    dim.push_back(X);
    dim.push_back(Y);
    dim.push_back(Z);

    ret = parse_constraint_dsl(constraint_op_list, constraint_commands, dim, bpc.m_tile_name);
    if (ret < 0) {
      fprintf(stderr, "incorrect syntax when parsing constraint DSL\n");
      exit(-1);
    }

    ret = constrain_bp( bpc, constraint_op_list);
    if (ret < 0) {
      fprintf(stderr, "constrain_bp failure\n");
      exit(-1);
    }

    /*
    if (bpc.m_verbose > 1) {
      printf("*************** after constraint dsl:\n");
      bpc.debugPrint();
    }
    */

  }
  else if (constraint_fn) {
    if (bpc.m_verbose > 0) {
      printf ( "#filter constraints.\n" );
    }
    bpc.filter_constraint(constraint_list);
  }

  if (cull_list.size() > 0) {
    int cull_idx;
    int64_t tile_idx, pos;
    int32_t tile_id, n, cull_tile_id;
    if (bpc.m_verbose > 0) {
      printf( "#culling tile ids\n" );
    }
    for (cull_idx=0; cull_idx<cull_list.size(); cull_idx++) {
      cull_tile_id = cull_list[cull_idx];

      for (pos=0; pos<bpc.m_num_verts; pos++) {
        n = bpc.getVali( BUF_TILE_IDX_N, pos );
        for (tile_idx=0; tile_idx<n; tile_idx++) {
          if (bpc.getVali( BUF_TILE_IDX, pos, tile_idx ) == cull_tile_id) {
            break;
          }
        }
        if (tile_idx < n) {
          if (bpc.m_verbose > 1) {
            printf("#culling tile %i from cell %i (tile_idx:%i)\n", (int)cull_tile_id, (int)pos, (int)tile_idx);
          }
          tile_id = bpc.getVali( BUF_TILE_IDX, pos, n-1 );
          bpc.SetVali( BUF_TILE_IDX, pos, n-1, cull_tile_id );
          bpc.SetVali( BUF_TILE_IDX, pos, tile_idx, tile_id );
          n--;
          bpc.SetVali( BUF_TILE_IDX_N, pos, n );
        }
      }
    }
  }

  /*
  if (debug_print) {

    //DEBUG!!!!
    //testing out residual belief propagation wwork
    //

    int64_t idx, n, _cell;
    int32_t _tile, _idir;
    float _a, _b;

    n = bpc.m_num_values * bpc.m_num_verts*6;
    for (idx=0; idx<n; idx++) {
      bpc.getMuPos( idx, &_idir, &_cell, &_tile );

      _a = bpc.m_rand.randF();
      _b = bpc.m_rand.randF();

      printf(" heap_idx:%i -> (idir:%i, cell:%i, tile:%i) mu_cur:%f, mu_nxt:%f\n",
          (int)idx, (int)_idir, (int)_cell, (int)_tile, _a, _b);
      bpc.SetVal( BUF_MU,     _idir, _cell, _tile, _a );
      bpc.SetVal( BUF_MU_NXT, _idir, _cell, _tile, _b );
    }

    printf("---\n");

    printf("about to init:\n");
    bpc.indexHeap_init();

    printf("---\n");

    printf("heap:\n");
    bpc.indexHeap_debug_print();

    printf("---\n");

    ret = bpc.indexHeap_consistency();
    printf("indexHeap_consistency got: %i\n", (int)ret);
    exit(0);
    //
    //DEBUG!!!

    bpc.debugPrint();
    exit(0);
  }
  */

  if (test_num >= 0) {
    run_test(test_num);
    exit(0);
  }

  if (bpc.m_verbose > 1) {
    _cb_f = bp_cb_V2;
  }

  // prepare raycast [optional]
  //
  if (raycast) {

    //_cb_f = bp_cb;
    _cb_f = bp_cb_v1;

    vres.x = X;
    vres.y = Y;
    vres.z = Z;

    if (bpc.m_verbose > 0) {
      printf ( "preparing raycast.\n" );
    }
    alloc_img (iresx, iresy);
    alloc_volume (VIZ_VOL, vres, 4);
    cam.setOrbit ( 30, 20, 0, vres/2.0f, 50, 1 );

    if (bpc.m_verbose > 0) {
      printf ( "prepare raycast done. vol: %d,%d,%d  img: %d,%d\n", vres.x, vres.y, vres.z, iresx, iresy );
    }

    m_vres.x = X;
    m_vres.y = Y;
    m_vres.z = Z;

    m_cam.setOrbit( 30, 20, 0, m_vres/2.0f, 50, 1 );

    m_iresx = iresx;
    m_iresy = iresy;

  }

  if (wfc_flag) {

    if (bpc.m_verbose > 0) {
      printf ( "wfc realize.\n" );
    }
    ret = bpc.wfc();

    if (bpc.m_verbose > 0) {
      printf("# wfc got: %i\n", ret);
      bpc.debugPrint();
    }

  }

  else if (g_opt.alg_idx == 5) {
    ret = bpc.Realize();
    printf("bpc.Realize got: %i\n", ret);

    if (bpc.m_verbose > 0) {
      printf("# bp realize got: %i\n", ret);

      printf("####################### DEBUG PRINT\n" );
      bpc.debugPrint();
    }


  }
  else {

    if (bpc.m_verbose > 0) {
      printf ( "bpc realize.\n" );
    }
    ret = bpc.start();
    if (ret < 0) {
      printf("ERROR: bpc.start() failed (%i)\n", ret);

      if (bpc.m_verbose > 0) {
        printf("####################### DEBUG PRINT\n" );
        bpc.debugPrint();
      }

      exit(-1);
    }

    n_it = bpc.m_num_verts * bpc.m_num_values;

    //for (int64_t it=0; it < bpc.m_num_verts; it++) {
    for (it=0; it < n_it; it++) {

      //ret = bpc.single_realize_cb(it, NULL);
      //ret = bpc.single_realize_cb(it, bp_cb);

      if (g_opt.alg_idx == 1) {
        ret = bpc.single_realize_min_belief_cb(it, _cb_f);
      }
      else if (g_opt.alg_idx == 2) {
        ret = bpc.single_realize_min_entropy_max_belief_cb(it, _cb_f);
      }
      else if (g_opt.alg_idx == 3) {
        ret = bpc.single_realize_min_entropy_min_belief_cb(it, _cb_f);
      }
      else if (g_opt.alg_idx == 4) {
        ret = bpc.single_realize_residue_cb(it, _cb_f);
      }
      else {
        //ret = bpc.single_realize_cb(it, _cb_f);
        ret = bpc.single_realize_max_belief_cb(it, _cb_f);
      }

      if (ret<=0) { break; }

      if ( raycast )  {

        //DEBUG
        printf("BUF_BELIEF: %i, VIZ_VOL: %i\n", (int)BUF_BELIEF, (int)VIZ_VOL);
        visualize_belief ( bpc, BUF_BELIEF, VIZ_VOL, vres );

        raycast_cpu ( vres, &cam, VIZ_VOL, m_img, iresx, iresy, Vector3DF(0,0,0), Vector3DF(vres) );
        snprintf ( imgfile, 511, "%s%04d.png", base_png.c_str(), (int) it );

        if (bpc.m_verbose > 0) { printf ( "  output: %s\n", imgfile ); }
        save_png ( imgfile, m_img, iresx, iresy, 3 );
      }

    }

    if (bpc.m_verbose > 0) {
      printf("# bp realize got: %i\n", ret);

      printf("####################### DEBUG PRINT\n" );
      bpc.debugPrint();
    }

  }

  if (g_opt.tilemap_fn.size() > 0) {

    if (bpc.m_verbose > 1) {
      printf("writing tilemap (%s)\n", g_opt.tilemap_fn.c_str());
    }

    if (g_opt.tileobj_fn.size() > 0) {
      g_opt.outstl_fn = g_opt.tilemap_fn;
      write_bp_stl(g_opt, bpc, tri_shape_lib);
    }
    else {
      write_tiled_json(g_opt, bpc);
    }
  }

  if (name_fn) { free(name_fn); }
  if (rule_fn) { free(rule_fn); }
  if (constraint_fn) { free(constraint_fn); }

  return 0;
}
