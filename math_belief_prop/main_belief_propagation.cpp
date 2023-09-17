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

//#define TINYOBJLOADER_IMPLEMENTATION
//#include "tiny_obj_loader.h"

#include "pd_getopt.h"

#include "bp_helper.h"
#include "bp_experiment.h"

extern char *optarg;


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

/*
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
}
*/

/*
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
*/

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
      maxv = std::max(maxv, src.getValF( bp_id, k ));
    }
    vox->x = maxv;

    // green
    //
    maxv = 0.0;
    for (int k=g_l; k <= g_u; k++) {
      maxv = std::max(maxv, src.getValF( bp_id, k ));
    }
    vox->y = maxv;

    // blue
    //
    maxv = 0.0;
    for (int k=b_l; k <= b_u; k++) {
      maxv = std::max(maxv, src.getValF( bp_id, k ));
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
        maxv = std::max(maxv, src.getValF( bp_id, k ));
     }
     vox->x = maxv;

     // green
     //
     maxv = 0.0;
     for (int k=g_l; k <= g_u; k++) {
        maxv = std::max(maxv, src.getValF( bp_id, k ));
     }
     vox->y = maxv;

     // blue
     //
     maxv = 0.0;
     for (int k=b_l; k <= b_u; k++) {
        maxv = std::max(maxv, src.getValF( bp_id, k ));
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
int _g_default_block_retry_count;

void show_usage(FILE *fp) {
  fprintf(fp, "usage:\n");
  fprintf(fp, "\n");
  fprintf(fp, "    bpc [-h] [-v] [-N <name_file>] [-R <rule_file] [-C <fn>] [-T <test#>] [-D <#>] [-X <#>] [-Y <#>] [-Z <#>]\n");
  fprintf(fp, "\n");
  fprintf(fp, "  -N <fn>  CSV name file\n");
  fprintf(fp, "  -R <fn>  CSV rule file\n");
  fprintf(fp, "  -C <fn>  constrained realization file\n");
  fprintf(fp, "  -J <dsl> constraint dsl to help populuate/cull initial grid\n");
  fprintf(fp, "  -j <range> set range of admissible tiles for block algorithms\n");
  fprintf(fp, "  -e <#>   set convergence epsilon\n");
  fprintf(fp, "  -z <#>   set zero epsilon\n");
  fprintf(fp, "  -w <#>   set (update) rate\n");
  fprintf(fp, "  -I <#>   set max step iteration\n");
  fprintf(fp, "  -i <#>   set max iteration (default to #(cell) * #(tile) )\n");
  fprintf(fp, "  -D <#>   set X,Y,Z = D\n");
  fprintf(fp, "  -X <#>   set X\n");
  fprintf(fp, "  -Y <#>   set Y\n");
  fprintf(fp, "  -Z <#>   set Z\n");
  fprintf(fp, "  -T <#>   run test number\n");
  fprintf(fp, "  -S <#>   seed\n");
  fprintf(fp, "  -G <#>   Algorithm/BP decimation policy\n");
  fprintf(fp, "    0      fix maximum belief tile (default)\n");
  fprintf(fp, "    1      (unused)\n");
  fprintf(fp, "    2      after convergence, pick minimum entropy cell, maximum belief tile value\n");
  fprintf(fp, "    3      after convergence, pick minimum entropy cell, maximum belief tile value, wave acceleration\n");
  fprintf(fp, "    4      use residue algorithm (schedule max residue updates until convergence)\n");
  fprintf(fp, "    -1     'wave function collapse'\n");
  fprintf(fp, "    -2     Merrell's model syntehsis (sequencial)\n");
  fprintf(fp, "    -3     Merrell's model synthesis (random position)\n");
  fprintf(fp, "    -4     Merrell's model synthesis (random position and block size)\n");
  fprintf(fp, "    -5     breakout model synthesis\n");
  fprintf(fp, "    -6     breakout model synthesis, min entorpy block choice\n");
  fprintf(fp, "    -7     breakout model synthesis, min entropy block + noise choice\n");
  fprintf(fp, "    -8     breakout model synthesis, max entropy block + noise choice\n");
  fprintf(fp, "  -b <#>   block size (for use in MMS and breakout, default 8x8x8, clamped to dimension)\n");
  fprintf(fp, "  -m <#>   block size retry count (default %i)\n", _g_default_block_retry_count);
  fprintf(fp, "  -a <#>   noise function parameters\n");
  fprintf(fp, "  -E       use SVD decomposition speedup (default off)\n");
  fprintf(fp, "  -B       use checkboard speedup (default off)\n");
  fprintf(fp, "  -A <#>   alpha (for visualization)\n");

  fprintf(fp, "  -M <fn>  output tilemap (JSON) or 3D file (STL) (STL output if object location, '-L', option used)\n");
  fprintf(fp, "  -L <fn>  use object location CSV file for location of 3D object files for each tile name\n");
  fprintf(fp, "  -Q <fn>  tileset filename (PNG)\n");
  fprintf(fp, "  -u       reverse y for tiled output (default 0)\n");
  fprintf(fp, "  -s <#>   png tile stride\n");

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

int main(int argc, char **argv) {
  int i, j, k, idx, ret=0;
  char ch = -1;

  char *name_fn = NULL, *rule_fn = NULL, *constraint_fn = NULL;
  std::string name_fn_str,
              rule_fn_str,
              constraint_fn_str;

  int test_num = -1;
  int X=0, Y=0, Z=0, D=0;

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

  int max_iter = -1,
      it = -1,
      n_it = -1;

  int64_t cell=-1;
  int32_t n_idx=-1,
          tile=-1,
          tile_idx=-1 ;

  std::vector< std::vector< int32_t > > constraint_list;
  std::vector< std::vector< float > > tri_shape_lib;

  BeliefPropagation bpc;

  _g_default_block_retry_count = bpc.m_block_retry_limit;

  eps_range.push_back( bpc.op.eps_converge );
  eps_range.push_back( bpc.op.eps_converge );

  int arg=1;

  bpc.op.tiled_reverse_y = 0;
  bpc.op.alpha = 0.5;
  bpc.op.alg_idx = 0;
  bpc.op.eps_zero = 1.0/256.0;
  while ((ch=pd_getopt(argc, argv, "hvdV:r:e:z:I:i:N:R:C:T:D:X:Y:Z:S:A:G:w:EBQ:M:s:c:uJ:L:lb:j:m:a:U:")) != EOF) {
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
        bpc.op.verbose = atoi(optarg);
        break;
      case 'r':
        raycast = 1;
        iresx = atoi(optarg);
        iresy = iresx;

        m_iresx = iresx;
        m_iresy = iresy;

        break;

      case 'A':
        bpc.op.alpha = atof(optarg);
        break;
      case 'G':
        bpc.op.alg_idx = atoi(optarg);
        break;

      case 'e':
        _eps_str = optarg;

        ret = parse_frange( eps_range, _eps_str );
        if (ret < 0) {
          fprintf(stderr, "bad value for convergence epsilon\n");
          show_usage(stderr);
          exit(-1);
        }
        bpc.op.eps_converge_beg = eps_range[0];
        bpc.op.eps_converge_end = eps_range[1];

        break;
      case 'z':
        eps_zero = atof(optarg);
        if (eps_zero > 0.0) {
          bpc.op.eps_zero = eps_zero;
        }
        break;

      case 'I':
        max_iter = atoi(optarg);
        if (max_iter > 0) {
          bpc.op.max_step = (int64_t)max_iter;
        }
        break;
      case 'i':
        n_it = atoi(optarg);
        break;
      case 'm':
        bpc.m_block_retry_limit = atoi(optarg);
        break;
      case 'a':
        bpc.op.noise_coefficient = atof(optarg);
        break;

      case 'w':
        step_factor = atof(optarg);
        if (step_factor > 0.0) {
          bpc.op.step_rate = step_factor;
        }
        break;

      case 'N':
        name_fn = strdup(optarg);

        bpc.op.name_fn = name_fn;
        break;
      case 'R':
        rule_fn = strdup(optarg);

        bpc.op.rule_fn = rule_fn;
        break;
      case 'C':
        constraint_fn = strdup(optarg);
        break;

      case 'S':
        seed = atoi(optarg);
        bpc.op.seed = seed;
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

      case 'j':
        bpc.op.admissible_tile_range_cmd = optarg;
        break;
      case 'J':
        bpc.op.constraint_cmd = optarg;
        break;

      case 'E':
        bpc.op.use_svd = 1;
        break;
      case 'B':
        bpc.op.use_checkerboard = 1;
        break;
      case 'l':
        bpc.op.use_lookahead = 1;
        break;

      case 'L':
        bpc.op.tileobj_fn = optarg;
        break;
      case 'Q':
        bpc.op.tileset_fn = optarg;
        break;
      case 'M':
        bpc.op.tilemap_fn = optarg;
        break;
      case 's':
        bpc.op.tileset_stride_x = atoi(optarg);
        bpc.op.tileset_stride_y = bpc.op.tileset_stride_x;
        break;
      case 'u':
        bpc.op.tiled_reverse_y = 1;
        break;

      case 'b':
        bpc.op.block_size[0] = atoi(optarg);
        bpc.op.block_size[1] = atoi(optarg);
        bpc.op.block_size[2] = atoi(optarg);

        bpc.op.sub_block_range[0][0] = 1;
        bpc.op.sub_block_range[1][0] = 1;
        bpc.op.sub_block_range[2][0] = 1;
        bpc.op.sub_block_range[0][1] = bpc.op.block_size[0];
        bpc.op.sub_block_range[1][1] = bpc.op.block_size[1];
        bpc.op.sub_block_range[2][1] = bpc.op.block_size[2];
        break;

      case 'U':
        bpc.op.experiment_str = optarg;
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

  if (bpc.op.tileobj_fn.size() > 0) {
    ret=load_obj_stl_lib( bpc.op.tileobj_fn, tri_shape_lib );
    if (ret<0) {
      fprintf(stderr, "ERROR: when trying to load '%s' (load_obj_stl_lib)\n", bpc.op.tileobj_fn.c_str());
      exit(-1);
    }
  }


  if (bpc.op.verbose > 0) {
    printf ( "bpc init csv. (%s, %s)\n",
        name_fn_str.c_str(),
        rule_fn_str.c_str() );
    fflush(stdout);
  }

  // clamp block size
  //
  if (X < bpc.op.block_size[0]) { bpc.op.block_size[0] = X; }
  if (Y < bpc.op.block_size[1]) { bpc.op.block_size[1] = Y; }
  if (Z < bpc.op.block_size[2]) { bpc.op.block_size[2] = Z; }

  ret = bp_init_CSV( bpc, X,Y,Z, name_fn_str, rule_fn_str );

  if (ret<0) {
    fprintf(stderr, "error loading CSV\n"); fflush(stderr);
    exit(-1);
  }

  if (test_num >= 0) {
    run_test(bpc, test_num);
    exit(0);
  }

  //if (bpc.op.experiment_idx >= 0) {
  if ( bpc.op.experiment_str.size() > 0 ) {
    ret = run_experiment(bpc);
    exit(ret);
  }

  // prepare raycast [optional]
  //
  if (raycast) {

    vres.x = X;
    vres.y = Y;
    vres.z = Z;

    if (bpc.op.verbose > 0) {
      printf ( "preparing raycast.\n" );
    }
    alloc_img (iresx, iresy);
    alloc_volume (VIZ_VOL, vres, 4);
    cam.SetOrbit ( 30, 20, 0, vres/2.0f, 50, 1 );

    if (bpc.op.verbose > 0) {
      printf ( "prepare raycast done. vol: %d,%d,%d  img: %d,%d\n", vres.x, vres.y, vres.z, iresx, iresy );
    }

    m_vres.x = X;
    m_vres.y = Y;
    m_vres.z = Z;

    m_cam.SetOrbit( 30, 20, 0, m_vres/2.0f, 50, 1 );

    m_iresx = iresx;
    m_iresy = iresy;

  }

  //-------------------
  // Algorithm profile
  //-------------------

  bpc.SelectAlgorithm ( bpc.op.alg_idx );


  //if (bpc.op.verbose > 0) { printf ( "bpc realize.\n" ); }

  //------------ NOTE 
  // should replace bpc.start() with bp_restart()
  // to avoid duplicate code in bp_helper
  //

  ret = bp_restart(bpc);
  if (ret < 0) {
    printf("ERROR: bp_restart() failed (%i)\n", ret);
    if (bpc.op.verbose > 0) {
      printf("####################### DEBUG PRINT\n" );
      //bpc.debugPrint();
      bpc.debugPrintTerse();
    }
    exit(-1);
  }

  //----
  //----
  //----

  // set default if not specified on command line
  //
  if (n_it <= 0) {
    n_it = bpc.m_num_verts * bpc.m_num_values;
  }

  if (bpc.op.verbose >= VB_DEBUG) {
    printf("AFTER INIT:... (n_it:%i)\n", (int)n_it);
    bpc.debugPrintTerse();
  }

  for (it=0; it < n_it; it++) {

    if (it > 0) {
      ret = bpc.RealizePre();
      if (ret < 0) {
        printf("RealizePre failed with %i (it:%i)\n", (int)ret, (int)it);
        break;
      }
    }

    ret = 1;
    while (ret>0) {
      ret = bpc.RealizeStep();
    }

    ret = bpc.RealizePost();
    if (ret <= 0) { break; }

    if (bpc.m_return == 0) {

      if (bpc.op.verbose >= VB_STEP) {

        if (bpc.op.alg_idx == ALG_MMS_SEQ) {
          if (bpc.op.tileobj_fn.size() > 0) {
            bpc.op.cur_run = it;
            printf("# step:%i: writing stl\n", (int)bpc.op.cur_run);
            fflush(stdout);
            bpc.op.outstl_fn = bpc.op.tilemap_fn;
            write_bp_stl( bpc, tri_shape_lib );
          }
          else {
            bpc.op.cur_run = it;
            printf("# step:%i: writing tiled json\n", (int)bpc.op.cur_run);
            fflush(stdout);
            write_tiled_json( bpc );
          }
        }

      }

      //printf("success!\n");
      //bpc.debugPrintTerse();
    }
    else {
      //printf("FAIL\n");
      //bpc.debugPrintTerse();
    }

    if ( raycast )  {

      //DEBUG
      printf("BUF_BELIEF: %i, VIZ_VOL: %i\n", (int)BUF_BELIEF, (int)VIZ_VOL);
      visualize_belief ( bpc, BUF_BELIEF, VIZ_VOL, vres );

      raycast_cpu ( vres, &cam, VIZ_VOL, m_img, iresx, iresy, Vector3DF(0,0,0), Vector3DF(vres) );
      snprintf ( imgfile, 511, "%s%04d.png", base_png.c_str(), (int) it );

      if (bpc.op.verbose > 0) { printf ( "  output: %s\n", imgfile ); }
      save_png ( imgfile, m_img, iresx, iresy, 3 );
    }

  }

  //----
  //----
  //----

  if (bpc.op.verbose > 0) {
    printf("# bp realize got: %i\n", ret);

    printf("####################### DEBUG PRINT\n" );
    bpc.debugPrintTerse();
  }

  //----
  //----
  //----

  if (bpc.op.tilemap_fn.size() > 0) {

    if (bpc.op.verbose > 1) {
      printf("writing tilemap (%s)\n", bpc.op.tilemap_fn.c_str());
      fflush(stdout);
    }

    if (bpc.op.tileobj_fn.size() > 0) {
      printf("writing stl\n");
      fflush(stdout);
      bpc.op.outstl_fn = bpc.op.tilemap_fn;
      write_bp_stl( bpc, tri_shape_lib );
    }
    else {
      printf("writing tiled json\n");
      fflush(stdout);
      write_tiled_json( bpc );
    }
  }

  //----
  //----
  //----

  printf("finishing\n"); fflush(stdout);

  if (name_fn) { free(name_fn); }
  if (rule_fn) { free(rule_fn); }
  if (constraint_fn) { free(constraint_fn); }

  printf("...\n"); fflush(stdout);

  return 0;
}
