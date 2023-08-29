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
#include "main.h"      // window system 
#include "nv_gui.h"      // gui system
#include "image.h"
#include "mersenne.h"
#include "camera3d.h"
#include "dataptr.h"
#include "geom_helper.h"
#include "string_helper.h"

#include "constraint_collapse.h"

#ifdef USE_OPENGL
  #include <GL/glew.h>
#endif
#ifdef USE_CUDA
  #include "common_cuda.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <vector>
#include <string>

#define BUF_VOL      0      // render volume

class Sample : public Application {
public:
  Sample();
  virtual void startup();
  virtual bool init();
  virtual void display();
  virtual void reshape(int w, int h);
  virtual void on_arg(int i, std::string arg, std::string val );
  virtual void motion(AppEnum button, int x, int y, int dx, int dy);
  virtual void keyboard(int keycode, AppEnum action, int mods, int x, int y);
  virtual void mouse(AppEnum button, AppEnum state, int mods, int x, int y);
  virtual void mousewheel(int delta);
  virtual void shutdown();
  
  void      Restart();

  int       experiments ( std::string outexpr, std::string outrun );
  int       write_tiled_json ( ConstraintCollapse & cc );

  ConstraintCollapse cc;

  std::vector< std::string > m_tile_name;
  std::vector< std::vector<float> > m_tile_rule;
  
  // Volume rendering
  void      Visualize ( ConstraintCollapse& src, int bp_id, int vol_id );
  
  void      AllocVolume(int id, Vector3DI res, int chan=1);
  float     getVoxel ( int id, int x, int y, int z );
  Vector4DF getVoxel4 ( int id, int x, int y, int z );
  void      ClearImg (Image* img);
  void      RaycastCPU ( Camera3D* cam, int id, Image* img, Vector3DF vmin, Vector3DF vmax );      
    
  Camera3D* m_cam;          // camera
  Image*    m_img;          // output image  
  Image*    m_img2;     
  Vector3DI m_vres;         // volume res
  DataPtr   m_vol[4];       // volume
  
  int       mouse_down;  
  int       m_vis;
  bool      m_run;  
  bool      m_save;
  float     m_frame;    
 
};
Sample obj;

Sample::Sample()
{ 
}

void Sample::on_arg(int i, std::string arg, std::string optarg )
{
    std::string name_fn, rule_fn, constraint_fn;    
    float valf;
    int vali;
    int wfc_flag = 0;
    int seed = 0;    
    int test_num = 0;
    char dash = arg.at(0);
    char ch = arg.at(1);
    
    if ( dash=='-' ) {
    switch (ch) {                 
      case 'S':
        seed = strToI(optarg);
        cc.op.seed = seed;
        break; 
      case 'd':
        cc.op.decay = strToF(optarg);
        break;
      case 'b':
        cc.op.border_r = strToF(optarg);
        break;
      case 'n':
        cc.op.noise_r = strToF(optarg);
        break;
      case 'f':
        cc.op.noise_flip = strToF(optarg);
        break;
      case 'D': {
        int D = strToI(optarg);
        cc.op.res.Set ( D, D, D );
        } break;
      case 'X':
        cc.op.res.x = strToI(optarg);
        break;
      case 'Y':
        cc.op.res.y = strToI(optarg);
        break;
      case 'Z':
        cc.op.res.z = strToI(optarg);
        break;

      case 'N':
        cc.op.name_fn = optarg;
        break;
      case 'R':
        cc.op.rule_fn = optarg;
        break;
      case 'C':
        cc.op.constraint_fn = optarg;
        break;
      case 'M':
        cc.op.tilemap_fn = optarg;
        break;
      case 'Q':
        cc.op.tileset_fn = optarg;
        break;

     case 's':
        cc.op.tileset_stride_x = strToI(optarg);
        cc.op.tileset_stride_y = cc.op.tileset_stride_x;
        break;
      case 'T':
        test_num = strToI(optarg);
        break;
    }
    }
}

void Sample::AllocVolume (int id, Vector3DI res, int chan)    // volume alloc
{
  uint64_t cnt = res.x*res.y*res.z;
  uint64_t sz = cnt * chan * sizeof(float);
  int flags = DT_CPU;
  m_vol[id].Resize( chan*sizeof(float), cnt, 0x0, flags );
  memset( (void *) (m_vol[id].getPtr(0)), 0, sz );
}
float Sample::getVoxel ( int id, int x, int y, int z )
{
  float* dat = (float*) m_vol[id].getPtr ( (z*m_vres.y + y)*m_vres.x + x );
  return *dat;
}
Vector4DF Sample::getVoxel4 ( int id, int x, int y, int z )
{
  int yinv = m_vres.y-1 - y;
  Vector4DF* dat = (Vector4DF*) m_vol[id].getPtr ( (z*m_vres.y + yinv)*m_vres.x + x );  
  return *dat;
}


void Sample::Visualize ( ConstraintCollapse& src, int bp_id, int vol_id ) {

   Vector4DF* vox = (Vector4DF*) m_vol[ vol_id ].getPtr (0);

   float maxv;
   float dmu;
   float scalar = 1.0;
   int t;

   // map belief to RGBA voxel
   for ( uint64_t j=0; j < src.getNumVerts(); j++ ) {    
     *vox = src.getSample ( bp_id, j );
     vox++;
   }
}

void Sample::ClearImg (Image* img)
{
    img->Fill ( 0 );
}

int Sample::write_tiled_json ( ConstraintCollapse & cc ) 
{
  FILE *fp;
  int i, j, n, tileset_size;
  int t;
  int64_t vtx;

  int sy, ey_inc;

  int tilecount = (int) cc.m_tile_name.size();
  tilecount--;

  //opt.tileset_width = ceil( sqrt( ((double)bpc.m_tile_name.size()) - 1.0 ) );
  cc.op.tileset_width = ceil( sqrt( (double) tilecount ) );
  cc.op.tileset_height = cc.op.tileset_width;

  cc.op.tileset_width *= cc.op.tileset_stride_x;
  cc.op.tileset_height *= cc.op.tileset_stride_y;

  char fname[1024];
  sprintf (fname, "%s_%03d_%03d.json", cc.op.tilemap_fn.c_str(), cc.op.res.x, cc.op.cur_run );

  fp = fopen( fname, "w");

  if (!fp) { 
      printf ( "ERROR: Unable to create tilemap .json\n");
      return -1; 
  }

  fprintf(fp, "{\n");
  fprintf(fp, "  \"backgroundcolor\":\"#ffffff\",\n");
  fprintf(fp, "  \"height\": %i,\n", (int) cc.op.res.y);
  fprintf(fp, "  \"width\": %i,\n", (int) cc.op.res.x);
  fprintf(fp, "  \"layers\": [{\n");

  fprintf(fp, "    \"data\": [");

  // tiled expects y to increment in the negative direction
  // so we need to reverse the y direction when exporting
  //
  if ( 0 ) {
    // reversed y
    for (i=(int)( cc.op.res.y-1); i>=0; i--) {
      for (j=0; j<(int) cc.op.res.x; j++) {

        vtx = cc.getVertex(j, i, 0);

        t = cc.getValI ( BUF_T, vtx );

        fprintf(fp, " %i", (int) t );
        if ((i==0) && (j==(cc.op.res.x-1))) { fprintf(fp, "%s",  ""); }
        else                                { fprintf(fp, "%s", ","); }
      }
      fprintf(fp, "\n  ");
    }

  } else {
    // standard y
    for (i=0; i<(int)( cc.op.res.y); i++) {
      for (j=0; j<(int) cc.op.res.x; j++) {
        
        vtx = cc.getVertex(j, i, 0);
        
        t = cc.getValI ( BUF_T, vtx );
        
        fprintf(fp, " %i", (int) t); 
        if ((i==(cc.op.res.y-1)) && (j==(cc.op.res.x-1))) { fprintf(fp, "%s",  ""); }
        else                                { fprintf(fp, "%s", ","); }
      }
      fprintf(fp, "\n  ");
    }
  }

  fprintf(fp, "\n    ],\n");
  fprintf(fp, "    \"name\":\"main\",\n");
  fprintf(fp, "    \"opacity\":1,\n");
  fprintf(fp, "    \"type\":\"tilelayer\",\n");
  fprintf(fp, "    \"visible\":true,\n");
  fprintf(fp, "    \"width\": %i,\n", (int) cc.op.res.x);
  fprintf(fp, "    \"height\": %i,\n", (int) cc.op.res.y);
  fprintf(fp, "    \"x\":0,\n");
  fprintf(fp, "    \"y\":0\n");

  fprintf(fp, "  }\n");

  fprintf(fp, "  ],\n");
  fprintf(fp, "  \"nextobjectid\": %i,\n", 1);
  fprintf(fp, "  \"orientation\": \"%s\",\n", "orthogonal");
  fprintf(fp, "  \"properties\": [ ],\n");
  fprintf(fp, "  \"renderorder\": \"%s\",\n", "right-down");
  fprintf(fp, "  \"tileheight\": %i,\n", (int) cc.op.tileset_stride_y);
  fprintf(fp, "  \"tilewidth\": %i,\n", (int) cc.op.tileset_stride_x);
  fprintf(fp, "  \"tilesets\": [{\n");

  fprintf(fp, "    \"firstgid\": %i,\n", 1);
  fprintf(fp, "    \"columns\": %i,\n", (int) cc.op.res.x);
  fprintf(fp, "    \"name\": \"%s\",\n", "tileset");
  fprintf(fp, "    \"image\": \"%s\",\n", cc.op.tileset_fn.c_str());
  fprintf(fp, "    \"imageheight\": %i,\n", (int) cc.op.tileset_height);
  fprintf(fp, "    \"imagewidth\": %i,\n", (int) cc.op.tileset_width);
  fprintf(fp, "    \"tilecount\": %i,\n", tilecount);
  fprintf(fp, "    \"tileheight\": %i,\n", (int) cc.op.tileset_stride_y);
  fprintf(fp, "    \"tilewidth\": %i\n", (int) cc.op.tileset_stride_x);

  fprintf(fp, "  }],\n");
  fprintf(fp, "  \"version\": %i\n", 1);
  fprintf(fp, "}\n");

  fclose(fp);

  return 0;
}

void Sample::RaycastCPU ( Camera3D* cam, int id, Image* img, Vector3DF vmin, Vector3DF vmax )
{
  Vector3DF rpos, rdir;
  Vector4DF clr;

  Vector3DF wp, dwp, p, dp, t;
  Vector3DF vdel = m_vres;
  Vector4DF val;
  int iter;
  float alpha, k;
  float pStep = 0.1;          // volume quality   - lower=better (0.01), higher=worse (0.1)
  float kDensity = 2.0;       // volume density   - lower=softer, higher=more opaque
  float kIntensity = 16.0;    // volume intensity - lower=darker, higher=brighter
  float kWidth = 4.0;         // transfer func    - lower=broader, higher=narrower (when sigmoid transfer enabled)

  int xres = img->GetWidth();
  int yres = img->GetHeight();
  
  // for each pixel in image..
  for (int y=0; y < yres; y++) {
    for (int x=0; x < xres; x++) {
      
      // get camera ray
      rpos = cam->getPos();
      rdir = cam->inverseRay ( x, y, xres, yres );  
      rdir.Normalize();

      // intersect with volume box
      clr.Set(0,0,0,0);
      float t;        
      
      if ( intersectLineBox ( rpos, rdir, vmin, vmax, t ) ) {

        // hit volume, start raycast...    
        wp = rpos + rdir * (t + pStep);                     // starting point in world space        
        dwp = (vmax-vmin) * rdir * pStep;                     // ray sample stepping in world space
        p = Vector3DF(m_vres) * (wp - vmin) / (vmax-vmin);    // starting point in volume        
        dp = rdir * pStep;                // step delta along ray
        
        // accumulate along ray
        for (iter=0; iter < 512 && clr.w < 0.99 && p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < m_vres.x && p.y < m_vres.y && p.z < m_vres.z; iter++) {
          val = getVoxel4 ( BUF_VOL, p.x, p.y, p.z );          // get voxel value
          //alpha = val.w;                        // opacity = linear transfer
          alpha = 1.0 / (1+exp(-(val.w-1.0)*kWidth));        // opacity = sigmoid transfer - accentuates boundaries at 0.5
          clr += Vector4DF(val.x,val.y,val.z, 0) * (1-clr.w) * alpha * kIntensity * pStep;  // accumulate color            
          clr.w += alpha * kDensity * pStep;              // attenuate alpha          
          p += dp;                           // next sample
        }  
        if (clr.x > 1.0) clr.x = 1;
        if (clr.y > 1.0) clr.y = 1;
        if (clr.z > 1.0) clr.z = 1;
        clr.x *= 255.0; clr.y *= 255.0; clr.z *= 255.0;

        img->SetPixel ( x, y, clr.x, clr.y, clr.z );
      }      
    }
  }

  #ifdef USE_OPENGL
    //commit image to OpenGL (hardware gl texture) for on-screen display
    img->Commit ( DT_GLTEX );      
  #endif
}

void Sample::Restart()
{
}


bool Sample::init()
{
  int i, ret;  

  addSearchPath(ASSET_PATH);

  m_vis = BUF_T;
  
  // Render volume  
  m_vres.Set ( cc.op.res.x, cc.op.res.y, cc.op.res.z );         // match BP res
  AllocVolume ( BUF_VOL, m_vres, 4 );
  
  // App Options
  //
  m_frame     = 0;  
  m_run       = false;   // must start out false until all other init is done
  m_save      = false;  // save to disk
  m_cam = new Camera3D;
  m_cam->setOrbit ( 30, 20, 0, m_vres/2.0f, 100, 1 );
  m_img = new Image;
  m_img->ResizeImage ( getWidth()/2, getHeight()/2, ImageOp::RGB8 );
  
  printf("Init done\n"); 
  fflush(stdout);
  #ifdef USE_OPENGL
    init2D("arial");
    setText(18,1);
  #endif

  #ifdef USE_CUDA   
    if ( m_run_cuda ) {
      CUcontext ctx; 
      CUdevice dev;
      cuStart ( DEV_FIRST, 0, dev, ctx, 0, true );    // start CUDA
    }
  #endif

  cc.expr.num_expr = 30;
  cc.expr.num_run = 40;
  cc.expr.grid_min.Set (6, 6,  1);
  cc.expr.grid_max.Set (36,36, 1);  

  experiments ( "expr.csv", "run.csv" );

  exit (-12);

  // initialize
  std::string name_path, rule_path;
  getFileLocation ( cc.op.name_fn, name_path );
  getFileLocation ( cc.op.rule_fn, rule_path );  
  ret = cc.init (cc.op.res.x, cc.op.res.y, cc.op.res.z, name_path, rule_path );
  if (ret<0) {
    fprintf(stderr, "bpc error loading CSV\n");
    exit(-1);
  }  
  // start belief prop
  cc.start ();   
 
  m_run = true;

  return true;
}

// Experiments
// - experiments consist of multiple runs over a change in state variables
// - the state variables are stored in bpc.expr struct
// - set the desired min/max state variables prior to calling this func
//
int Sample::experiments ( std::string outexpr, std::string outrun ) 
{
  int ret, runret;
  int run;
  std::string csv;

  // open experiment file  
  FILE* fpe = fopen ( outexpr.c_str(), "w" );
  if ( fpe==0 ) { printf ( "ERROR: Cannot open %s for output.\n", outexpr.c_str() ); exit(-7); }  
  fprintf (fpe,"reset\n");
  fclose (fpe);
  fpe = fopen ( outexpr.c_str(), "a" );   // append
  fprintf ( fpe, "gx, gy, gz, tiles, # runs, success, %%, fail_constr, total_time, success_time, fail_tile, max_tile, start seed\n" );

  // open run file  
  FILE* fpr = fopen ( outrun.c_str(), "w" );
  if ( fpr==0 ) { printf ( "ERROR: Cannot open %s for output.\n", outrun.c_str() ); exit(-7); }  
  fprintf (fpr,"reset\n");
  fclose (fpr);
  fpr = fopen ( outrun.c_str(), "a" );   // append  
  fprintf ( fpr, "status, run, max_run, step, gx,gy,gz, time, maxtime, time%%, constr, constr%%, temp, stuck, seed\n" );

  // platform-specific, find name & rule files
  std::string name_path, rule_path;
  #ifdef _WIN32
    getFileLocation ( cc.op.name_fn, name_path );
    getFileLocation ( cc.op.rule_fn, rule_path );
  #else
    name_path = bpc.op.name_fn;
    rule_path = bpc.op.rule_fn;
  #endif

  int num_experiments = cc.expr.num_expr;
  int num_runs = cc.expr.num_run;

  Vector3DF dgrid = Vector3DF(cc.expr.grid_max - cc.expr.grid_min) / num_experiments;  
  Vector3DF grid = cc.expr.grid_min;  

  int success = 0;
  int fail = 0;
  float fail_constr = 0;
  float total_time = 0;

  for (int e=0; e < num_experiments; e++) {

    // reset bp
    cc.reset ();

    // setup options
    cc.op.res = grid;    
    cc.op.max_run = cc.expr.num_run;
    cc.op.success_time = 0;
    cc.op.fail_time = 0;    

    // initialize
    std::string name_path, rule_path;
    getFileLocation ( cc.op.name_fn, name_path );
    getFileLocation ( cc.op.rule_fn, rule_path );  
     
    ret = cc.init (cc.op.res.x, cc.op.res.y, cc.op.res.z, name_path, rule_path );
    if (ret<0) { fprintf(stderr, "error loading CSV\n"); exit(-1); }  
  
    // start 
    cc.start ();   

    success = 0;
    fail = 0;

    // run    
    cc.op.cur_run = 0;    
    for ( run=0; run < cc.op.max_run; ) {

        ret = 1;
        for (int iter=0; iter < 20 && ret==1; iter++)
            ret = cc.step ();    

        // printf ( "%s\n", cc.getStatMsg().c_str() );

        if (ret==0 || ret==-1) {
            // DONE!
            write_tiled_json ( cc );  

            printf ( "  %s\n", cc.getStatMsg().c_str() );

            if (ret==0) {
                success++;
                cc.op.success_time += cc.op.elapsed_time;
            } else {
                fail++;
                fail_constr += cc.op.constrained_cnt;
                cc.op.fail_time += cc.op.elapsed_time;
            }
            total_time += cc.op.elapsed_time;

            cc.start();
            run++;

            csv = cc.getStatCSV();
            fprintf ( fpr, "%s\n", csv.c_str() );
            fclose ( fpr );
            fpr = fopen ( outrun.c_str(), "a" );            
        }          
    }    

    float ave_time = total_time / cc.op.max_run;
    cc.op.success_time /= success;
    cc.op.fail_time /= fail;
    fail_constr /= cc.op.max_run;

    //if ( bpc.op.verbose >= VB_EXPERIMENT ) {
      printf ( "GRID: %d,%d,%d, tiles:%d, runs:%d, success: %d (%4.1f%%), failconstr: %f, time: %f / %f / %f / %f, sseed: %d\n",
          (int) cc.op.res.x, (int) cc.op.res.y, (int) cc.op.res.z,
          (int) cc.m_num_values, 
          (int) cc.op.max_run, success,
          100*float(success)/cc.op.max_run,
          fail_constr,
          total_time/60.0, cc.op.success_time/60.0, cc.op.fail_time/60.0, cc.op.max_time/60.0, cc.op.seed );
    //}

    fprintf ( fpe, "%d,%d,%d, %d, %d, %d, %4.1f%%, %f, %f, %f, %f, %f, %d\n",
          (int) cc.op.res.x, (int) cc.op.res.y, (int) cc.op.res.z,
          (int) cc.m_num_values, 
          (int) cc.op.max_run, success, 100*float(success)/cc.op.max_run,
          fail_constr,
          total_time/60.0, cc.op.success_time/60.0, cc.op.fail_time/60.0, cc.op.max_time/60.0, cc.op.seed );
    
    fclose ( fpe );
    fpe = fopen ( outexpr.c_str(), "a" ); 

    // next experiment
    grid += dgrid;    
  }

  fclose ( fpe );
  fclose ( fpr );

  return 0;
}


void Sample::display()
{
  int ret;
  float md= 0.0;
  char savename[256] = {'\0'};
  char msg[256];
  Vector3DF a, b, c;
  Vector3DF p, q, d;

  // Run Belief Propagation  
  //
  if (m_run) {

    int ret = 1;
    for (int iter=0; iter < 10 && ret==1; iter++) {
        ret = cc.step();        
    }

    printf ( "%s\n", cc.getStatMsg().c_str() );

    if (ret==0 || ret==-1) {
        // DONE!
        write_tiled_json ( cc );
        
        cc.start();
    }
    
  }
  
  // Raycast  
  ClearImg (m_img);
  
  Visualize ( cc, m_vis, BUF_VOL );
  RaycastCPU ( m_cam, BUF_VOL, m_img, Vector3DF(0,0,0), Vector3DF(m_vres) );      // raycast volume  

  // optional write to disk
  if ( m_save ) {    
    sprintf ( savename, "out%04d.png", (int) m_frame );
    m_img->Save ( savename );        
    m_frame++;
  } else {
    sprintf ( savename, "save is off");
  }  
  
  // Interactive rendering (opengl only)
  #ifdef USE_OPENGL
    clearGL();
    start2D();
      setview2D(getWidth(), getHeight());  
      drawImg ( m_img->getGLID(), 0, 0, getWidth(), getHeight(), 1,1,1,1 );  // draw raycast image   
    end2D();
    draw2D();                    // complete 2D rendering to OpenGL

    // draw grid in 3D
    start3D(m_cam);
    setLight(S3D, 20, 100, 20);  
    for (int i=-10; i <= 10; i++ ) {
      drawLine3D( i, 0, -10, i, 0, 10, 1,1,1, .1);
      drawLine3D( -10, 0, i, 10, 0, i, 1,1,1, .1);
    }
    drawBox3D ( Vector3DF(0,0,0), m_vres, 1,1,1, 0.3);
    end3D();
    draw3D();                    // complete 3D rendering to OpenGL      
  #else
    //dbgprintf ( "Running.. saved: %s\n", savename);
    dbgprintf ( "Running..\n" );
  #endif

  appPostRedisplay();    
}

void Sample::motion(AppEnum btn, int x, int y, int dx, int dy)
{
  float fine = 0.5;

  switch (mouse_down) {
  case AppEnum::BUTTON_LEFT: {

    appPostRedisplay();  // Update display
  } break;

  case AppEnum::BUTTON_MIDDLE: {
    // Adjust target pos    
    m_cam->moveRelative(float(dx) * fine * m_cam->getOrbitDist() / 1000, float(-dy) * fine * m_cam->getOrbitDist() / 1000, 0);
    appPostRedisplay();  // Update display
  } break;

  case AppEnum::BUTTON_RIGHT: {

    // Adjust camera orbit 
    Vector3DF angs = m_cam->getAng();
    angs.x += dx * 0.2f * fine;
    angs.y -= dy * 0.2f * fine;
    m_cam->setOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());    
    appPostRedisplay();  // Update display
  } break;
  }
}


void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{
  if ( guiHandler(button, state, x, y) ) return;
  mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;    // Track when we are in a mouse drag
}

void Sample::mousewheel(int delta)
{
  // Adjust zoom
  float zoomamt = 1.0;
  float dist = m_cam->getOrbitDist();
  float dolly = m_cam->getDolly();
  float zoom = (dist - dolly) * 0.001f;
  dist -= delta * zoom * zoomamt;

  m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), dist, dolly);
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
  if (action==AppEnum::BUTTON_RELEASE) return;

  switch (keycode) {
  case 'w':  write_tiled_json ( cc ); break;
  case ' ':  m_run = !m_run;  break;
  case 'g':  printf("??\n"); fflush(stdout); Restart();  break;
  case '.':  m_vis++; if (m_vis>3) m_vis=0; break;
  case ',':  m_vis--; if (m_vis<0) m_vis=3; break;
  };
}

void Sample::reshape(int w, int h)
{
  #ifdef USE_OPENGL
    glViewport(0, 0, w, h);
    setview2D(w, h);
  #endif

  m_cam->setSize( w, h );
  m_cam->setAspect(float(w) / float(h));
  m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
  m_cam->updateMatricies();

  appPostRedisplay();
}

void Sample::startup()
{
  int w = 800, h = 600;
  appStart("Volume Raycast", "Volume Raycast", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}





