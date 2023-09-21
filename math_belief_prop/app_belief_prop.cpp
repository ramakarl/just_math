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

#include "belief_propagation.h"
#include "bp_helper.h"

#define DEBUG_GL        false

// #define USE_PERF       // explicit perf instrumentation (uncomment to opt in)

#ifdef USE_PERF
  #include "timex.h"      // app perf
#else
  #define PERF_PUSH(x)
  #define PERF_POP()
#endif

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


 std::vector< std::vector< float > >    tri_shape_lib;

class Sample : public Application {
public:
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

  // Algorithm Functions
  void      Restart ();
  void      RunAlgorithmInteractive ();
  void      Visualize ( BeliefPropagation& src, int vol_id );  
  
  BeliefPropagation bpc;

  // Tileset Icons (2D)
  void      GenerateTileImgs ( std::string tilefile, int tilesz );
  void      DrawTileSet ();  
  void      DrawTileMap ();

  // Volumetric (3D)
  void      AllocVolume(int id, Vector3DI res, int chan=1);
  float     getVoxel ( int id, int x, int y, int z );
  Vector4DF getVoxel4 ( int id, int x, int y, int z );
  void      ClearImg (Image* img);
  void      RaycastCPU ( Camera3D* cam, int id, Image* img, Vector3DF vmin, Vector3DF vmax );
  void      DrawGrid3D ();

  Camera3D* m_cam;          // camera
  Image*    m_img;          // output image
  Image*    m_img2;
  
  int       m_viz;
  Vector3DI m_vres;         // volume res
  DataPtr   m_vol[4];       // volume

  std::vector<Image*>   m_tile_imgs;

  // UI
  int       mouse_down;
  bool      m_run;  
  bool      m_save;
  bool      m_draw_tileset;
  float     m_scaling_2D;  
  float     m_frame;  

};
Sample obj;

void Sample::on_arg(int i, std::string arg, std::string optarg )
{
    float valf;
    int vali;
    int wfc_flag = 0;
    int seed = 0;
    int test_num = 0;
    char dash = arg.at(0);
    char ch = (arg.length()<=1) ? '0' : arg.at(1); 

    // get opt structure to load
    bp_opt_t* op = bpc.get_opt();

    if ( dash=='-' ) {
    switch (ch) {
      case 'd':
       // debug_print = 1;
        break;
      case 'V':
        op->verbose = strToI(optarg);
        break;
      case 'e':
        valf = strToF(optarg);
        if (valf > 0.0) {
          bpc.setConverge ( op, valf );
        }
        break;
      case 'z':
        valf = strToF(optarg);
        if (valf > 0.0) {
          op->eps_zero = valf;
        }
        break;
      case 'I':
        vali  = strToI(optarg);
        if (vali > 0) {
          op->max_step = (int64_t) vali;
        }
        break;
      case 'i':
        vali = strToI(optarg);
        if (vali > 0) {
          op->max_iter = (int64_t) vali;
        }
        break;
      case 'N':
        op->name_fn = optarg;
        break;
      case 'R':
        op->rule_fn = optarg;
        break;      

      case 'j':
        op->admissible_tile_range_cmd = optarg;
        break;
      case 'J':        
        op->constraint_cmd = optarg;
        break;
      case 'C':
        op->tilefilter_fn = optarg;        
        break;
      case 'L':
        op->tileobj_fn = optarg;
        break;
      case 'M':
        op->tilemap_fn = optarg;
        break;
      case 'Q':
        op->tileset_fn = optarg;
        break;
      case 's':
        op->tileset_stride_x = strToI(optarg);
        op->tileset_stride_y = op->tileset_stride_x;
        break;
      case 'G':     // algorithm selection
        op->alg_idx = strToI(optarg);
        break;
      case 'b':
        op->block_size[0] = strToI(optarg);
        op->block_size[1] = strToI(optarg);
        op->block_size[2] = strToI(optarg);

        op->sub_block_range[0][0] = 1;
        op->sub_block_range[1][0] = 1;
        op->sub_block_range[2][0] = 1;
        op->sub_block_range[0][1] = op->block_size[0];
        op->sub_block_range[1][1] = op->block_size[1];
        op->sub_block_range[2][1] = op->block_size[2];
        break;
      case 'S':
        seed = strToI(optarg);
        op->seed = seed;
        break;
      case 'r':
        op->max_run = strToI(optarg);
        break;
      case 'c':
        op->cull_list.push_back( strToI(optarg) );
        break;
      case 'T':
        test_num = strToI(optarg);
        break;

      case 'D':
        op->D = strToI(optarg);
        op->X = op->Y = op->Z = op->D;
        break;
      case 'X':
        op->X = strToI(optarg);
        break;
      case 'Y':
        op->Y = strToI(optarg);
        break;
      case 'Z':
        op->Z = strToI(optarg);
        break;

      case 'w': {
        float r = strToF(optarg);
        if (r > 0.0) {
          op->step_rate = r;
        }
        }break;
      
      case 'A':
        op->alg_accel = ALG_ACCEL_WAVE;
        break;

      case 'W':
        op->alg_accel = ALG_ACCEL_NONE;
        op->alg_run_opt = ALG_RUN_WFC;
        op->alg_cell_opt = ALG_CELL_WFC;
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
  Vector4DF* dat = (Vector4DF*) m_vol[id].getPtr ( (z*m_vres.y + y)*m_vres.x + x );
  return *dat;
}

void Sample::Visualize ( BeliefPropagation& src, int vol_id ) 
{
    // volume to write to
    Vector4DF* vox = (Vector4DF*) m_vol[ vol_id ].getPtr (0);
    Vector4DF clr;

    for ( uint64_t j=0; j < src.getNumVerts(); j++ ) {

        // map BP sample to RGBA voxel
        clr = src.getVisSample ( j );

        // write voxel
        *vox++ = clr;
   }
}

void Sample::ClearImg (Image* img)
{
    img->Fill ( 0 );
}

void Sample::RaycastCPU ( Camera3D* cam, int id, Image* img, Vector3DF vmin, Vector3DF vmax )
{
  Vector3DF rpos, rdir;
  Vector4DF clr;

  Vector3DF wp, dwp, p, dp, t;
  Vector3DF vdel = m_vres;
  Vector4DF val;
  int iter;
  float alpha;
  float pStep = 0.2;          // volume quality   - lower=better (0.01), higher=worse (0.1)
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
      float t;
      clr.Set(0,0,0,0);

      if ( intersectLineBox ( rpos, rdir, vmin, vmax, t ) ) {

        // hit volume, start raycast...
        wp = rpos + rdir * (t + pStep);                     // starting point in world space
        dwp = (vmax-vmin) * rdir * pStep;                     // ray sample stepping in world space
        p = Vector3DF(m_vres) * (wp - vmin) / (vmax-vmin);    // starting point in volume
        dp = rdir * pStep;                // step delta along ray

        // accumulate along ray
        for (iter=0; iter < 512 && clr.w < 0.99 && p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < m_vres.x && p.y < m_vres.y && p.z < m_vres.z; iter++) {
          val = getVoxel4 ( BUF_VOL, p.x, m_vres.y-p.y, p.z );          // get voxel value
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
    img->Commit ( DT_CPU | DT_GLTEX );
  #endif
}

void Sample::Restart ()
{
    // restart BP state
    bp_restart ( bpc ); 

    // make sure we're running
    m_run = false;
}


void Sample::GenerateTileImgs (std::string tilefile, int tilesz)
{
    Image* tileset;
    Image* icon;

    tileset = new Image();
    
    // load tileset image
    if ( tileset->Load ( tilefile ) ) {

        // divide tileset image into TX x TY tiles each of size 'tilesz' pixels
        int imgx = tileset->GetWidth();
        int imgy = tileset->GetHeight();
        int tiles = bpc.getNumValues(0);
        int TX = imgx/tilesz, TY = imgy/tilesz;
        int x = 0, y = 0;
        
        // process each tile ID
        for (int n=0; n < tiles; n++) {

            // create an image
            icon = new Image( tilesz, tilesz, ImageOp::RGB8 );

            // we assume the input and output are RGB8 formatted.
            // transfer a sub-block of pixels from input tileset to tile icon,
            // filling exactly the number of pixels in 'icon' (tilesz^2)            
            uchar* src_px = tileset->GetData ();
            uchar* dst_px = icon->GetData ();
            ImageOp::Format src_fmt = tileset->GetFormat();
            if (src_fmt != ImageOp::RGB8 && src_fmt != ImageOp::RGBA8 ) {
                printf ("ERROR: Tile set image format must be RGB8 or RGBA8.\n" );
                exit (-7);
            }            
            bool src_alpha = (tileset->GetFormat()==ImageOp::RGBA8);
            int src_bpp = tileset->GetBytesPerPixel ();
            src_px += ( (y * tilesz) * imgx + (x * tilesz) ) * src_bpp;
            for (int py=0; py < tilesz; py++) {
              for (int px=0; px < tilesz; px++) {
                // RGB transfer
                *dst_px++ = *src_px++;  
                *dst_px++ = *src_px++;
                *dst_px++ = *src_px++;     
                if (src_alpha) src_px++;
              }
              src_px += (imgx - tilesz) * src_bpp;
            }
            // commit to opengl
            icon->Commit ( DT_CPU | DT_GLTEX );

            // add to image list
            m_tile_imgs.push_back ( icon );
  
            // get next tile icon in tileset
            if (++x >= TX) {
                x = 0;
                if (++y >= TY ) {
                    printf ( "ERROR: Ran out of tiles. Should never happen.\n" );
                    exit(-8);
                }
            }
        }

    } else {
        printf ("ERROR: Unable to find tileset file %s\n", tilefile.c_str() );
    }

    delete tileset;
}

bool Sample::init()
{
  int ret;

  addSearchPath(ASSET_PATH);

  #ifdef USE_PERF
    PERF_INIT (64, true, false, true, 0, "" );
  #endif

  m_viz = VIZ_NONE;

  _bp_opt_t* op = bpc.get_opt();

  // Render volume
  // match resolution of BP settings
  m_vres.Set ( op->X, op->Y, op->Z );
  AllocVolume ( BUF_VOL, m_vres, 4 );

  // UI Options
  //
  m_frame     = 0;
  m_run       = false;  // must start out false until all other init is done
  m_save      = false;  // save to disk
  m_draw_tileset = false;
  m_scaling_2D = 2;

  m_cam = new Camera3D;
  m_cam->setNearFar ( 1, 2000 );
  m_cam->SetOrbit ( 30, 20, 0, m_vres/2.0f, 250, 1 );
  m_img = new Image;
  m_img->ResizeImage ( 256, 256, ImageOp::RGB8 );

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

    // pm_90:
    // -W 1 -r 1 -V 3 -I 50 -S 181 -e .0001 -X 90 -Y 90 -Z 1 -N pm_tilename.csv -R pm_tilerule.csv -Q pm_tileset.png -s 8 -M pm_tiles -J "d 0"

    // stairs:
    // -W 1 -r 1 -V 3 -I 50 -S 181 -e .0001 -X 10 -Y 10 -Z 10 -N stair_name.csv -R stair_rule.csv

  //-- Experiments  
  /* bpc.expr.num_expr = 20;
  bpc.expr.num_run = 50;
  bpc.expr.grid_min.Set (10, 10, 1);
  bpc.expr.grid_max.Set (210, 210, 1);
  bpc.expr.maxstep_min = 1;
  bpc.expr.maxstep_max = 1;
  bpc.expr.steprate_min = 0.98;
  bpc.expr.steprate_max = 0.98;
  bpc.expr.eps_min = .0001;
  bpc.expr.eps_max = .0001;
  bpc.st.instr = 0;

  bp_experiments ( bpc, "expr_pm.csv", "run_pm.csv" );
  exit(-6); */
    
  //-- Multirun testing  
  /* bp_multirun ( bpc, bpc.op.max_run, "run.csv" );
  
  exit(-5); */
    
  // Initiate Algorithm
  
  // find name & rule files
  std::string name_path, rule_path;
  getFileLocation ( op->name_fn, name_path );
  getFileLocation ( op->rule_fn, rule_path );

  // find contraint file
  bp_read_constraint_file ( bpc, op->tilefilter_fn, op->constraint_cmd );

  // initialize belief prop (using helper func)
  ret = bp_init_CSV ( bpc, op->X, op->Y, op->Z, name_path, rule_path );
  if (ret<0) {
     fprintf(stderr, "bpc error loading CSV\n");
     exit(-1);
  }

  // tileobj -> tri_shape_lib
  std::string obj_path;
  if (bpc.op.tileobj_fn.size() > 0) {
    getFileLocation ( op->tileobj_fn, obj_path );
    ret=load_obj_stl_lib( obj_path, tri_shape_lib );
    if (ret<0) {
      fprintf(stderr, "ERROR: when trying to load '%s' (load_obj_stl_lib)\n", bpc.op.tileobj_fn.c_str());
      exit(-1);
    }
  }
  
  // Generate tile images (2D only, if tileset is set)
  if ( bpc.op.Z==1 && bpc.op.tileset_fn.length()>0 ) {
     GenerateTileImgs ( bpc.op.tileset_fn, bpc.op.tileset_stride_x);
  }

  // Select algorithm
  bpc.SelectAlgorithm ( bpc.op.alg_idx );
  
  // Restart
  bp_restart ( bpc ); 

  // start viz
  m_viz = VIZ_TILES_2D;
  bpc.SetVis ( m_viz );

  // start running
  m_run = false;

  return true;
}

void Sample::DrawTileSet ()
{
    int scaling = 2;
    int spacing = 4;
    int tw = bpc.op.tileset_stride_x * scaling;
    int th = bpc.op.tileset_stride_y * scaling;
    int num_tiles = bpc.getNumValues(0);
    int tx, ty;

    start2D();
    
    // compute region where we draw them, to draw as overlay
    tx = 0; ty = 0;
    for (int n=0; n < num_tiles; n++) {
        tx += (tw + spacing);
        if (tx+tw > getWidth() ) {
            tx = 0; ty += (th + spacing);
        }
    }
    ty += (th + spacing);
    drawFill ( 0, 0, getWidth(), ty, .2,.2,.2, 1 );

    // draw tiles
    tx = 0; ty = 0;
    for (int n=0; n < num_tiles; n++) {
        drawImg ( m_tile_imgs[n]->getGLID(), tx, ty, tx+tw, ty+th, 1,1,1, 1 );    
        
        tx += (tw + spacing);
        if (tx+tw > getWidth() ) {
            tx = 0; ty += (th + spacing);
        }
    }

    end2D();
}


void Sample::DrawTileMap ()
{
    // Visualize must be called first to populate BUF_VOL
    // with literal tile values

    start2D();

    Vector4DF val;

    int tile;    
    int tw = bpc.op.tileset_stride_x * m_scaling_2D;
    int th = bpc.op.tileset_stride_y * m_scaling_2D;
    float num_tiles = bpc.getNumValues(0);
    float alpha;

    // draw fog box
    drawFill ( 0, 0, bpc.op.X*tw, bpc.op.Y*th, .8,.8,.8,1 );

    // draw tiles
    for (int y=0; y < bpc.op.Y; y++) {
        for (int x=0; x < bpc.op.X; x++) {        
            
            // get voxel value (2D) containing tile ID
            val = getVoxel4 ( BUF_VOL, x, y, 0 );   
            tile = val.x - 1;
            if ( tile >= 0 && tile < m_tile_imgs.size() ) {
                alpha = 1.0 / sqrt(val.w);      // uncertainty = tile count per cell
                drawImg ( m_tile_imgs[ tile ]->getGLID(), x*tw, y*th, (x+1)*tw, (y+1)*th, 1,1,1, alpha );            
            }
        }
    }
    
    end2D();
}


void Sample::DrawGrid3D ()
{
    setLight(S3D, 20, 100, 20);
    for (int i=-10; i <= 10; i++ ) {
      drawLine3D( i, 0, -10, i, 0, 10, 1,1,1, .1);
      drawLine3D( -10, 0, i, 10, 0, i, 1,1,1, .1);
    }
    drawBox3D ( Vector3DF(0,0,0), m_vres, 1,1,1, 0.3);
    end3D();
}

void Sample::RunAlgorithmInteractive ()
{

    PERF_PUSH("Step");
    int ret = bpc.RealizeStep ();
    PERF_POP();

    // check for step complete (0)
    if (ret <= 0) {

        // *NOTE*: right now RealizeStep ret error (<0) is ignored.

        // step complete
        // finish this iteration
        PERF_PUSH("Post");
        ret = bpc.RealizePost();
        PERF_POP();

        if ( ret > 0) {

            // iteration complete (all steps)
            // start new iteration
            PERF_PUSH("Pre");
            bpc.RealizePre();
            PERF_POP();
            
        } else if ( ret <= 0 ) {
             
            // write json output (failed or success)
            if (bpc.op.tileobj_fn.size() > 0) {
              bpc.op.outstl_fn = bpc.op.tilemap_fn;
              write_bp_stl( bpc, tri_shape_lib );
            } else {
              write_tiled_json( bpc );
            }
            // finish
            bpc.finish (ret);

            // stop
            m_run = false;
        }

    } 
    
    fflush(stdout);
}

void Sample::display()
{
  int ret;
  float md= 0.0;
  char savename[256] = {'\0'};

  Vector3DF a, b, c;
  Vector3DF p, q, d;

  void (*_cb_f)(void *) = NULL;

  //--------- Run Algorithm
  //
  if (m_run) {

    RunAlgorithmInteractive ();
    
  }

  //--------- Visualization

  // render cadence every 5 steps for perf
  if ( bpc.getStep() % 5 == 0) { 

      PERF_PUSH ("Render");

      // 3D visualize
      if ( m_viz >= VIZ_TILE0 ) {
          Vector3DF wfc_off(0,0,-10);
          Vector3DF bpc_off(0,0,0);

          // 3D raycast on CPU
          ClearImg (m_img);

          PERF_PUSH ("Visualize");
          Visualize ( bpc, BUF_VOL );
          PERF_POP ();

          PERF_PUSH ("RaycastCPU");
          RaycastCPU ( m_cam, BUF_VOL, m_img, bpc_off+Vector3DF(0,0,0), bpc_off+Vector3DF(m_vres) );      // raycast volume
          PERF_POP ();
  
          // OpenGL draw
          #ifdef USE_OPENGL
          clearGL();
          // Draw 3D raycast image (in 2D)
          start2D();
            setview2D(getWidth(), getHeight());
            drawImg ( m_img->getGLID(), 0, 0, getWidth(), getHeight(), 1,1,1,1 );  // draw raycast image
          end2D();
          // Draw 3D grid
          start3D(m_cam);
            DrawGrid3D ();
          end3D();
          #endif 
  
      } else {
      // 2D visualize
            
          #ifdef USE_OPENGL
     
          clearGL();      
          setview2D(getWidth(), getHeight());
      
          // Draw current 2D map
          // get tile values from algorithm
          PERF_PUSH ("Visualize");
          Visualize ( bpc, BUF_VOL );
          PERF_POP();

          PERF_PUSH ("DrawTileMap");
          DrawTileMap ();        
          PERF_POP();

          // Draw 2D tileset if requested
          if ( m_draw_tileset ) {
              DrawTileSet ();          
          }       

          #endif 
      }

      // Complete rendering
      draw2D();
      //draw3D();

      PERF_POP();
  }

  appPostRedisplay();
}



// optional write to disk
  /* if ( m_save ) {
    sprintf ( savename, "out%04d.png", (int) m_frame );
    m_img->Save ( savename );
    m_frame++;
  } else {
    sprintf ( savename, "save is off");
  } */


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
    m_cam->SetOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
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

  m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), dist, dolly);

  m_scaling_2D += (delta > 0) ? 0.1 : -0.1;
  if (m_scaling_2D < 0.1 ) m_scaling_2D = 0.1;
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
  if (action==AppEnum::BUTTON_RELEASE) return;

  switch (keycode) {
  case 'w':  

      write_tiled_json( bpc ); 
      break;

  case ' ':        
      m_run = !m_run;  
      write_tiled_json( bpc ); 
      break;
  
  case 'r':  
      // restart with same seed
      bpc.finish(-77);
      Restart ();   
      break;

   case 'g':  
      // regenerate with new seed
      bpc.finish(-77);   // -77 = stopped by user
      bpc.advance_seed();
      Restart (); 
      break;

  case 't':
      m_draw_tileset = !m_draw_tileset;
      break;

  case ',':  
      m_viz--; 
      if (m_viz < 1) m_viz = 5;
      bpc.SetVis ( m_viz );
      break;
  case '.':  
      m_viz++; 
      if (m_viz > 5) m_viz = 1;  
      bpc.SetVis ( m_viz );
      break;

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
  m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());  

  m_img->ResizeImage ( w/2, h/2, ImageOp::RGB8, DT_CPU | DT_GLTEX );  

  appPostRedisplay();
}

void Sample::startup()
{
  int w = 1400, h = 1300;
  appStart( "BMS / WFC / BP", "Breakout Model Synth", w, h, 4, 2, 16, DEBUG_GL);
}

void Sample::shutdown()
{
}





