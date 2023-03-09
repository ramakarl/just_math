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

  int       write_tiled_json ( ConstraintCollapse & cc );

  ConstraintCollapse cc;

  bool m_run_cc;
  int m_X, m_Y, m_Z, m_D;
  int64_t m_it;
  std::string   m_name_fn;
  std::string   m_rule_fn;
  std::string   m_tilemap_fn;
  std::string   m_constraint_fn;
  std::string   m_tileset_fn;
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
  
  int       m_tileset_width, m_tileset_height;
  int       m_tileset_stride_x, m_tileset_stride_y;

  int       mouse_down;  
  bool      m_run;
  bool      m_run_cuda;
  bool      m_save;
  float     m_frame;
  int       m_peak_iter;
  int       m_show;
 
};
Sample obj;

Sample::Sample()
{
    // default values
    m_name_fn = "";
    m_tilemap_fn = "out.json";
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
      case 'd':
       // debug_print = 1;
        break;        
      case 'N':
        m_name_fn = optarg;
        break;
      case 'R':
        m_rule_fn = optarg;
        break;
      case 'C':
        m_constraint_fn = optarg;
        break;
      case 'M':
        m_tilemap_fn = optarg;
        break;
      case 'Q':
        m_tileset_fn = optarg;
        break;
      case 'S':
        seed = strToI(optarg);
        cc.m_seed = seed;
        break;
      case 's':
        m_tileset_stride_x = strToI(optarg);
        m_tileset_stride_y = m_tileset_stride_x;
        break;
      case 'T':
        test_num = strToI(optarg);
        break;
      case 'D':
        m_D = strToI(optarg);
        m_X = m_Y = m_Z = m_D;
        break;
      case 'X':
        m_X = strToI(optarg);
        break;
      case 'Y':
        m_Y = strToI(optarg);
        break;
      case 'Z':
        m_Z = strToI(optarg);
        break;

      case 'W':
        wfc_flag = 1;
        break;    
    }
    }
}

void Sample::AllocVolume (int id, Vector3DI res, int chan)    // volume alloc
{
  uint64_t cnt = res.x*res.y*res.z;
  uint64_t sz = cnt * chan * sizeof(float);
  int flags = m_run_cuda ? (DT_CPU | DT_CUMEM) : DT_CPU;
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
  int64_t vtx;

  int sy, ey_inc;

  int tilecount = (int) cc.m_tile_name.size();
  tilecount--;

  //opt.tileset_width = ceil( sqrt( ((double)bpc.m_tile_name.size()) - 1.0 ) );
  int tileset_width = ceil( sqrt( (double) tilecount ) );
  int tileset_height = m_tileset_width;

  m_tileset_width *= m_tileset_stride_x;
  m_tileset_height *= m_tileset_stride_y;

  fp = fopen( m_tilemap_fn.c_str(), "w");
  if (!fp) { 
      printf ( "ERROR: Unable to create tilemap .json\n");
      return -1; 
  }

  fprintf(fp, "{\n");
  fprintf(fp, "  \"backgroundcolor\":\"#ffffff\",\n");
  fprintf(fp, "  \"height\": %i,\n", (int) cc.m_res.y);
  fprintf(fp, "  \"width\": %i,\n", (int) cc.m_res.x);
  fprintf(fp, "  \"layers\": [{\n");

  fprintf(fp, "    \"data\": [");

  // tiled expects y to increment in the negative direction
  // so we need to reverse the y direction when exporting
  //
  if ( 0 ) {
    // reversed y
    for (i=(int)( cc.m_res.y-1); i>=0; i--) {
      for (j=0; j<(int) cc.m_res.x; j++) {
        vtx = cc.getVertex(j, i, 0);
        fprintf(fp, " %i", (int) cc.getVal ( BUF_T, vtx ));
        if ((i==0) && (j==(cc.m_res.x-1))) { fprintf(fp, "%s",  ""); }
        else                                { fprintf(fp, "%s", ","); }
      }
      fprintf(fp, "\n  ");
    }

  } else {
    // standard y
    for (i=0; i<(int)( cc.m_res.y); i++) {
      for (j=0; j<(int) cc.m_res.x; j++) {
        vtx = cc.getVertex(j, i, 0);
        fprintf(fp, " %i", (int) cc.getVal ( BUF_T, vtx ));
        if ((i==(cc.m_res.y-1)) && (j==(cc.m_res.x-1))) { fprintf(fp, "%s",  ""); }
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
  fprintf(fp, "    \"width\": %i,\n", (int) cc.m_res.x);
  fprintf(fp, "    \"height\": %i,\n", (int) cc.m_res.y);
  fprintf(fp, "    \"x\":0,\n");
  fprintf(fp, "    \"y\":0\n");

  fprintf(fp, "  }\n");

  fprintf(fp, "  ],\n");
  fprintf(fp, "  \"nextobjectid\": %i,\n", 1);
  fprintf(fp, "  \"orientation\": \"%s\",\n", "orthogonal");
  fprintf(fp, "  \"properties\": [ ],\n");
  fprintf(fp, "  \"renderorder\": \"%s\",\n", "right-down");
  fprintf(fp, "  \"tileheight\": %i,\n", (int) m_tileset_stride_y);
  fprintf(fp, "  \"tilewidth\": %i,\n", (int) m_tileset_stride_x);
  fprintf(fp, "  \"tilesets\": [{\n");

  fprintf(fp, "    \"firstgid\": %i,\n", 1);
  fprintf(fp, "    \"columns\": %i,\n", (int) cc.m_res.x);
  fprintf(fp, "    \"name\": \"%s\",\n", "tileset");
  fprintf(fp, "    \"image\": \"%s\",\n", m_tileset_fn.c_str());
  fprintf(fp, "    \"imageheight\": %i,\n", (int) m_tileset_height);
  fprintf(fp, "    \"imagewidth\": %i,\n", (int) m_tileset_width);
  fprintf(fp, "    \"tilecount\": %i,\n", tilecount);
  fprintf(fp, "    \"tileheight\": %i,\n", (int) m_tileset_stride_y);
  fprintf(fp, "    \"tilewidth\": %i\n", (int) m_tileset_stride_x);

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

  m_run_cc = true;
  m_show = BUF_T;
  
  // Render volume  
  m_vres.Set ( m_X, m_Y, m_Z );         // match BP res
  AllocVolume ( BUF_VOL, m_vres, 4 );
  
  // App Options
  //
  m_frame     = 0;  
  m_run       = false;   // must start out false until all other init is done
  m_run_cuda  = false;  // run cuda pathway   
  m_save      = false;  // save to disk
  m_cam = new Camera3D;
  m_cam->setOrbit ( 30, 20, 0, m_vres/2.0f, 100, 1 );
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

  // Initiate Belief Propagation
  m_constraint_fn = "";
  std::string name_path, rule_path, constraint_path;

  getFileLocation ( m_name_fn, name_path );
  getFileLocation ( m_rule_fn, rule_path );
  getFileLocation ( m_constraint_fn, constraint_path );
 
  if (m_run_cc) {
      // init belief prop
      ret = cc.init (m_X, m_Y, m_Z, name_path, rule_path );
      if (ret<0) {
        fprintf(stderr, "bpc error loading CSV\n");
        exit(-1);
      }
      // constrain belief prop      
      if (!m_constraint_fn.empty()) {
        cc.read_constraints ( constraint_path );        
      }
      // start belief prop
      cc.start (); 
  }
  
  m_it = 0;

  m_run = false;

  return true;
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
     //m_run = false;  // single step
    
    if ( m_run_cc ) {

      for (int iter=0; iter < 10; iter++) {
          int cnt = cc.single_step();
          if (cnt==0) {
              // DONE!
              write_tiled_json ( cc );
              printf ( "DONE!\n");
              m_run = false;      
          }
      }
      cc.info();      

    }

    m_it++;
    fflush(stdout);
  }
  
  // Raycast  
  ClearImg (m_img);
  
  if ( m_run_cc ) {     
      Visualize ( cc, m_show, BUF_VOL );
      RaycastCPU ( m_cam, BUF_VOL, m_img, Vector3DF(0,0,0), Vector3DF(m_vres) );      // raycast volume
  }  

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
  case '.':  m_show++; if (m_show>3) m_show=0; break;
  case ',':  m_show--; if (m_show<0) m_show=3; break;
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





