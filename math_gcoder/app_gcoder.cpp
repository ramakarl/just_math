//--------------------------------------------------------------------------------
// JUST MATH:
// GCoder Generator
//
//
//
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
#include "dataptr.h"
#include "geom_helper.h"
#include "string_helper.h"

#ifdef USE_OPENGL
  #include <GL/glew.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <vector>
#include <string>

#define BUF_VOL      0      // render volume

#define CLR_MOVE      0
#define CLR_CUT       1
#define CLR_MACHINE   2
#define CLR_MATL      3
#define CLR_WORK      4
#define CLR_GRID      5

struct Tool {
    std::string     name;
    char            type;
    char            units;
    float           uconv;
    float           width;
    float           depth;
    float           profile[65535];
};

struct Pass {
    int             tool_id;    
    Vector3DF       pitch;
    Vector3DF       cut_depth;    // x=current depth, z=depth per pass
};

struct Line {
    Vector3DF       a, b;
    int             c;
};

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

  void SetMachine ( Vector3DF mmin, Vector3DF mmax );
  void SetWork ( float depth, Vector3DF mtl_size, float margin );
  void SetSource ( std::string name );
  void AddTool ( std::string name, char tt, float width, float depth, std::string units );
  int  FindTool ( std::string name );
  void AddPass ( std::string tool );
  void CutPass (int p);
  float getCutHeight ( Vector3DF pos, int tid, float accuracy );
  void AddCut ( Vector3DF a );
  void AddMove ( Vector3DF a );
  void StartGCode ( int pid );
  void EndGCode ();

  void AddLine ( Vector3DF a, Vector3DF b, char c, int pass );
  void DrawBox ( Vector3DF a, Vector3DF b, Vector4DF clr );
  void DrawLine ( Vector3DF p1, Vector3DF p2, int c );
  
  Vector3DF     m_machine_min;      // min travel of machine
  Vector3DF     m_machine_max;      // max travel of machine

  Vector3DF     m_material_size;    // material dimensions
  Vector3DF     m_margin;           // margin on material  
  Vector3DF     m_work_pos;         // work position
  Vector3DF     m_work_min;         // work area
  Vector3DF     m_work_max;         
  Vector3DF     m_work_size;  

  std::string   m_img_name;         // source image name
  Image*        m_img_relief;       // image data
  Vector3DF     m_pix_size;         // size of one pixel in world units
  Vector3DI     m_res;
  
  float         m_detail;
  float         m_stepover;
  float         m_accuracy;

  float         m_max_depth;        // desired depth  
  float         m_pmin, m_pmax;     // img min/max values
  float         m_dmin, m_dmax;     // depth min/max values
  float         m_doffset;
  Vector3DF     m_prev_pos;

  std::string   m_out_name;         // output name
  bool          m_out;
  FILE*         m_gfile;
  
  std::vector<Tool> m_Tools;
  std::vector<Pass> m_Passes;

  std::vector<Line> m_Lines[8];
  Vector4DF     m_palette[16];

  Camera3D*     m_cam;    

  int           mouse_down;
  int           m_curr_pass;

  bool          m_extras;
  bool          m_run;  
  bool          m_save;
  float         m_frame;  

};
Sample obj;

void Sample::on_arg(int i, std::string arg, std::string optarg )
{
    char dash = arg.at(0);
    char ch = (arg.length()<=1) ? '0' : arg.at(1); 

    if (!dash) return;

    switch (ch) {
    case 'i':
        m_img_name = optarg;
        break;
    case 'o':
        m_out_name = optarg;
        break;
    };

}

bool Sample::init()
{
  int ret;

  addSearchPath(ASSET_PATH);

  init2D("arial");
  setText(18,1);

  m_extras = true;
  m_curr_pass = 0;
  m_out = (m_out_name.size() > 0);

  m_cam = new Camera3D;
  m_cam->setNearFar(1, 10000);
  m_cam->SetOrbit ( Vector3DF(160,30,0), Vector3DF(0,0,0), 2000, 1 );

  // Drawing colors   
  m_palette[ CLR_MOVE ] =   Vector4DF(0, 1, 1, 0.5);   // cyan = relocate
  m_palette[ CLR_CUT ] =    Vector4DF(1, 1, 1, 1);     // white = cut
  m_palette[ CLR_MACHINE ]= Vector4DF(1, 0, 0, 1);     // red = machine limit
  m_palette[ CLR_MATL ] =   Vector4DF(1,.5, 0, 1);     // orange = material limit
  m_palette[ CLR_WORK ] =   Vector4DF(1, 1, 0, 1);     // yellow = working area
  m_palette[ CLR_GRID ] =   Vector4DF(1, 1, 1, 0.3);   // gray = grid


  // Load tools
  AddTool ( "1/2 flat",   'f',   0.50,    1.0, "in" );
  AddTool ( "1/8 sphere", 's',  0.125,  0.125, "in" );
  AddTool ( "1/16 vbit",  'v', 1/16.0, 3/16.0, "in" );
  
  //AddTool ( "1/8 sphere", 's', 0.125, 1.0, "in" );

  // Set source image
  SetSource ( m_img_name );

  // Set machine
  SetMachine ( Vector3DF(-600, 0, -10 ), Vector3DF(600, 2000, 50) );       // X left/right, Y back/fwd, Z up/down

  // Set work
  SetWork ( 12, Vector3DF( 400, 300, 26), 26 );         // depth=12 mm, 300x200x26 mm = 12 x 7.8 x 1", margin=1"
  
  // Add passes
  m_detail =   0.10;        // x-resolution as % of tool width
  m_stepover = 0.30;        // y-pitch as % of tool width
  m_accuracy = 0.20;        // search accuracy as % of tool width

  AddPass ( "1/2 flat" );
  AddPass ( "1/8 sphere" );
  AddPass ( "1/16 vbit" );

  // Create relief
  for (int n=0; n < m_Passes.size(); n++)
    CutPass ( n );  

  return true;
}


void Sample::AddTool ( std::string name, char tt, float width, float depth, std::string units )
{
    float x;

    Tool t;
    t.name = name;
    t.type = tt;
    t.units = (units.at(0)=='m' ) ? 'm' : 'i';
    t.uconv = (t.units=='m') ? 1 : 25.4;
    t.width = width * t.uconv;
    t.depth = depth * t.uconv;

    switch ( tt ) {
    case 'f':       // flat bit
        for (int n=0; n < 65535; n++)
            t.profile[n] = 0;
        break;
    case 's':       // sphere bit
        for (int n=0; n < 65535; n++) {
            x = float(n)/65535.0;
            t.profile[n] = sqrt(x*x) * t.width;
        }
        break;
    case 'v':       // v-bit
        for (int n=0; n < 65535; n++) {
            x = float(n)/65535.0;
            t.profile[n] = x * t.depth;
        }
        break;
    }

    m_Tools.push_back ( t );
}

int Sample::FindTool ( std::string name )
{
    for (int n=0; n < m_Tools.size(); n++) {
        if (m_Tools[n].name.compare (name)==0) return n;
    }
    return -1;
}

void Sample::AddLine ( Vector3DF a, Vector3DF b, char c, int pass )
{
    Line l;
    l.a = a;
    l.b = b;
    l.c = (int) c;
    m_Lines[pass].push_back ( l );
}

void Sample::DrawBox ( Vector3DF p1, Vector3DF p2, Vector4DF clr )
{
    // swizzle and unit convert
    Vector3DF a = Vector3DF(-p1.x, p1.z, p1.y);
    Vector3DF b = Vector3DF(-p2.x, p2.z, p2.y);
    drawBox3D ( a, b, clr.x,clr.y,clr.z,clr.w );
}
void Sample::DrawLine ( Vector3DF p1, Vector3DF p2, int c )
{
    // swizzle and unit convert
    Vector3DF a = Vector3DF(-p1.x, p1.z, p1.y);
    Vector3DF b = Vector3DF(-p2.x, p2.z, p2.y);
    Vector4DF clr = m_palette[c];
    if (c==CLR_CUT) { clr = Vector4DF(1,1,1,1) * (p1.z-m_dmin)/(m_dmax-m_dmin); clr.w=1; }
    drawLine3D ( a, b, clr );
}


void Sample::SetMachine ( Vector3DF mmin, Vector3DF mmax )
{
    m_machine_min = mmin;
    m_machine_max = mmax;
}
void Sample::SetWork ( float depth, Vector3DF mtl_size, float margin )
{
    m_material_size  = mtl_size;
    m_margin = Vector3DF(margin, margin, 0);
    m_work_min = m_margin;
    m_work_max = m_material_size - m_margin;
    m_work_min.z = m_material_size.z - depth;
    m_work_max.z = m_material_size.z;

    m_work_size = m_work_max - m_work_min;
    float work_asp = m_work_size.y / m_work_size.x;
    float src_asp = float(m_res.y) / float(m_res.x);
    if ( src_asp < work_asp) {
        m_work_size.y = m_work_size.x * src_asp;
        m_work_max.y = m_work_min.y + m_work_size.y;
    }

    m_max_depth = depth;
    m_pix_size = m_work_size / m_res;

    m_doffset = -m_material_size.z;
}

void Sample::SetSource ( std::string name )
{      
    // Load relief img
    // - support 16-bit TIFF
    #ifndef BUILD_TIFF
        dbgprintf ( "ERROR: BUILD_TIFF not compiled.\n");
        exit(-1);
    #endif  
    std::string imgpath;
    if ( !getFileLocation ( name, imgpath ) ) { dbgprintf ( "ERROR: Cannot find file %s\n", name.c_str() ); exit(-1); }
    m_img_relief = new Image;
    if ( !m_img_relief->Load ( imgpath ) ) { dbgprintf ( "ERROR: Unable to load %s\n", name.c_str() ); exit(-1); }
    m_img_relief->Commit (DT_GLTEX);

    m_res = Vector3DI(m_img_relief->GetWidth(), m_img_relief->GetHeight(), 1);

    // compute min/max pixel values
    float v;
    m_pmin = 65536;
    m_pmax = 0;
    for (int x=0; x < m_res.x; x++) {
        for (int y=0; y < m_res.y; y++) {            
            v = m_img_relief->GetPixel16 (x,y);
            if ( v < m_pmin) m_pmin = v;
            if ( v > m_pmax) m_pmax = v;
        }
    }
    dbgprintf ( "Pixel min: %6.0f, max: %6.0f\n", m_pmin, m_pmax );

}


void Sample::AddPass ( std::string tname )
{
    Pass p;
    p.tool_id = FindTool ( tname );
    if ( p.tool_id==-1 ) {
        dbgprintf ( "ERROR: Tool '%s' not found.\n", tname.c_str());
        exit (-2);
    }
    
    float tool_wid = m_Tools[p.tool_id].width;
    p.pitch.x = tool_wid * m_detail;
    p.pitch.y = tool_wid * m_stepover;    
    p.pitch.z = tool_wid * m_accuracy;
    p.cut_depth = Vector3DF(0, m_max_depth, 0.5 * tool_wid);

    m_Passes.push_back ( p );
}

float Sample::getCutHeight ( Vector3DF pos, int tid, float accuracy )
{
    float f, T, h, d, r;
    Vector3DF pix, o;

    // Search for Maximum safe height

    // Compute tool width in pixels
    Vector3DI tool_px = Vector3DF(m_Tools[tid].width, m_Tools[tid].width, 0) / m_pix_size;

    // Compute search accuracy
    int px_jump = accuracy / m_pix_size.x;
    px_jump = (px_jump < 1) ? 1 : (px_jump > tool_px.x) ? tool_px.x : px_jump;

    float hmax = -m_max_depth;
    
    pix.x = (pos.x-m_work_min.x) * m_res.x / m_work_size.x;
    pix.y = m_res.y - (pos.y-m_work_min.y) * m_res.y / m_work_size.y;

    // Scan over tool extents
    for (int j=-tool_px.x; j <= tool_px.x; j+= 10 ) {
        for (int k=-tool_px.y; k <= tool_px.y; k+= 10 ) {
            if ( pix.x+j >= 0 && pix.x+j < m_res.x && pix.y+k >= 0 && pix.y+k < m_res.y ) {
    
                // check if inside tool radius
                o = Vector3DF(j, k, 0) * m_pix_size;
                r = sqrt(o.x*o.x + o.y*o.y) / m_Tools[tid].width;
                if ( r < 1 ) {

                    // get artwork height and convert to real units
                    f = float( m_img_relief->GetPixel16 (pix.x + j, pix.y + k) - m_pmin ) / (m_pmax-m_pmin);
                    f = f*m_max_depth;

                    // get radially symmetric tool profile                    
                    T = m_Tools[tid].profile[ int(r*65535) ];
                
                    // compute tool height at this point
                    // h(x,y) = f(x+j, y+k) - T(j,k)
                    h = f - T;        

                    // only accept maximum height over extents
                    // tool can only go as high as this point or it would cut away artwork
                    if ( h > hmax ) hmax = h;
                }
            }
        }
    }

    

    // simple cut depth - assumes 0 tool width
    /* pix.x = (pos.x-m_work_min.x) * m_res.x / m_work_size.x;
    pix.y = m_res.y - (pos.y-m_work_min.y) * m_res.y / m_work_size.y;
    
        v = float( m_img_relief->GetPixel16 (pix.x, pix.y) - m_pmin ) / (m_pmax-m_pmin);                    
        d = (1.0-v) * m_max_depth;        
    } else {
        v = 0;
        d = 0;
    } */
    
    return hmax;
}

void Sample::AddCut ( Vector3DF a )
{
    if ( m_out ) {
        if ( fabs(a.z-m_prev_pos.z) < 0.01 ) {
            fprintf ( m_gfile, "G01 X%3.1f\n", a.x );   // no z change
        } else {
            fprintf ( m_gfile, "G01 X%3.1f Z%3.2f\n", a.x, a.z+m_doffset );
        }
    }
    AddLine ( m_prev_pos, a, CLR_CUT, m_curr_pass ); 
    m_prev_pos = a;
}
void Sample::AddMove ( Vector3DF a )
{
    if ( m_out) fprintf ( m_gfile, "G00 X%3.2f Y%3.2f Z%3.2f\n", a.x, a.y, a.z+m_doffset );
    AddLine ( m_prev_pos, a, CLR_MOVE, m_curr_pass );
    m_prev_pos = a;
}

void Sample::StartGCode ( int pid )
{
    if ( m_out) {
        char fname[1024];    
        sprintf (fname, "%s_p%d.gcode", m_out_name.c_str(), pid );
        m_gfile = fopen ( fname, "wt" );
    }
}

void Sample::EndGCode ()
{
    if ( m_out) 
        fclose ( m_gfile );
}

void Sample::CutPass (int pid)
{    
    Vector3DF pl;
    Vector3DF pos;
    float scanx;
    float v, d;

    StartGCode ( pid );

    dbgprintf ( "Cutting pass %d\n", pid );

    Pass p = m_Passes[pid];
    m_curr_pass = pid;
   
    m_dmin = 1e10;
    m_dmax = -1e10;
    float work_hgt = m_work_max.z;
    float safe_hgt = work_hgt + 5;
    int dir = 1;

    // goto start of work
    m_prev_pos = Vector3DF(0,0,work_hgt);
    AddMove ( Vector3DF(0,0,safe_hgt) );
    AddMove ( Vector3DF(m_work_min.x,m_work_min.y,safe_hgt) );
    AddMove ( Vector3DF(m_work_min.x,m_work_min.y,work_hgt) );

    p.cut_depth.x = m_max_depth;

    //for ( float cd = 0; cd < m_max_depth; cd += p.cut_depth.z ) {
    //    p.cut_depth.x = cd;

        for ( pos.y = m_work_min.y; pos.y < m_work_max.y; pos.y += p.pitch.y ) {    

            pos.x = (dir>0) ? m_work_min.x : m_work_max.x;
            AddMove ( Vector3DF(pos.x, pos.y, safe_hgt) );     // reposition         
            AddMove ( Vector3DF(pos.x, pos.y, work_hgt) );     // lower down
        
            for ( scanx = 0; scanx <= m_work_size.x; scanx += p.pitch.x ) {

                pos.x = (dir>0) ? m_work_min.x + scanx : m_work_max.x - scanx;
            
                d = m_work_min.z + getCutHeight ( pos, p.tool_id, p.pitch.z );
                if (d < m_dmin) m_dmin = d;
                if (d > m_dmax) m_dmax = d;

                if ( d < m_work_max.z - p.cut_depth.x) 
                    d = m_work_max.z - p.cut_depth.x;

                pos.z = d;

                AddCut ( pos );            
            }

            AddMove ( Vector3DF(pos.x, pos.y, safe_hgt) );     // raise up

            dir = -dir;
        }
    //}

    EndGCode ();
}



void Sample::display()
{
    clearGL();
    start2D();
        setview2D( getWidth(), getHeight() );
        float asp = float(m_img_relief->GetHeight()) / m_img_relief->GetWidth();
        drawImg ( m_img_relief->getGLID(), 0, 0, 500, 500*asp, 1,1,1,1 );
    end2D();
    

    start3D(m_cam);
    
        int p = m_curr_pass;

        // Draw tool width        
        int t = m_Passes[p].tool_id;
        drawCircle3D ( Vector3DF(0,0,0), Vector3DF(0,1,0), m_Tools[t].width, Vector4DF(0,0,1,1) );
    
        // Draw tool path        
        for (int n=0; n < m_Lines[p].size(); n++) {
            DrawLine ( m_Lines[p][n].a, m_Lines[p][n].b, m_Lines[p][n].c );
        }

        if ( m_extras ) {
            // Draw grid
            float i;
            for (i=m_machine_min.x; i <= m_machine_max.x; i += 50 ) DrawLine ( Vector3DF( i, m_machine_min.y, 0), Vector3DF(i, m_machine_max.y, 0), CLR_GRID );        
            for (i=m_machine_min.y; i <= m_machine_max.y; i += 50 ) DrawLine ( Vector3DF( m_machine_min.x, i, 0), Vector3DF(m_machine_max.x, i, 0), CLR_GRID );        
            // Draw machine
            DrawBox ( m_machine_min, m_machine_max, Vector4DF(1,0,0,1) );
            // Draw material
            DrawBox ( Vector3DF(0,0,0), m_material_size, Vector4DF(1,0.5,0,1) );
            // Draw work
            DrawBox ( m_work_min, m_work_max, Vector4DF(1,1,0,1) );
        }

    end3D();


    draw3D();                    // complete 3D rendering to OpenGL
    draw2D();
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
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
  if (action==AppEnum::BUTTON_RELEASE) return;

  switch (keycode) {
  case '`': m_extras = !m_extras; break;  
  case '1': m_curr_pass = 0;    break;
  case '2': m_curr_pass = 1;    break;
  case '3': m_curr_pass = 2;    break;
  };
  if ( m_curr_pass < 0 ) m_curr_pass = 0;
  if ( m_curr_pass >= m_Passes.size() ) m_curr_pass = m_Passes.size()-1;
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
  
  appPostRedisplay();
}

void Sample::startup()
{
  int w = 1900, h = 1000;

  m_img_name = "";
  m_out_name = "";

  appStart("GCoder", "GCoder", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}






