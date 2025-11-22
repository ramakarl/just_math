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
#include "gxlib.h"      // gui system
using namespace glib;

//#include "dataptr.h"
#include "geom_helper.h"
#include "string_helper.h"

#ifdef BUILD_OPENGL
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
    float           profile[65536];
};

struct Pass {
    int             tool_id;    
    int             num_depth_pass;
    int             max_depth_pass;
    Vec3F       pitch;
    Vec3F       cut_depth;    // x=current depth, z=depth per pass
    float           feedrate;    
};

struct Line {
    Vec3F       a, b;
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

  void SetMachine ( Vec3F mmin, Vec3F mmax );
  void SetWork ( float depth, Vec2F wrk_pos, Vec2F wrk_sz, Vec3F mtl_size );
  void SetSource ( std::string name );
  void AddTool ( std::string name, char tt, float width, float depth, std::string units );
  int  FindTool ( std::string name );
  void AddToolPass ( std::string tool, float feedrate, int max_dp );
  void CutToolPass (int p);
  float getCutHeight ( Vec3F pos, int tid, float accuracy, float work_minz, float work_maxz, float curr_depth );
  void AddCut ( Vec3F a, int tp, int dp );
  void AddMove ( Vec3F a, int tp, int dp );
  void StartGCode ( int pid );
  void EndGCode ( int pid );

  Vec3F getWorldPoint(Vec3F p);
  void AddLine ( Vec3F a, Vec3F b, char c, int tpass, int dpass );
  void DrawBox ( Vec3F a, Vec3F b, Vec4F clr );
  void DrawLine ( Vec3F p1, Vec3F p2, int c );
  
  Vec3F     m_machine_min;      // min travel of machine
  Vec3F     m_machine_max;      // max travel of machine

  Vec3F     m_material_size;    // material dimensions    
  Vec3F     m_work_min;         // work area
  Vec3F     m_work_max;         
  Vec3F     m_work_size;  

  std::string   m_img_name;         // source image name
  ImageX*       m_img_relief;       // image data
  Vec3F         m_pix_size;         // size of one pixel in world units
  Vec3I         m_res;
  
  float         m_detail;
  float         m_stepover;
  float         m_accuracy;
  float         m_pass_safety;

  float         m_max_depth;        // desired depth  
  float         m_pmin, m_pmax;     // img min/max values
  float         m_dmin, m_dmax;     // depth min/max values
  float         m_doffset;
  Vec3F     m_prev_pos;

  std::string   m_out_name;         // output name
  bool          m_out;
  FILE*         m_gfile;
  
  std::vector<Tool> m_Tools;
  std::vector<Pass> m_Passes;

  std::vector<Line> m_Lines[8][8];
  Vec4F     m_palette[16];

  Camera3D*     m_cam;    

  int           mouse_down;
  int           m_curr_tpass, m_curr_dpass;

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
  addSearchPath(ASSET_PATH);

  init2D("arial");
  setTextSz (18,1);

  m_extras = true;
  m_curr_tpass = 0;
  m_curr_dpass = 0;
  m_out = true;

  m_cam = new Camera3D;
  m_cam->setNearFar(1, 10000);  
  m_cam->SetOrbit ( Vec3F(160,50,0), Vec3F(0, 0, 0), 500, 1 );

  // Drawing colors   
  m_palette[ CLR_MOVE ] =   Vec4F(0, 1, 1, 0.5);   // cyan = relocate
  m_palette[ CLR_CUT ] =    Vec4F(1, 1, 1, 1);     // white = cut
  m_palette[ CLR_MACHINE ]= Vec4F(1, 0, 0, 1);     // red = machine limit
  m_palette[ CLR_MATL ] =   Vec4F(1,.5, 0, 1);     // orange = material limit
  m_palette[ CLR_WORK ] =   Vec4F(1, 1, 0, 1);     // yellow = working area
  m_palette[ CLR_GRID ] =   Vec4F(1, 1, 1, 0.3);   // gray = grid

    // Load tools
  AddTool ( "1/2 flat",     'f',  0.5,    0.5,    "in" );
  AddTool ( "1/4 sphere",   's',  0.25,   0.25,   "in" );
  AddTool ( "1/8 sphere",   's',  0.125,  0.125,  "in" );
  AddTool ( "4mm tapered",  's',  4,      4,      "mm" );          
  AddTool ( "0.5mm tapered", 's', 0.5,    0.5,    "mm" );  
  
  // 25.4 mm = 1"
  // 12.7 mm = 1/2"
  // 6.35 mm = 1/4"
  // 3.18 mm = 1/8"
  // 1.58 mm = 1/16"
  // 0.75 mm = 1/32"
  
  //AddTool ( "1/2 flat",   'f',   0.50,    1.0, "in" );  
  //AddTool ( "1/16 vbit",  'v', 1/16.0, 3/16.0, "in" );  
  //AddTool ( "1/8 sphere", 's', 0.125, 1.0, "in" );

  if (m_img_name.empty() ) {
    m_img_name = "dance_muses.png";
    dbgprintf ( "NOTE: Provide input depth image with argument -i {name}.\n");
    dbgprintf ( "  Using dance_muses.png for demo..\n");    
  }
  if (m_out_name.empty() ) {
    m_out_name = "dance_muses";
    dbgprintf ( "NOTE: Provide output gcode base name with -o {name}.\n");
    dbgprintf ( "  Using dance_muses.gcode for demo..\n");    
  }

  // Set source image
  SetSource ( m_img_name );

  // Set machine
  // min_range, max_range
  SetMachine ( Vec3F(-600, 0, -10 ), Vec3F(600, 2000, 50) );       // X left/right, Y back/fwd, Z up/down

  // Set work
  // cut_depth, work_pos, work_size, material_size (all in mm)
  SetWork ( 12, Vec2F(19, 19), Vec2F( 318.16, 146), Vec3F( 520, 177, 19 ) );
  
  // muses = 8172 x 3750 = 2.1792 asp, 318x146mm = 12.5 x 5 3/4" (7")

  // Detail settings
  m_detail =   0.05;        // x-resolution as % of tool width
  m_stepover = 0.20;        // y-pitch as % of tool width
  m_accuracy = 0.10;        // search accuracy as % of tool width
  m_pass_safety = 0.5;      // safe pass depth as % of the tool width. specific to machine, spindle and typical speed/feed rate

  dbgprintf ( "Detail:   %3.0f%%\n", m_detail * 100 );
  dbgprintf ( "Stepover: %3.0f%%\n", m_stepover * 100);
  dbgprintf ( "Accuracy: %3.0f%%\n", m_accuracy * 100);
  dbgprintf ( "Safety:   %3.0f%%\n", m_pass_safety * 100);

  // Add passes
  AddToolPass ( "1/4 sphere", 1000, 3 );
  AddToolPass ( "4mm tapered", 1000, 1 );
  
  // Create relief & g-code
  for (int n=0; n < m_Passes.size(); n++)
    CutToolPass ( n );
  
   // Adjust view
  m_curr_tpass = m_Passes.size()-1;

  Vec3F ctr = getWorldPoint ( (m_work_min + m_work_max) / 2.0f );
  m_cam->SetOrbit(Vec3F(160, 50, 0), ctr, 500, .01 );

  dbgprintf ( "Starting intactive mode..\n" );

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
        for (int n=0; n <= 65535; n++)
            t.profile[n] = 0;
        break;
    case 's':       // sphere bit
        for (int n=0; n <= 65535; n++) {
            x = float(n)/65535.0;
            t.profile[n] = 1.0 - sqrt( 1-(x*x) ) * t.width * 0.5;
        }
        break;
    case 'v':       // v-bit
        for (int n=0; n <= 65535; n++) {
            x = float(n)/65535.0;
            t.profile[n] = x * t.depth;         // CHECK IF CORRECT
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

void Sample::AddLine ( Vec3F a, Vec3F b, char c, int tpass, int dpass )
{
    Line l;
    l.a = a;
    l.b = b;
    l.c = (int) c;
    m_Lines[tpass][dpass].push_back ( l );
}

// convert from machine axes to world render axes
Vec3F Sample::getWorldPoint ( Vec3F p )
{
  return Vec3F(-p.x, p.z, p.y);
}

void Sample::DrawBox ( Vec3F p1, Vec3F p2, Vec4F clr )
{
    // swizzle and unit convert    
    drawBox3D ( getWorldPoint(p1), getWorldPoint(p2), clr );
}
void Sample::DrawLine ( Vec3F p1, Vec3F p2, int c )
{
    // swizzle and unit convert    
    Vec4F clr = m_palette[c];
    if (c==CLR_CUT) { clr =  Vec4F(.1,.1,.1,1) + Vec4F(.9,.9,.9,0) * (p1.z-m_dmin)/(m_dmax-m_dmin); clr.w=1; }
    drawLine3D ( getWorldPoint(p1), getWorldPoint(p2), clr );
}


void Sample::SetMachine ( Vec3F mmin, Vec3F mmax )
{
    m_machine_min = mmin;
    m_machine_max = mmax;
}
void Sample::SetWork ( float depth, Vec2F wrk_pos, Vec2F wrk_sz, Vec3F mtl_size )
{
    m_material_size  = mtl_size;    
    m_work_min = Vec3F(wrk_pos.x, wrk_pos.y, m_material_size.z - depth );
    m_work_max = Vec3F(wrk_pos.x+wrk_sz.x, wrk_pos.y+wrk_sz.y, m_material_size.z );
    
    m_work_size = m_work_max - m_work_min;
    float work_asp = m_work_size.y / m_work_size.x;
    float src_asp = float(m_res.y) / float(m_res.x);
    if ( src_asp < work_asp) {
        m_work_size.y = m_work_size.x * src_asp;
        m_work_max.y = m_work_min.y + m_work_size.y;
    } else {
        m_work_size.x = m_work_size.y / src_asp;
        m_work_max.x = m_work_min.x + m_work_size.x;
    }
    m_max_depth = depth;
    m_pix_size = m_work_size / m_res;

    m_doffset = -m_material_size.z;

    dbgprintf ( "Material size: %4.2f, %4.2f, %4.2f\n", m_material_size.x, m_material_size.y, m_material_size.z);
    dbgprintf ( "Work pos:      %4.2f, %4.2f, %4.2f\n", m_work_min.x, m_work_min.y, m_work_min.z);
    dbgprintf ( "Work size:     %4.2f, %4.2f, %4.2f\n", m_work_size.x, m_work_size.y, m_work_size.z);
    dbgprintf ( "Pixel size:    %4.5f, %4.5f\n\n", m_pix_size.x, m_pix_size.y);    
}

void Sample::SetSource ( std::string name )
{      
    // Load relief img
    // - support 16-bit TIFF    

    std::string imgpath;
    if ( !getFileLocation ( name, imgpath ) ) { dbgprintf ( "ERROR: Cannot find file %s\n", name.c_str() ); exit(-1); }
    m_img_relief = new ImageX;

    dbgprintf ( "\nLoading Depth Image\n");
    dbgprintf ( "  File: % s\n", imgpath.c_str() );

    if ( !m_img_relief->Load ( imgpath ) ) { dbgprintf ( "ERROR: Unable to load %s\n", name.c_str() ); exit(-1); }
    m_img_relief->Commit (DT_GLTEX);

    m_res = Vec3I(m_img_relief->GetWidth(), m_img_relief->GetHeight(), 1);

    // compute min/max pixel values
    float v;
    m_pmin = 1;
    m_pmax = 0;
    for (int x=0; x < m_res.x; x++) {
        for (int y=0; y < m_res.y; y++) {            
            v = m_img_relief->GetPixel (x,y).x;
            if ( v < m_pmin) m_pmin = v;
            if ( v > m_pmax) m_pmax = v;
        }
    }
    dbgprintf ( "  Res: %d x %d\n", m_res.x, m_res.y );
    dbgprintf ( "  Pixel min: %6.0f, max: %6.0f\n\n", m_pmin, m_pmax );
}


void Sample::AddToolPass ( std::string tname, float feedrate, int max_dp )
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
    p.cut_depth = Vec3F(0, m_max_depth, 0.5 * tool_wid);
    p.feedrate = feedrate;
    p.max_depth_pass = max_dp;

    m_Passes.push_back ( p );
}

float Sample::getCutHeight ( Vec3F pos, int tid, float accuracy, float work_minz, float work_maxz, float curr_depth )
{
    float f, T, h, d, r;
    Vec3F pix, o;

    // Search for Maximum safe height

    // Compute tool width in pixels
    Vec3I tool_px = Vec3F(m_Tools[tid].width, m_Tools[tid].width, 0) * 0.5f / m_pix_size;

    // Compute search accuracy
    int px_jump = accuracy / m_pix_size.x;
    px_jump = (px_jump < 1) ? 1 : (px_jump > tool_px.x) ? tool_px.x : px_jump;

    float hmax = -m_max_depth;
    
    // Get pixel for current pos
    pix.x =     (pos.x-m_work_min.x) * m_res.x / m_work_size.x;
    pix.y =     m_res.y - (pos.y-m_work_min.y) * m_res.y / m_work_size.y;

    // Scan tool extent region in image
    for (int j=-tool_px.x; j <= tool_px.x; j+= px_jump ) {
        for (int k=-tool_px.y; k <= tool_px.y; k+= px_jump  ) {

            if ( pix.x+j >= 0 && pix.x+j < m_res.x && pix.y+k >= 0 && pix.y+k < m_res.y ) {
    
                // check if inside tool radius
                o = Vec3F(j, k, 0) * m_pix_size;
                r = sqrt(o.x*o.x + o.y*o.y) / (m_Tools[tid].width * 0.5);
                if ( r < 1.0 ) {

                    // get artwork height and convert to real units
                    f = float( m_img_relief->GetPixel (pix.x + j, pix.y + k).x - m_pmin ) / (m_pmax-m_pmin);
                    f = f * m_max_depth;

                    // get radially symmetric tool profile                    
                    T = m_Tools[tid].profile[ int( r * 65535 ) ];
                
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

    // Height above work base
    d = work_minz + hmax;
    if (d < m_dmin) m_dmin = d;
    if (d > m_dmax) m_dmax = d;

    // Clip to current depth
    if ( d < work_maxz - curr_depth) 
       d = work_maxz - curr_depth;
    
    return d;
}

void Sample::AddCut ( Vec3F a, int tp, int dp )
{
    if ( m_out ) {
        if ( fabs(a.z-m_prev_pos.z) < 0.001 ) {
            fprintf ( m_gfile, "G01 X%4.2f\n", a.x );   // no z change
        } else {
            fprintf ( m_gfile, "G01 X%4.2f Z%4.3f\n", a.x, a.z+m_doffset );
        }
    }
    AddLine ( m_prev_pos, a, CLR_CUT, tp, dp ); 
    m_prev_pos = a;
}
void Sample::AddMove ( Vec3F a, int tp, int dp )
{
    if ( m_out) fprintf ( m_gfile, "G00 X%4.2f Y%4.2f Z%4.3f\n", a.x, a.y, a.z+m_doffset );
    AddLine ( m_prev_pos, a, CLR_MOVE, tp, dp );
    m_prev_pos = a;
}

void Sample::StartGCode ( int pid )
{
    if ( m_out) {

        // Start g-code file
        char fname[1024];    
        sprintf (fname, "%s_p%d.gcode", m_out_name.c_str(), pid );
        m_gfile = fopen ( fname, "wt" );

        // Set feed rate
        fprintf ( m_gfile, "F%4.2f\n", m_Passes[pid].feedrate );
    }
}

void Sample::EndGCode ( int pid )
{
    if ( m_out) {
        fclose ( m_gfile );
        dbgprintf ( "\nOutput file: %s_p%d.gcode, for Pass %d/%d\n\n", m_out_name.c_str(), pid, pid+1, m_Passes.size() );
    }
}

void Sample::CutToolPass (int pid)
{    
    Vec3F pl;
    Vec3F pos;
    float scanx;    

    Pass* p = &m_Passes[pid];
    int tp = pid;
    int tid = p->tool_id;
    float twidth = m_Tools[tid].width;

    dbgprintf ( "\nTool Pass: %d\n  Tool: %s, Total depth: %4.2f, Tool width: %4.2f, Safe depth: %4.2f \n", pid, m_Tools[tid].name.c_str(), m_max_depth, twidth, twidth * m_pass_safety);

    StartGCode ( pid );
       
    m_dmin = 1e10;
    m_dmax = -1e10;
    float work_hgt = m_work_max.z;
    float safe_hgt = work_hgt + 5;
    int dir = 1;

    // goto start of work
    m_prev_pos = Vec3F(0, 0, work_hgt);
    AddMove ( Vec3F(0, 0, safe_hgt), tp, 0 );
    AddMove ( Vec3F(m_work_min.x,m_work_min.y,safe_hgt), tp, 0 );
    AddMove ( Vec3F(m_work_min.x,m_work_min.y,work_hgt), tp, 0 );

    p->num_depth_pass = (m_max_depth / p->cut_depth.z)+1;
    if (p->num_depth_pass > p->max_depth_pass ) p->num_depth_pass = p->max_depth_pass;

    float pass_depth = float(m_max_depth) / p->num_depth_pass;
    float curr_depth = pass_depth;

    //-- test 
    /*FILE* fp = fopen ( "test.txt", "wt" );
    for ( pos.y = m_work_min.y; pos.y < m_work_max.y; pos.y += p->pitch.y ) { 
         for ( pos.x = m_work_min.x; pos.x <= m_work_max.x; pos.x += p->pitch.x ) {
            pos.z = getCutHeight ( pos, tid, p->pitch.z, m_work_min.z, m_work_max.z, curr_depth );
            fprintf ( fp, "%f %f %f\n", pos.x, pos.y, pos.z );
         }
    }
    fclose (fp);
    return; */
    
    for ( int dp = 0; dp < p->num_depth_pass; dp++ ) {
        
        dbgprintf ( "  Depth pass %d/%d, Depth: %4.2f\n", dp+1, p->num_depth_pass, curr_depth );

        // start depth pass
        pos.y = m_work_min.y;
        pos.x = (dir>0) ? m_work_min.x : m_work_max.x;                
        AddMove ( Vec3F(pos.x, pos.y, safe_hgt), tp, dp );     // safe height
    
        for ( pos.y = m_work_min.y; pos.y < m_work_max.y; pos.y += p->pitch.y ) {    

            pos.x = (dir>0) ? m_work_min.x : m_work_max.x;
            AddMove ( Vec3F(pos.x, pos.y, work_hgt), tp, dp );     // reposition            
        
            for ( scanx = 0; scanx <= m_work_size.x; scanx += p->pitch.x ) {

                pos.x = (dir>0) ? m_work_min.x + scanx : m_work_max.x - scanx;                
                pos.z = getCutHeight ( pos, tid, p->pitch.z, m_work_min.z, m_work_max.z, curr_depth );

                AddCut ( pos, tp, dp );      
            }

            AddMove ( Vec3F(pos.x, pos.y, work_hgt), tp, dp );

            dir = -dir;
        }
        AddMove ( Vec3F(pos.x, pos.y, safe_hgt), tp, dp );     // safe height

        curr_depth += pass_depth;
    }

    EndGCode ( pid );

}



void Sample::display()
{
    int w = getWidth(), h = getHeight();

    clearGL();
    start2D( w, h );    
        float asp = float(m_img_relief->GetHeight()) / m_img_relief->GetWidth();
        drawImg ( m_img_relief, Vec2F(0, 0), Vec2F(500, 500*asp), Vec4F(1,1,1,1) );
    end2D();
    

    start3D(m_cam);
    
        int p = m_curr_tpass;

        // Draw tool width        
        int t = m_Passes[p].tool_id;
        drawCircle3D ( Vec3F(0,0,0), Vec3F(0,1,0), m_Tools[t].width, Vec4F(0,0,1,1) );
    
        // Draw tool path        
        int num_dp = m_Passes[p].num_depth_pass;
        int d = m_curr_dpass;
        for (int n=0; n < m_Lines[p][d].size(); n++) {
            DrawLine ( m_Lines[p][d][n].a, m_Lines[p][d][n].b, m_Lines[p][d][n].c );
        }

        if ( m_extras ) {
            // Draw grid
            float i;
            for (i=m_machine_min.x; i <= m_machine_max.x; i += 50 ) DrawLine ( Vec3F( i, m_machine_min.y, 0), Vec3F(i, m_machine_max.y, 0), CLR_GRID );        
            for (i=m_machine_min.y; i <= m_machine_max.y; i += 50 ) DrawLine ( Vec3F( m_machine_min.x, i, 0), Vec3F(m_machine_max.x, i, 0), CLR_GRID );        
            // Draw machine
            DrawBox ( m_machine_min, m_machine_max, Vec4F(1,0,0,1) );
            // Draw material
            DrawBox ( Vec3F(0,0,0), m_material_size, Vec4F(1,0.5,0,1) );
            // Draw work
            DrawBox ( m_work_min, m_work_max, Vec4F(1,1,0,1) );
        }

    end3D();

    // render with opengl
    drawAll();

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
    Vec3F angs = m_cam->getAng();
    angs.x += dx * 0.2f * fine;
    angs.y -= dy * 0.2f * fine;
    m_cam->SetOrbit(angs, m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
    appPostRedisplay();  // Update display
  } break;
  }
}


void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{  
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
  case '1': m_curr_tpass = 0;    break;
  case '2': m_curr_tpass = 1;    break;
  case '3': m_curr_tpass = 2;    break;
  case '+': case '=': m_curr_dpass++; break;
  case '-': case '_': m_curr_dpass--; break;
  };
  if ( m_curr_tpass < 0 ) m_curr_tpass = 0;
  if ( m_curr_tpass >= m_Passes.size() ) m_curr_tpass = m_Passes.size()-1;
  if ( m_curr_dpass < 0 ) m_curr_dpass = 0;
  if ( m_curr_dpass >= m_Passes[m_curr_tpass].num_depth_pass ) m_curr_dpass = m_Passes[m_curr_tpass].num_depth_pass-1;  
}

void Sample::reshape(int w, int h)
{
  #ifdef BUILD_OPENGL
    glViewport(0, 0, w, h);
    setview2D(w, h);
  #endif

  if (m_cam==0) return;
  m_cam->setSize( w, h );
  m_cam->setAspect(float(w) / float(h));
  m_cam->SetOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
  
  appPostRedisplay();
}

void Sample::startup()
{
  int w = 1200, h = 700;

  m_img_name = "";
  m_out_name = "";

  appStart("GCoder", "GCoder", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}







