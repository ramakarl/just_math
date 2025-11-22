//--------------------------------------------------------------------------------
// JUST MATH:
// Artificial Neural Network
//
// This example uses the NeuralNet class to train a simple
// network to learn real-valued 1D functions.
// Training instances are generated for f(x) = sin(x)
// with the learning process visualized interactively. 
//
// The interactive display loop does:
//    TrainInstance (10) - train the network with 10 more random instances
//    Evaluate () - evaluate the network over all plot points
//    drawLine..  - draw the current learned function on screen
//

//--------------------------------------------------------------------------------
// Copyright 2019-2023 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>
#include <time.h>

#include "main.h"        // window system 
#include "gxlib.h"      // rendering
using namespace glib;

#include "dataptr.h"

#include "neural_net.h"

#ifdef BUILD_OPENGL
  #include <GL/glew.h>
#endif
#ifdef BUILD_CUDA
  #include "common_cuda.h"
#endif

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
  
  void      TrainInstances ( int iter );
  double    Evaluate ();

  void      Restart();

  NeuralNet  ann;

  DataPtr   m_indata;
  DataPtr   m_outdata;
  int       m_numinst;
  double    m_error;
  
  Vec3F m_pnt[512];  
  int       m_numpnts;  
  int       m_frame; 

  int       mouse_down;  
  bool      m_run;
  bool      m_run_cuda;  
  clock_t   m_t1, m_t2;
};
Sample obj;

void Sample::on_arg(int i, std::string arg, std::string optarg )
{
    char dash = arg.at(0);
    char ch = arg.at(1);
    
    if ( dash=='-' ) {
      switch (ch) {      
        case 'd': break;
        case 'V': break;
      }
    }
}


void Sample::Restart()
{
}

void Sample::TrainInstances ( int iter )
{
    double* v_in = (double*) m_indata.getPtr(0);
    double* v_out = (double*) m_outdata.getPtr(0);

    // Train the network for I iterations
    for (int i=0; i < iter; i++) {

        // Generate a random training instance
        double x = double(rand())/RAND_MAX;
        *v_in = x;
        *v_out = sin(x*3.1415*2);
    
        // Feed into the network
        ann.FeedForward ( m_indata );

        // Backpropagate to learn the weights relative to expected output
        ann.Backprop ( m_outdata );
    }

    m_numinst += iter;
}

double Sample::Evaluate ()
{
    double* v_in = (double*) m_indata.getPtr(0);
    double* v_out = (double*) m_outdata.getPtr(0);
    double x, y;

    double e, error = 0;

    // Evaluate the network at unobserved, discrete sample points
    // corresponding to plot locations    
    for (int j=0; j < m_numpnts; j++) {

        // Create test point
        x = double(j) / m_numpnts;        

        // Feed point into network
        *v_in = x;
        ann.FeedForward ( m_indata );        
        
        // Retrieve output
        ann.Retrieve ( m_outdata );
        m_pnt[j].x = *v_out;

        // Measure error
        // MSE mean squared error = (o-y)^2
        y = sin(x*3.1415*2);
        e = (*v_out - y);
        error += e*e;  
    }
    return error;
}


bool Sample::init()
{
    addSearchPath(ASSET_PATH);

    #ifdef BUILD_CUDA   
      if ( m_run_cuda ) {
          CUcontext ctx; 
          CUdevice dev;
          cuStart ( DEV_FIRST, 0, dev, ctx, 0, true );    // start CUDA
      }
    #endif

    #ifdef BUILD_OPENGL
      init2D("arial");
      setTextSz (18,1);
    #endif

    // Clear plot pnts
    m_frame = 0;
    m_error = 0;
    m_numpnts = 256;
    for (int n=0; n < m_numpnts; n++) {
        m_pnt[n] = Vec3F(0,0,0);
    }

    // Create training data buffers
    m_numinst = 0;
    m_indata.Resize( sizeof(double), 1, 0, DT_CPU );
    m_outdata.Resize( sizeof(double), 1, 0, DT_CPU );

    // Build a network
    // M = # of fully connected inner layers
    // N = # of hidden units

    int M = 1;                    
    int N = 5; 

    ann.AddLayer ( 0, 1 );        // input layer: 1 unit
    ann.AddLayer ( 1, N );        // 1 in, N units wide
    for (int m=0; m < M; m++)
        ann.AddLayer ( N, N );    // N in, N units wide - fully connected
    ann.AddLayer ( N, 1 );        // out layer: N in, 1 out

    // Initialize network with random weights
    ann.Initialize ( 1.0, 0.3, 0.1 );

    ann.Print();

    m_frame = 0;

    m_run = true;

    return true;
}


void Sample::display()
{
    float md= 0.0;    
    char savename[256] = {'\0'};
    int w = getWidth(), h = getHeight();    

    if (m_run) {

        // Train network
        TrainInstances( 10 );

        // Retrieve output by evaluating network
        m_error = Evaluate();

        // Plot error vs. iteration
        if ( m_frame/4 < m_numpnts) 
            m_pnt[ m_frame/4 ].y = m_error;

        m_frame++;
    }

    // Console output    
    dbgprintf( "Training. # Instances: %d, Mean Sq. Err: %f\n", m_numinst, m_error);
    
    // Interactive rendering (opengl only)
    #ifdef BUILD_OPENGL
    clearGL();
    start2D( w, h );

        h /= 2;

        float y, yl = 0;
        float sz, x, dx = float(w) / m_numpnts;        

        // Plot target function
        x = 0;
        sz = h*0.8;
        for (int p=1; p < m_numpnts; p++) {                    
            y = sin(float(p)*3.1415*2/m_numpnts);
            drawLine ( Vec2F(x, h - y*sz), Vec2F(x-dx, h - yl*sz), Vec4F(0,.3,.3,1) );
            x += dx;
            yl = y;
        }

        // Plot network output points
        x = 0;
        sz = h*0.8;
        drawLine ( Vec2F(x, h), Vec2F(w, h), Vec4F(0,0,1,1) );
        for (int p=1; p < m_numpnts; p++) {                    
            drawLine ( Vec2F(x, h - m_pnt[p].x*sz), Vec2F(x-dx, h - m_pnt[p-1].x*sz), Vec4F(1,1,1,1) );
            x += dx;
        }

        // Plot error vs training time
        x = 0;
        sz = h/200;
        drawLine ( Vec2F(x, h), Vec2F(w, h), Vec4F(0,0,1,1) );
        for (int p=1; p < m_numpnts; p++) {                    
            drawLine ( Vec2F(x, h - m_pnt[p].y*sz), Vec2F(x-dx, h - m_pnt[p-1].y*sz), Vec4F(1,.5,0,1) );
            x += dx;
        }

    end2D();
    
    drawAll();                    // complete 2D rendering to OpenGL

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
  } break;

  case AppEnum::BUTTON_RIGHT: {
  } break;

  }
}


void Sample::mouse(AppEnum button, AppEnum state, int mods, int x, int y)
{  
  mouse_down = (state == AppEnum::BUTTON_PRESS) ? button : -1;    // Track when we are in a mouse drag
}

void Sample::mousewheel(int delta)
{  
}


void Sample::keyboard(int keycode, AppEnum action, int mods, int x, int y)
{
  if (action==AppEnum::BUTTON_RELEASE) return;

  switch (keycode) {
  case ' ':  m_run = !m_run;  break;
  };
}

void Sample::reshape(int w, int h)
{
  #ifdef BUILD_OPENGL
    glViewport(0, 0, w, h);
    setview2D(w, h);
  #endif

  /*m_cam->setSize( w, h );
  m_cam->setAspect(float(w) / float(h));
  m_cam->setOrbit(m_cam->getAng(), m_cam->getToPos(), m_cam->getOrbitDist(), m_cam->getDolly());
  m_cam->updateMatricies(); */

  appPostRedisplay();
}

void Sample::startup()
{
  int w = 800, h = 600;
  appStart("Artificial Neural Network", "Artificial Neural Network", w, h, 4, 2, 16, false);
}

void Sample::shutdown()
{
}





