
#ifdef USE_NETWORK
    #include "network_system.h"
#endif

#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include "nv_gui.h"
#include "main.h"

// console app includes 
#ifdef WIN32
    #include <windows.h>
    #include <time.h>           // for clock() 
#else
    #include <unistd.h>     // linux
#endif

// Global singletons
Application* pApp = 0x0;

//------------------------------------------------ CONSOLE MAIN

int main(int argc, char** argv)
{
    #ifdef WIN32 
        //-- Debugging modes
        _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF);
        _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_WNDW);
    #endif

    dbgprintf ("Starting.\n" );
    pApp->startup();                        //-- App startup
    
    pApp->appHandleArgs( argc, argv );    //-- App handles args
    
    pApp->m_running = true;  

    dbgprintf ("Running..\n" );
    while (pApp->m_running) {

        pApp->appRun();     //-- Run app

        if (pApp->getKeyPress(KEY_ESCAPE)) {            //-- ESC key
            // Post close message for proper shutdown
            dbgprintf("ESC pressed.\n");
            pApp->m_running = false;
        }
    }    

    pApp->appShutdown();

    return 0;
}


//------------------------------------------------------------ Application

Application::Application() : m_renderCnt(1), m_win(0), m_debugFilter(0)
{
    dbgprintf("Application (constructor)\n");
    pApp = this;        // global handle
    m_mouseX = -1;
    m_mouseY = -1;
    m_mods = 0;
    m_fullscreen = false;
    m_startup = true;
    m_active = false;
    memset(m_keyPressed, 0, sizeof(m_keyPressed));
    memset(m_keyToggled, 0, sizeof(m_keyToggled));
}

// appStart - called from the user function startup() to indicate desired application config
bool Application::appStart(const std::string& title, const std::string& shortname, int width, int height, int Major, int Minor, int MSAA, bool GLDebug )
{
    bool vsyncstate = true;

    m_winSz[0] = width;             // desired width & height, may not be actual/final
    m_winSz[1] = height;

    m_cflags.major = Major;         // desired OpenGL settings
    m_cflags.minor = Minor;
    m_cflags.MSAA = MSAA;
    m_cflags.debug = GLDebug;
    m_title = title;

    if (!init()) { dbgprintf("ERROR: Unable to init() app.\n"); return false; }     // user init

    m_active = true;                // YES. make active

    return true;
}

void Application::appRun()
{
    if (pApp != 0x0) {
        if (pApp->m_active && pApp->m_renderCnt > 0) {
            pApp->m_renderCnt--;
            pApp->display();            // Main loop
            appSwapBuffers();           // Swap buffers
        }
    }
}


void Application::appShutdown()
{    
    shutdown();                 // perform user shutdown() first
}

void Application::appHandleArgs(int argc, char** argv)
{
    for (int i = 0; i < argc; i++) {
        if (argv[i][0] == '-') {                // valued argument> app -i input.txt
            on_arg(i, argv[i], argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "-vsync") == 0 && i + 1 < argc) {        // builtin> app -vsync 1
            bool vsync = atoi(argv[i + 1]) ? true : false;
            appSetVSync(vsync);
            i++;
        } else {
            on_arg(i, argv[i], "");                // non-valued arg> app input.txt
        }        
    }
}

#ifdef USE_NETWORK

    void Application::appSendEventToApp ( Event* e )
    {
        pApp->on_event ( e );
    }
#endif


// Every platform main must implement these, even if they do nothing
//
void Application::appSwapBuffers()                   {};
void Application::appResizeWindow ( int w, int h )   {};
void Application::appForegroundWindow()              {};
void Application::appMaximize()  {};
void Application::appMinimize()  {};
void Application::appRestore()   {};
void Application::appPostQuit()  {};
void Application::appSetVSync(bool state)  {};
bool Application::appStopWindow ()   {return false;};

// Input functions
void Application::appUpdateMouse ( float mx, float my, AppEnum button, AppEnum state)    {};
void Application::appUpdateMouse ( float mx, float my )  {};
void Application::appHandleEvent ( guiEvent g )          {};
void Application::appSetKeyPress ( int key, bool state ) {};
void Application::appOpenKeyboard ()     {};
void Application::appCloseKeyboard ()    {};
void Application::appOpenBrowser ( std::string app, std::string query )  {};

// Set functions		
void appSetTitle(const char* title)     {};
void appSetFullscreen ( bool fullscreen )   {};
void appSetVSync(bool state)    {};