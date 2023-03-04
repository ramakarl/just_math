

#ifndef __MAIN_H__
	#define __MAIN_H__

    // Application lifecycle:
    //     def. Calls made to the base Application will be defined as 'app functions'.
    //     def. Calls made to the derived user class will be defined as 'user functions'.
    //     def. Calls made from the Java/JNI into a native hook will be defined as 'native functions'.
    //
    // Lifecycle:
    //  1. User instantiates a derived class MyApp inheriting from Application. e.g. class MyApp : public Application {..}
    //  2. Application constructor is called. This sets the global pApp to access the Application abstractly.
    //  3. The OS-platform invokes hooks which start the process. e.g. WinMain or Android onCreate
    //  4. The user function startup() is called, which invoked appStartup with desired configuration variables
    //  5. The OS-platform calls appStartWindow
    //  6. appStartWindow is handed context variables from the OS/Native system. e.g HWND or ANativeWindow, JavaVM, etc.
    //  7. appStartWindow creates a new OSWindow to store these platform-specific globals
    //  8. appStartWindow calls appCreateGL to create an OpenGL context & surface
    //  9. The OpenGL context, surface and display are also stored in the OSWindow variables
    // 10. appStartWindow calls user init() if this is the first invocation
    // 11. appStartWindow calls user activate() to handle repeated changes to the display surface
    // 12. appStartWindow calls enable_nvgui to start the 2D drawing framework
    // 13. The OS-platform OR the Application prepare a main loop
    // 14. appRun is called repeatedly, which makes calls to the user display() to perform draw updates
    // 15. The OS-platform OR the Application handle mouse & keyboard events, which are passed to appHandleEvent
    // 16. appHandleEvent processes mouse deltas and calls the user mouse() and motion() functions
    // 17. The OS-platform calls addStopWindow if the app is backgrounded, loses focus, or is re-oriented
    // 18. Once the application resumes, the OS-platform will call appStartWindow again.
    // 19. Upon termination appShutdown is called

	#pragma warning(disable:4996) // preventing snprintf >> _snprintf_s

	#include "common_defs.h"	
	#include "main_includes.h"
	#include <stdio.h>
	#include <vector>
	#include <string>
	#include <map>

	// trick for pragma message so we can write:
	// #pragma message(__FILE__"("S__LINE__"): blah")
	#define S__(x) #x
	#define S_(x) S__(x)
	#define S__LINE__ S_(__LINE__)

	#ifdef DEBUG_HEAP
		#define _CRTDBG_MAP_ALLOC  
		#include <stdlib.h>  
		//#include <crtdbg.h> 
	#else
		#include <stdlib.h>  
	#endif

	#ifdef WIN32
		#ifdef MEMORY_LEAKS_CHECK
			#   pragma message("build will Check for Memory Leaks!")
			#   define _CRTDBG_MAP_ALLOC
			#   include <stdlib.h>
			#   include <crtdbg.h>
			inline void* operator new(size_t size, const char *file, int line)
			{
			   return ::operator new(size, 1, file, line);
			}

			inline void __cdecl operator delete(void *ptr, const char *file, int line) 
			{
			   ::operator delete(ptr, _NORMAL_BLOCK, file, line);
			}

			#define DEBUG_NEW new( __FILE__, __LINE__)
			#define MALLOC_DBG(x) _malloc_dbg(x, 1, __FILE__, __LINE__);
			#define malloc(x) MALLOC_DBG(x)
			#define new DEBUG_NEW
		#endif
	#endif

	

	typedef void (*OSProc)(void);
	class OSWindow;							// Forward reference. Described specifically by each platform.
	struct Event;

	//----------------- to be declared in the code of the sample: so the sample can decide how to display messages
	class Application {
	public:
		Application();								// this initializes the global pApp

		// Application-level functions
		// - abstraction virtual, available for app to override
		virtual void startup()  {}
		virtual bool init() { return true; }
        virtual bool activate() { return true; }
		virtual void shutdown() {}
		virtual void reshape(int w, int h) { }
		virtual void on_arg(int i, std::string arg, std::string val ) {}
		virtual void mouse( AppEnum button, AppEnum action, int mods, int x, int y) {}
		virtual void motion( AppEnum button, int x, int y, int dx, int dy) {}
		virtual void mousewheel(int delta) {}
		virtual void keyboard(int keycode, AppEnum action, int mods, int x, int y) {}
		virtual void keyboardchar(unsigned char key, int mods, int x, int y) {}
		virtual void display() {}
		virtual bool begin() { return true; }
		virtual void end() {}
		virtual void checkpoint() {}
	
		#ifdef USE_NETWORK
			virtual void on_event( Event* e )  {}			// Events
			void appSendEventToApp ( Event* e );
		#endif	

		// App Context
		struct ContextFlags {
			int         major, minor, MSAA, depth, stencil;
			bool        debug, robust, core, forward, stereo;
			ContextFlags(int _major=3, int _minor=0, bool _core=true, int _MSAA=1, int _depth=24, int _stencil=8,bool _debug=false, bool _robust=false, bool _forward=false, bool _stereo=false)
			{
				major = _major; minor = _minor; MSAA = _MSAA; depth = _depth; stencil = _stencil; core = _core; debug = _debug;	robust = _robust;
				forward = _forward;	stereo = _stereo;
			}
		};

	  	// Hardware/Platform specific
	  	// - these functions are implemented differently in main_win, main_x11, main_android.cpp
	  	// - functions are listed here generally in order they are called

		bool appStart( const std::string &name, const std::string& shortname, int width, int height, int Major, int Minor, int MSAA=16, bool GLDebug=false );
		void appHandleArgs (int argc, char** argv);
		bool appStartWindow ( void* arg1=0, void* arg2=0, void* arg3=0, void* arg4=0 );
		bool appCreateGL (const Application::ContextFlags *cflags, int& width, int& height);
		bool appInitGL ();
		void appRun();
		void appSwapBuffers();
		void appPostRedisplay(int n=1) { m_renderCnt=n; }
		void appResizeWindow ( int w, int h );
		void appForegroundWindow();
		void appMaximize();
		void appMinimize();
		void appRestore();
		void appQuit();
		bool appStopWindow ();
		void appShutdown ();

		// Input functions
		void appUpdateMouse ( float mx, float my, AppEnum button, AppEnum state);
		void appUpdateMouse ( float mx, float my );
		void appHandleEvent ( guiEvent g );
		void appSetKeyPress ( int key, bool state );
		void appOpenKeyboard ();
		void appCloseKeyboard ();
        void appOpenBrowser ( std::string app, std::string query );

		// Set functions		
		void appSetTitle(const char* title);
		void appSetFullscreen ( bool fullscreen );
		void appSetVSync(bool state);
		void appSwapInterval(int i);
		void appSaveFrame ( char* fname );

		// Accessors
		bool 				isActive();		
		bool 				onPress(int key) 	{ return m_keyPressed[key] && m_keyToggled[key]; }
		inline void         setWinSz(int w, int h) { m_winSz[0]=w; m_winSz[1]=h; }
		inline const int*   getWinSz() const { return m_winSz; }
		inline int          getWidth() const { return m_winSz[0]; }
		inline int          getHeight() const { return m_winSz[1]; }
		inline const int    getWheel() const { return m_wheel; }
		inline int          getMods() const { return m_mods; }
		bool 				getKeyPress(int key) { return m_keyPressed[key]; }
		void				setKeyPress(int key, bool v) { m_keyPressed[key] = v; }
		inline int			getKeyMods();
		inline void         setMods(int m) { m_mods = m; }
		inline float        getX() { return m_mouseX; }
		inline float        getY() { return m_mouseY; }
		inline float        getDX() { return m_dX; }
		inline float        getDY() { return m_dY; }
		inline bool			isFirstFrame()	{ return m_display_frame==0; }
		inline int 			getDisplayFrame() { return m_display_frame; }

	public:

		std::string   	m_title;

		OSWindow*	   	m_win;
		int				m_renderCnt;

		float			m_mouseX, m_mouseY;				// mouse motion
		float 			m_lastX, m_lastY;
		float 			m_spanX, m_spanY;
		float 			m_dX, m_dY;
		float 			m_startX, m_startY;
		AppEnum 		m_mouseButton;
		AppEnum			m_mouseState;
		int				m_wheel;

		int				m_winSz[4];						// window info
		int				m_mods;
		ContextFlags	m_cflags;
		bool			m_doSwap;
		bool            m_startup;
		bool			m_running;
		bool			m_active;
		bool			m_vsync;
		bool			m_fullscreen;

		bool			m_keyPressed[ 400 ];			// keyboard
		bool			m_keyToggled[ 400 ];

		int				m_display_frame;				// frame capture
		int				m_golden_frame;

		unsigned int  	m_debugFilter;
	};

	// External define (for inclusion in other headers)
	//
	extern Application* pApp;	

#endif
