
//-------- MAIN ANDROID - We are on android platform here

#include <jni.h>

#include <android_native_app_glue.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>

#include "nv_gui.h"

#include "main.h"

// Global abstract pointer to the application
Application* pApp = 0x0;

// OSWindow
// This structure contains Hardware/Platform specific variables
// that would normally go into main.h, but we want to be OS specific.
// Therefore to keep main.h cross-platform, we have a struct for OS-specific variables

struct OSWindow
{
    OSWindow (Application* app) : m_app(app), m_screen(0), m_visible(true), _awindow(0), _display(0), _surface(0), _context(0) {}    // empty constructor

    Application*    m_app;              // handle to the owner application
    int             m_screen;
    bool            m_visible;
    ANativeWindow*  _awindow;
    JavaVM*         _javavm;
    jobject         _javaGlobalObject;
    jclass          _javaGlobalClass;
    EGLDisplay      _display;
    EGLSurface      _surface;
    EGLContext      _context;
    EGLint          _width;
    EGLint          _height;
};

//------------------------------------------------ ANDROID ENTRY POINTS

// These functions have interfaces to the Java/JNI
// Any function which is visible to Java/JNI will have the 'native' prefix. (NOT all functions)

std::string getStrFromJString (JNIEnv* env, jstring str)
{
    if ( !str ) std::string();

    const jsize len = env->GetStringUTFLength(str);
    const char* strChars = env->GetStringUTFChars(str, (jboolean *)0);
    std::string result(strChars, len);
    env->ReleaseStringUTFChars(str, strChars);
    return result;
}
jstring getJStringFromStr (JNIEnv* env, std::string str)
{
    char cstr[1024];
    strcpy ( cstr, str.c_str() );
    return env->NewStringUTF( cstr );
}

extern "C"
{
    #include <android/native_window.h>
    #include <android/native_window_jni.h>
    #include <android/surface_control.h>

    static ANativeWindow *window = 0;

    JNIEXPORT void JNICALL
    Java_com_quantasciences_qtvc_MainActivity_nativeSetSurface ( JNIEnv* env, jclass cself, jobject surface, jobject self)
    {
        if ( surface != 0 ) {
            // get the Window from the Java Android surface
            ANativeWindow* window = ANativeWindow_fromSurface ( env, surface );
            if (window==0x0) { dbgprintf ( "ERROR: Window is null.\n"); return; }

            // get the Java VM from environment
            JavaVM* javaVm;
            env->GetJavaVM( &javaVm );

            // get global objects
            jobject jo = reinterpret_cast<jobject>(env->NewGlobalRef(self));
            jclass jc = reinterpret_cast<jclass>( env->NewGlobalRef(cself));

            // start the native window
            pApp->appStartWindow ( window, javaVm, (void*) jo, (void*) jc );

        } else {
            // stop the native window
            pApp->appStopWindow ();
        }
    }

    JNIEXPORT void JNICALL
    Java_com_quantasciences_qtvc_MainActivity_nativeStartup ( JNIEnv *env, jclass cself, jobject self)
    {
        pApp->startup ();       // Call the user startup. This will call appStart.
    }

    JNIEXPORT void JNICALL
    Java_com_quantasciences_qtvc_MainActivity_nativeRun ( JNIEnv *env, jclass cself)
    {
        pApp->appRun();
    }

    JNIEXPORT void JNICALL
    Java_com_quantasciences_qtvc_MainActivity_nativePassEvent ( JNIEnv* env, jclass cself, jint type, jfloatArray vals )
    {
        guiEvent g;
        //jsize len = env->GetArrayLength(vals);
        jfloat *pvals = env->GetFloatArrayElements(vals, 0);

        g.typeOrdinal = type;
        g.xtarget = pvals[EVT_XTARGET];
        g.ytarget = pvals[EVT_YTARGET];
        g.xfocus = pvals[EVT_XFOCUS];
        g.yfocus = pvals[EVT_YFOCUS];
        g.xspan = pvals[EVT_XSPAN];
        g.yspan = pvals[EVT_YSPAN];

        pApp->appHandleEvent( g );
    }

    JNIEXPORT void JNICALL
    Java_com_quantasciences_qtvc_MainActivity_nativeLoadAssets ( JNIEnv* env, jclass cself, jobject assetManager, jstring sdpath )
    {
        //--- This function unpacks the .apk asset folder to a set of files on the Android device
        dbgprintf ( "Unpacking assets.\n");
        AAssetManager* mgr = AAssetManager_fromJava( env, assetManager );
        AAssetDir* assetDir = AAssetManager_openDir( mgr, "");
        const char* filename = (const char*)NULL;
        char filepath[1024];

        std::string path = getStrFromJString ( env, sdpath );
        strcpy ( filepath, path.c_str() );
        addSearchPath ( filepath );         // add asset path to searchable paths

        while ((filename = AAssetDir_getNextFileName(assetDir)) != NULL) {
            sprintf ( filepath, "%s/%s", path.c_str(), filename );
            dbgprintf ( "  File: %s --> %s\n", filename, filepath );

            AAsset* asset = AAssetManager_open( mgr, filename, AASSET_MODE_STREAMING);
            char buf[BUFSIZ];
            int nb_read = 0;

            FILE* out = fopen( filepath, "w" );
            if ( out == NULL ) {
                dbgprintf ( "ERROR: Unable to write to asset file %s.\n", filepath );
                exit(-1);
            }
            while ((nb_read = AAsset_read(asset, buf, BUFSIZ)) > 0)
                fwrite(buf, nb_read, 1, out);
            fclose(out);

            AAsset_close(asset);
        }
        AAssetDir_close(assetDir);
    }

    JNIEXPORT void JNICALL
    Java_com_quantasciences_qtvc_MainActivity_nativeCleanup(JNIEnv *env, jclass cself)
    {
        dbgprintf ( "nativeCleanup\n");

        // shutdown native stuff, also calls user shutdown()
        pApp->appShutdown ();

        // destroy global references
        env->DeleteGlobalRef( pApp->m_win->_javaGlobalClass );
        env->DeleteGlobalRef( pApp->m_win->_javaGlobalObject );
    }

}

//------------------------------------------------ Application

Application::Application() : m_renderCnt(1), m_win(0), m_debugFilter(0)
{
    dbgprintf ( "Application (constructor)\n" );
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
bool Application::appStart ( const std::string& title, const std::string& shortname, int width, int height, int Major, int Minor, int MSAA, bool GLDebug  )
{
    dbgprintf("appStart");

    bool vsyncstate = true;

    m_winSz[0] = width;             // desired width & height, may not be actual/final
    m_winSz[1] = height;

    m_cflags.major = Major;         // desired OpenGL version
    m_cflags.minor = Minor;

    m_active = false;                // not yet active

    return true;
}

bool Application::appStartWindow ( void* awin, void* jvm, void* jobj, void* jcls )
{
    dbgprintf ( "appStartWindow\n" );

    // Native entry point.
    // Java/JNI will call this function with a new surface and ANativeWindow
    if ( m_win == 0x0 ) {
        m_win = new OSWindow(this);         // create os-specific variables first time
    }

    //-- Assign Android window (from surface)
    m_win->_awindow = (ANativeWindow*) awin;
    m_win->_javavm = (JavaVM*) jvm;
    m_win->_javaGlobalObject = (jobject) jobj;
    m_win->_javaGlobalClass = (jclass) jcls;

    //-- Start OpenGL GLES 3.0
    int wid, hgt;
    appCreateGL ( &m_cflags, wid, hgt );
    m_winSz[0] = wid;
    m_winSz[1] = hgt;

    //-- Additional OpenGL initialization
    appInitGL();

    // GUI framework
    enable_nvgui();

    if ( m_startup ) {                      // Call user init() only ONCE per application
        dbgprintf("init()");
        if ( !init() ) { dbgprintf ( "ERROR: Unable to init() app.\n"); return false; }
    }
    dbgprintf("activate()");                // Call user activate() each time window/surface is recreated
    if ( !activate() ) { dbgprintf ( "ERROR: Activate failed.\n"); return false; }

    m_startup = false;
    m_active = true;                // yes, now active.

    return true;
}

bool Application::appCreateGL (const Application::ContextFlags *cflags, int& width, int& height)
{
    dbgprintf ( "appCreateGL\n" );

    const EGLint attribs[] = {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_NONE
    };
    EGLint attributes[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };

    EGLDisplay display;
    EGLConfig config;
    EGLint numConfigs;
    EGLint format;
    EGLSurface surface;
    EGLContext context;
    GLfloat ratio;

    if (m_win->_awindow == 0x0) {
        dbgprintf("ERROR: Window is null. Cannot start app.");
        return false;
    }

    dbgprintf("Initializing context");

    if ((display = eglGetDisplay(EGL_DEFAULT_DISPLAY)) == EGL_NO_DISPLAY) {
        dbgprintf("eglGetDisplay() returned error %d", eglGetError());
        return false;
    }
    if (!eglInitialize(display, 0, 0)) {
        dbgprintf("eglInitialize() returned error %d", eglGetError());
        return false;
    }

    if (!eglChooseConfig(display, attribs, &config, 1, &numConfigs)) {
        dbgprintf("eglChooseConfig() returned error %d", eglGetError());
        return false;
    }

    if (!eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format)) {
        dbgprintf("eglGetConfigAttrib() returned error %d", eglGetError());
        return false;
    }

    dbgprintf("Creating GL Surface");
    ANativeWindow_setBuffersGeometry(m_win->_awindow, 0, 0, format);

    if (!(surface = eglCreateWindowSurface(display, config, m_win->_awindow, 0))) {
        dbgprintf("eglCreateWindowSurface() returned error %d", eglGetError());
        return false;
    }

    dbgprintf("Creating GL Context");

    if (!(context = eglCreateContext(display, config, EGL_NO_CONTEXT, attributes ))) {
        dbgprintf("eglCreateContext() returned error %d", eglGetError());
        return false;
    }

    dbgprintf("Making Context Current");
    if (!eglMakeCurrent(display, surface, surface, context)) {
        dbgprintf("eglMakeCurrent() returned error %d", eglGetError());
        return false;
    }

    if (!eglQuerySurface(display, surface, EGL_WIDTH, &m_win->_width) ||
        !eglQuerySurface(display, surface, EGL_HEIGHT, &m_win->_height)) {
        dbgprintf("eglQuerySurface() returned error %d", eglGetError());
        return false;
    }

    m_win->_display = display;
    m_win->_surface = surface;
    m_win->_context = context;

    glDisable(GL_DITHER);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    dbgprintf("Set Viewport");
    glViewport(0, 0, m_win->_width, m_win->_height);

    // tell parent about new width & height of android device
    width = m_win->_width;
    height = m_win->_height;

    return true;
}

bool Application::appInitGL()
{
    dbgprintf ( "appInitGL\n" );
    // additional opengl initialization
    //  (primary init of opengl occurs in WINinteral::initBase)
    //  initScreenQuadGL ();
    return true;
}

// appStopWindow
// - this function is called when the application loses focus or activity
// for example, on android when the app is backgrounded but not closed
bool Application::appStopWindow ()
{
    dbgprintf ( "appStopWindow\n" );

    m_active = false;                // no longer active

    eglMakeCurrent ( m_win->_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT );

    eglDestroySurface( m_win->_display, m_win->_surface );

    eglDestroyContext( m_win->_display, m_win->_context);

    return true;
}

void Application::appShutdown ()
{
    // perform user shutdown() first
    shutdown();

    // destroy OpenGL surfaces
    appStopWindow();
}

void Application::appRun ()
{
    if ( pApp != 0x0 ) {
        if ( pApp->m_active && pApp->m_renderCnt > 0) {
            pApp->m_renderCnt--;

            // mouse glide
            if ( m_mouseButton == AppEnum::BUTTON_LEFT && m_mouseState == AppEnum::BUTTON_RELEASE ) {
//              appHandleEvent ( guiEvent(AppEnum::BUTTON_LEFT, AppEnum::BUTTON_RELEASE, m_mouseX, m_mouseY, m_dX, m_dY, 0, 0) );
                m_dX *= 0.95; m_dY *= 0.95;
                motion( m_mouseButton, m_mouseX, m_mouseY, m_dX, m_dY );
                if ( fabs(m_dX) < 1 && fabs(m_dY) < 1 ) m_mouseButton = AppEnum::BUTTON_NONE;
                //dbgprintf("Event: GLD, b:%d,%d x:%f, y:%f, dx:%f, dy:%f\n", m_mouseButton, m_mouseState, m_mouseX, m_mouseY, m_dX, m_dY);
            }

            pApp->display();            // Render user display() function
            appSwapBuffers();           // Swap buffers
        }
    }
}

void Application::appSwapBuffers()
{
    if (!eglSwapBuffers( m_win->_display, m_win->_surface) ) {
        dbgprintf( "ERROR: eglSwapBuffers() returned error %d", eglGetError() );
    }
}

bool Application::isActive()
{
    return m_active;
}
void Application::appSwapInterval(int i)
{
}

// update mouse - position only
void Application::appUpdateMouse ( float mx, float my )
{
    m_lastX = m_mouseX;
    m_lastY = m_mouseY;
    m_dX = (m_dX==-1) ? 0 : m_mouseX - mx;
    m_dY = (m_dY==-1) ? 0 : m_mouseY - my;
    m_mouseX = mx;
    m_mouseY = my;
}
// update mouse - with button or state change
void Application::appUpdateMouse ( float mx, float my, AppEnum button, AppEnum state )
{
    appUpdateMouse ( mx, my );
    if ( button != AppEnum::UNDEF ) {             // button changed
        m_mouseButton = button;
        m_startX = mx; m_startY = my;
    }
    if ( state != AppEnum::UNDEF ) {              // state changed
        m_mouseState = state;
        m_startX = mx; m_startY = my;
    }
    if ( state == AppEnum::BUTTON_PRESS || (state==AppEnum::BUTTON_RELEASE && button==AppEnum::BUTTON_RIGHT) ) {
        m_dX = -1; m_dY = -1;          // indicator to ignore the first motion update
    }
}

void Application::appSetKeyPress(int key, bool state)
{
    // not currently getting keyup for soft keyboard events
    // this will have to be changed to deal with hard keys

    // m_keyToggled[key] = (m_keyPressed[key] != state);
    // m_keyPressed[key] = state;
}

void Application::appHandleEvent (guiEvent g)
{
    switch ( g.typeOrdinal ){
        case AppEnum::ACTION_DOWN:
            if ( m_mouseButton == AppEnum::BUTTON_RIGHT ) return;
            appUpdateMouse( g.xtarget, g.ytarget, AppEnum::BUTTON_LEFT, AppEnum::BUTTON_PRESS );
            mouse( m_mouseButton, m_mouseState, 0, m_mouseX, m_mouseY );
            break;
        case AppEnum::ACTION_MOVE: {
            if ( m_mouseButton == AppEnum::BUTTON_RIGHT ) return;
            appUpdateMouse( g.xtarget, g.ytarget );
            motion( m_mouseButton, m_mouseX, m_mouseY, m_dX, m_dY );
            } break;
        case AppEnum::ACTION_UP: {
            if ( m_mouseButton == AppEnum::BUTTON_RIGHT ) return;
            float dx = m_dX, dy = m_dY;     // start glide
            appUpdateMouse( g.xtarget, g.ytarget , AppEnum::BUTTON_LEFT, AppEnum::BUTTON_RELEASE );
            mouse ( m_mouseButton, m_mouseState, 0, m_mouseX, m_mouseY );
            m_dX = dx; m_dY = dy;
     //     m_mouseButton = AppEnum::BUTTON_NONE;    //-- glide will turn it off
            } break;
        case AppEnum::ACTION_CANCEL:
            if ( m_mouseButton == AppEnum::BUTTON_RIGHT ) return;
            appUpdateMouse( g.xtarget, g.ytarget , AppEnum::BUTTON_LEFT, AppEnum::BUTTON_RELEASE );
            mouse ( m_mouseButton, m_mouseState, 0, m_mouseX, m_mouseY );
            m_mouseButton = AppEnum::BUTTON_NONE;
            break;
        case AppEnum::GESTURE_SCALE_BEGIN:          // start zoom
            appUpdateMouse( g.xtarget, g.ytarget, AppEnum::BUTTON_RIGHT, AppEnum::BUTTON_PRESS );
            m_spanX = g.xspan;
            m_spanY = g.yspan;
            mouse( m_mouseButton, m_mouseState, 0, m_mouseX, m_mouseY );
            break;
        case AppEnum::GESTURE_SCALE: {       // handle zoom
            bool ignore_first = (m_dX==-1 && m_dY==-1);
            appUpdateMouse( g.xtarget, g.ytarget );
            m_dX = ignore_first ? 0 : g.xspan - m_spanX;
            m_dY = ignore_first ? 0 :g.yspan - m_spanY;
            m_spanX = g.xspan;
            m_spanY = g.yspan;
            motion( m_mouseButton, m_mouseX, m_mouseY, m_dX, m_dY );
            } break;
        case AppEnum::GESTURE_SCALE_END:        // end zoom
            appUpdateMouse(g.xtarget, g.ytarget, AppEnum::BUTTON_RIGHT, AppEnum::BUTTON_RELEASE);
            mouse( m_mouseButton, m_mouseState, 0, m_mouseX, m_mouseY );
            m_mouseButton = AppEnum::BUTTON_NONE;
            break;
        case AppEnum::SOFT_KEY_PRESS:
            appSetKeyPress( (int) g.xtarget, true);
            keyboardchar( (uchar) g.xtarget, 0, 0, 0 );
            break;
    }
  //  dbgprintf("Event: %d, b:%d,%d x:%f, y:%f, dx:%f, dy:%f\n", g.typeOrdinal, m_mouseButton, m_mouseState, m_mouseX, m_mouseY, m_dX, m_dY);
}

#ifdef USE_NETWORK
    void Application::appSendEventToApp ( Event* e )
    {
        pApp->on_event ( e );
    }
#endif

void Application::appOpenBrowser ( std::string app, std::string query )
{
    dbgprintf( "appOpenLink");

    JNIEnv *env;
    jint rs = m_win->_javavm->AttachCurrentThread(&env, NULL);
    assert (rs == JNI_OK);
    jmethodID method = env->GetMethodID( m_win->_javaGlobalClass, "openLink", "(Ljava/lang/String;)V");

    jstring jstr = getJStringFromStr ( env, query );
    env->CallVoidMethod( m_win->_javaGlobalObject, method, jstr );
}


void Application::appOpenKeyboard()
{
    JNIEnv *env;
    jint rs = m_win->_javavm->AttachCurrentThread(&env, NULL);
    assert (rs == JNI_OK);
    jmethodID method = env->GetMethodID( m_win->_javaGlobalClass, "openSoftKeyboard", "()V");
    env->CallVoidMethod( m_win->_javaGlobalObject, method);
}

void Application::appForegroundWindow ()
{
}
void Application::appCloseKeyboard()
{
    JNIEnv *env;
    jint rs = m_win->_javavm->AttachCurrentThread(&env, NULL);
    assert (rs == JNI_OK);

    jmethodID method = env->GetMethodID( m_win->_javaGlobalClass, "closeSoftKeyboard", "()V");

    env->CallVoidMethod( m_win->_javaGlobalObject, method);
}

/*static const char *g_screenquad_vert =
	"#version 440 core\n"
	"layout(location = 0) in vec3 vertex;\n"
	"layout(location = 1) in vec3 normal;\n"
	"layout(location = 2) in vec3 texcoord;\n"
	"uniform vec4 uCoords;\n"
	"uniform vec2 uScreen;\n"
	"out vec3 vtc;\n"
	"void main() {\n"
	"   vtc = texcoord*0.5+0.5;\n"
	"   gl_Position = vec4( -1.0 + (uCoords.x/uScreen.x) + (vertex.x+1.0f)*(uCoords.z-uCoords.x)/uScreen.x,\n"
	"                       -1.0 + (uCoords.y/uScreen.y) + (vertex.y+1.0f)*(uCoords.w-uCoords.y)/uScreen.y,\n"
	"                       0.0f, 1.0f );\n"
	"}\n";

static const char *g_screenquad_frag =
	"#version 440\n"
	"uniform sampler2D uTex1;\n"
	"uniform sampler2D uTex2;\n"
	"uniform int uTexFlags;\n"
	"in vec3 vtc;\n"
	"out vec4 outColor;\n"
	"void main() {\n"
	"   vec4 op1 = ((uTexFlags & 0x01)==0) ? texture ( uTex1, vtc.xy) : texture ( uTex1, vec2(vtc.x, 1.0-vtc.y));\n"
	"   if ( (uTexFlags & 0x02) != 0 ) {\n"
	"		vec4 op2 = ((uTexFlags & 0x04)==0) ? texture ( uTex2, vtc.xy) : texture ( uTex2, vec2(vtc.x, 1.0-vtc.y));\n"
	"		outColor = vec4( op1.xyz*(1.0-op2.w) + op2.xyz * op2.w, 1 );\n"
	"   } else { \n"
	"		outColor = vec4( op1.xyz, 1 );\n"
	"   }\n"
	"}\n";


struct nvVertex {
	nvVertex(float x1, float y1, float z1, float tx1, float ty1, float tz1) { x=x1; y=y1; z=z1; tx=tx1; ty=ty1; tz=tz1; }
	float	x, y, z;
	float	nx, ny, nz;
	float	tx, ty, tz;
};
struct nvFace {
	nvFace(unsigned int x1, unsigned int y1, unsigned int z1) { a=x1; b=y1; c=z1; }
	unsigned int  a, b, c;
};

void NVPWindow::initScreenQuadGL()
{
	int status;
	int maxLog = 65536, lenLog;
	char log[65536];

	// Create a screen-space shader
	m_screenquad_prog = (int)glCreateProgram();
	GLuint vShader = (int)glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vShader, 1, (const GLchar**)&g_screenquad_vert, NULL);
	glCompileShader(vShader);
	glGetShaderiv(vShader, GL_COMPILE_STATUS, &status);
	if (!status) {
		glGetShaderInfoLog(vShader, maxLog, &lenLog, log);
		dbgprintf("*** Compile Error in init_screenquad vShader\n");
		dbgprintf("  %s\n", log);
	}

	GLuint fShader = (int)glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fShader, 1, (const GLchar**)&g_screenquad_frag, NULL);
	glCompileShader(fShader);
	glGetShaderiv(fShader, GL_COMPILE_STATUS, &status);
	if (!status) {
		glGetShaderInfoLog(fShader, maxLog, &lenLog, log);
		dbgprintf("*** Compile Error in init_screenquad fShader\n");
		dbgprintf("  %s\n", log);
	}
	glAttachShader(m_screenquad_prog, vShader);
	glAttachShader(m_screenquad_prog, fShader);
	glLinkProgram(m_screenquad_prog);
	glGetProgramiv(m_screenquad_prog, GL_LINK_STATUS, &status);
	if (!status) {
		dbgprintf("*** Error! Failed to link in init_screenquad\n");
	}
	checkGL ( "glLinkProgram (init_screenquad)" );
	
	// Get texture parameter
	m_screenquad_utex1 = glGetUniformLocation (m_screenquad_prog, "uTex1" );
	m_screenquad_utex2 = glGetUniformLocation (m_screenquad_prog, "uTex2");
	m_screenquad_utexflags = glGetUniformLocation(m_screenquad_prog, "uTexFlags");
	m_screenquad_ucoords = glGetUniformLocation ( m_screenquad_prog, "uCoords" );
	m_screenquad_uscreen = glGetUniformLocation ( m_screenquad_prog, "uScreen" );


	// Create a screen-space quad VBO
	std::vector<nvVertex> verts;
	std::vector<nvFace> faces;
	verts.push_back(nvVertex(-1, -1, 0, -1, 1, 0));
	verts.push_back(nvVertex(1, -1, 0, 1, 1, 0));
	verts.push_back(nvVertex(1, 1, 0, 1, -1, 0));
	verts.push_back(nvVertex(-1, 1, 0, -1, -1, 0));
	faces.push_back(nvFace(0, 1, 2));
	faces.push_back(nvFace(2, 3, 0));

	glGenBuffers(1, (GLuint*)&m_screenquad_vbo[0]);
	glGenBuffers(1, (GLuint*)&m_screenquad_vbo[1]);
	checkGL("glGenBuffers (init_screenquad)");
	glGenVertexArrays(1, (GLuint*)&m_screenquad_vbo[2]);
	glBindVertexArray(m_screenquad_vbo[2]);
	checkGL("glGenVertexArrays (init_screenquad)");
	glBindBuffer(GL_ARRAY_BUFFER, m_screenquad_vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(nvVertex), &verts[0].x, GL_STATIC_DRAW_ARB);
	checkGL("glBufferData[V] (init_screenquad)");
	glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(nvVertex), 0);				// pos
	glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)12);	// norm
	glVertexAttribPointer(2, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)24);	// texcoord
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_screenquad_vbo[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.size() * 3 * sizeof(int), &faces[0].a, GL_STATIC_DRAW_ARB);
	checkGL("glBufferData[F] (init_screenquad)");
	glBindVertexArray(0);
}


void NVPWindow::createScreenQuadGL ( int* glid, int w, int h )
{
	if ( *glid == -1 ) glDeleteTextures ( 1, (GLuint*) glid );
	glGenTextures ( 1, (GLuint*) glid );
	glBindTexture ( GL_TEXTURE_2D, *glid );
	checkGL ( "glBindTexture (createScreenQuadGL)" );
	glPixelStorei ( GL_UNPACK_ALIGNMENT, 4 );	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);	
	glTexImage2D  ( GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);	
	checkGL ( "glTexImage2D (createScreenQuadGL)" );
	glBindTexture ( GL_TEXTURE_2D, 0 );
}

void NVPWindow::clearScreenGL ()
{
	glClearDepth ( 1.0 );
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
}

void NVPWindow::renderScreenQuadGL(int glid, char inv1)
{
	renderScreenQuadGL ( glid, -1, (float)0, (float)0, (float)getWidth(), (float)getHeight(), inv1); 
}

void NVPWindow::compositeScreenQuadGL(int glid1, int glid2, char inv1, char inv2)
{
	renderScreenQuadGL( glid1, glid2, (float)0, (float)0, (float)getWidth(), (float)getHeight(), inv1, inv2 );
}

void NVPWindow::renderScreenQuadGL ( int glid1, int glid2, float x1, float y1, float x2, float y2, char inv1, char inv2 )

{
	// Prepare pipeline
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDepthMask(GL_FALSE);
	// Select shader	
	glBindVertexArray(m_screenquad_vbo[2]);
	glUseProgram(m_screenquad_prog);
	checkGL("glUseProgram");
	// Select VBO	
	glBindBuffer(GL_ARRAY_BUFFER, m_screenquad_vbo[0]);
	glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(nvVertex), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)12);
	glVertexAttribPointer(2, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)24);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_screenquad_vbo[1]);
	checkGL("glBindBuffer");
	// Select texture
	glEnable ( GL_TEXTURE_2D );
	glProgramUniform4f ( m_screenquad_prog, m_screenquad_ucoords, x1, y1, x2, y2 );
	glProgramUniform2f ( m_screenquad_prog, m_screenquad_uscreen, (float) getWidth(), (float) getHeight() );

	glActiveTexture ( GL_TEXTURE0 );
	glBindTexture ( GL_TEXTURE_2D, glid1 );

	glProgramUniform1i(m_screenquad_prog, m_screenquad_utex1, 0);
	int flags = 0;
	if (inv1 > 0) flags |= 1;												// y-invert tex1

	if (glid2 >= 0) {
		flags |= 2;															// enable tex2 compositing
		if (inv2 > 0) flags |= 4;											// y-invert tex2
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, glid2);
		glProgramUniform1i(m_screenquad_prog, m_screenquad_utex2, 1);
	}

	glProgramUniform1i(m_screenquad_prog, m_screenquad_utexflags, flags );	

	// Draw
	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1);
	checkGL("glDraw");
	glUseProgram(0);

	glDepthMask(GL_TRUE);
}*/

int main(int argc, char **argv)
{
    dbgprintf ("Starting here\n");
    return 0;
}


