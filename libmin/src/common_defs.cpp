//--------------------------------------------------------------------------------
// Copyright 2019-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
//
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

#include "common_defs.h"
#include "vec.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <string>

#if defined(__ANDROID__)
    #include <android/log.h>               // for Android printf logs  
#elif defined(_WIN32)
    #include <windows.h>
    #include <processthreadsapi.h>      // Process memory usage on Win32
    #include <psapi.h>  
#endif

static std::vector<std::string> gPaths;

static xlong gMemStart = 0;       // amount of used memory when application starts

void dbgprintf(const char * fmt, ...)
{
    va_list  vlist;
    va_start(vlist, fmt);    
    #if defined(__ANDROID__)
        __android_log_vprint(ANDROID_LOG_DEBUG, "NAPP", fmt, vlist );
    #elif defined(__linux__)
        vprintf(fmt, vlist);
    #elif defined(_WIN32)
        vprintf(fmt, vlist);
    #endif
}

char getPathDelim()
{
    #ifdef _WIN32
        return '\\';
    #else
        return '/';
    #endif
}

void addSearchPath ( const char* path )
{
    std::string pathstr = path;
    // every search path must be terminated with a delimiter. add one if needed
    if ( pathstr.at( pathstr.length()-1) != getPathDelim() ) {
        pathstr = pathstr + getPathDelim();
    }
    // add the path
    //dbgprintf ( "Added search path: %s\n", pathstr.c_str() );

    gPaths.push_back ( pathstr );
}
bool getFileLocation ( const char* filename, char* outpath )
{
    bool result = getFileLocation ( filename, outpath, gPaths );
    return result;
}
bool getFileLocation ( const char* filename, char* outpath, std::vector<std::string> searchPaths )
{
    bool found = false;
    FILE* fp = fopen( filename, "rb" );
    if (fp) {
        found = true;
        strcpy ( outpath, filename );
    } else {
        for (int i=0; i < searchPaths.size(); i++) {
            if (searchPaths[i].empty() ) continue;
            sprintf ( outpath, "%s%s", searchPaths[i].c_str(), filename );
            fp = fopen( outpath, "rb" );
            if (fp)	{ found = true;	break; }
        }
    }
    if ( found ) fclose ( fp );
    return found;
}

bool getFileLocation ( const std::string filename, std::string& outpath )
{
    char instr[2048];
    char outstr[2048];
    strncpy_sc (instr, filename.c_str(), 2048 );
    bool result = getFileLocation ( instr, outstr, gPaths );
    outpath = outstr;
    return result;
}

unsigned long getFileSize ( const std::string filename )
{
    char instr[2048];
    strncpy_sc (instr, filename.c_str(), 2048) ;
    FILE* fp;
    fp = fopen ( instr, "rb");
    if ( fp==0x0 ) return 0;
    fseek ( fp, 0, SEEK_END );
    unsigned long fsize = ftell ( fp );
    fclose ( fp );

    return fsize;
}
unsigned long getFilePos ( FILE* fp )
{
    return ftell ( fp );
}


void strncpy_sc ( char *dst, const char *src, size_t len)
{
#if defined(__ANDROID__)
    strlcpy (dst, src, len );
#elif defined(__linux__)
    strncpy ( dst, src, len );
#elif defined(_WIN32)
    strncpy ( dst, src, len );
#endif
}

void strncpy_sc (char *dst, size_t dstsz, const char *src, size_t len )
{
 #if defined(__ANDROID__)
    strlcpy (dst, src, len );
#elif defined(__linux__)
    strncpy( dst, src, len );
#elif defined(_WIN32)
    strncpy_s (dst, dstsz, src, len );
#endif
    
    /*//C11 standard
    //src or dest is a null pointer
    //dstsz or count is zero or greater than RSIZE_MAX
    //dstsz is less or equal strnlen_s(src, count), in other words, truncation would occur
    //overlap would occur between the source and the destination strings

    #define RSIZE_MAX INT64_MAX

        bool nullFailed = ( !src || !dst );
        bool sizeFailed = ( dstsz == 0 || dstsz > RSIZE_MAX || count == 0 || count > RSIZE_MAX );
        bool truncFailed = ( dstsz <= strnlen(src, count) );
        bool overlapFailed = false; // TODO - unsure of the test here

        if ( nullFailed || sizeFailed || truncFailed || overlapFailed ) {
            return;
            // TODO - return nice error
        } else {
            strlcpy ( dst, src, count );
        }
    #endif*/
}

void checkMem(xlong& total, xlong& used, xlong& app)
{
    struct _MEMORYSTATUSEX memx;
    memset(&memx, 0, sizeof(memx));
    memx.dwLength = sizeof(memx);

    GlobalMemoryStatusEx(&memx);
    total = memx.ullTotalPhys;
    used = memx.ullTotalPhys - memx.ullAvailPhys;

    PROCESS_MEMORY_COUNTERS pmc;

    BOOL result = GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
    app = 0;
    if (result)
        app = pmc.WorkingSetSize;
}


//------------------------------------------------------------- OPENGL
//
#ifdef USE_OPENGL
    
    const char * glErrorString(GLenum const err) 
    {
        switch (err) {        
        case GL_NO_ERROR:           return "Ok";  // opengl 2 errors (8)
        case GL_INVALID_ENUM:       return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE:      return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION:  return "GL_INVALID_OPERATION";
        case GL_STACK_OVERFLOW:     return "GL_STACK_OVERFLOW";
        case GL_STACK_UNDERFLOW:    return "GL_STACK_UNDERFLOW";
        case GL_OUT_OF_MEMORY:      return "GL_OUT_OF_MEMORY";
        case GL_TABLE_TOO_LARGE:    return "GL_TABLE_TOO_LARGE";        
        case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";      // opengl 3 errors (1)        
        default:                    return "UNKNOWN"; 
        }
    }

    #if defined(__ANDROID__)
    
        void checkGL(const char* msg, bool debug)
        {
            GLenum errCode = 0;
            errCode = glGetError();
            if ( errCode != GL_NO_ERROR || debug ) {
                const char* errString = glErrorString(errCode);
                dbgprintf("GL: %s, code: %x (%s)\n", msg, errCode, errString );
            }
        }

    #elif defined(__linux__)
        void checkGL(const char* msg, bool debug) {}
        void checkMem(xlong& total, xlong& used, xlong& app) {}

    #elif defined(_WIN32)

        void checkGL(const char* msg, bool debug)
        {
            GLenum errCode = 0;
            errCode = glGetError();
            if (errCode != GL_NO_ERROR || debug) {
                const char* errString = glErrorString(errCode);
                dbgprintf("GL: %s, code: %x (%s)\n", msg, errCode, errString);
            }
        }

    //-------------------------------------------- Texture interface

    //-- screen shader 
    static const char* g_tex_vertshader =
        "#version 300 es\n"
        "#extension GL_ARB_explicit_attrib_location : enable\n"
        "layout(location = 0) in vec3 vertex;\n"
        "layout(location = 1) in vec3 normal;\n"
        "layout(location = 2) in vec3 texcoord;\n"
        "uniform vec4 uCoords;\n"
        "uniform vec2 uScreen;\n"
        "out vec3 vtc;\n"
        "void main() {\n"
        "   vtc = texcoord * 0.5f + 0.5f;\n"
        "   gl_Position = vec4( -1.0f + (uCoords.x/uScreen.x) + (vertex.x+1.0f)*(uCoords.z-uCoords.x)/uScreen.x,\n"
        "                       -1.0f + (uCoords.y/uScreen.y) + (vertex.y+1.0f)*(uCoords.w-uCoords.y)/uScreen.y,\n"
        "                       0.0f, 1.0f );\n"
        "}\n";
    static const char* g_tex_fragshader =
        "#version 300 es\n"
        "  precision mediump float;\n"
        "  precision mediump int;\n"
        "uniform sampler2D uTex1;\n"
        "uniform sampler2D uTex2;\n"
        "uniform int uTexFlags;\n"
        "in vec3 vtc;\n"
        "layout(location = 0) out vec4 outColor;\n"
        "void main() {\n"
        "   vec4 op1 = ((uTexFlags & 0x01)==0) ? texture ( uTex1, vtc.xy) : texture ( uTex1, vec2(vtc.x, 1.0f - vtc.y));\n"
        "   if ( (uTexFlags & 0x04) != 0 ) {\n"
        "		vec4 op2 = ((uTexFlags & 0x02)==0) ? texture ( uTex2, vtc.xy) : texture ( uTex2, vec2(vtc.x, 1.0f - vtc.y));\n"
        "		outColor = vec4( op1.xyz * (1.0f - op2.w) + op2.xyz * op2.w, 1.0f );\n"
        "   } else { \n"
        "		outColor = vec4( op1.xyz, 1.0f );\n"
        "   }\n"
        "}\n";

    #define SHADRS   1
        
    TexInterface gTex;              // global texture interface

    struct nvVertex {
        nvVertex(float x1, float y1, float z1, float tx1, float ty1, float tz1) { x = x1; y = y1; z = z1; tx = tx1; ty = ty1; tz = tz1; }
        float	x, y, z;
        float	nx, ny, nz;
        float	tx, ty, tz;
    };
    struct nvFace {
        nvFace(unsigned int x1, unsigned int y1, unsigned int z1) { a = x1; b = y1; c = z1; }
        unsigned int  a, b, c;
    };

    void initBasicGL()
    {
        dbgprintf( "  initBasicGL: Initializing Glew for libmin.\n");

        glewInit();      // init glew pointers for libmin.dll

        int status;
        int maxLog = 65536, lenLog;
        char log[65536];

        // list of internal shaders
        const char** vertcode[SHADRS] = { &g_tex_vertshader };
        const char** fragcode[SHADRS] = { &g_tex_fragshader };

        // load each shader
        for (int n = 0; n < SHADRS; n++) {

            // Create a screen-space shader
            gTex.prog[n] = (int)glCreateProgram();
            GLuint vShader = (int)glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vShader, 1, (const GLchar**)vertcode[n], NULL);
            glCompileShader(vShader);
            glGetShaderiv(vShader, GL_COMPILE_STATUS, &status);
            if (!status) {
                glGetShaderInfoLog(vShader, maxLog, &lenLog, log);
                dbgprintf("*** Compile Error in init_screenquad vShader\n");
                dbgprintf("  %s\n", log);
            }
            GLuint fShader = (int)glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fShader, 1, (const GLchar**)fragcode[n], NULL);
            glCompileShader(fShader);
            glGetShaderiv(fShader, GL_COMPILE_STATUS, &status);
            if (!status) {
                glGetShaderInfoLog(fShader, maxLog, &lenLog, log);
                dbgprintf("*** Compile Error in init_screenquad fShader\n");
                dbgprintf("  %s\n", log);
            }
            glAttachShader(gTex.prog[n], vShader);
            glAttachShader(gTex.prog[n], fShader);
            glLinkProgram(gTex.prog[n]);
            glGetProgramiv(gTex.prog[n], GL_LINK_STATUS, &status);
            if (!status) {
                dbgprintf("*** Error! Failed to link in init_screenquad\n");
            }
            checkGL("glLinkProgram (init_screenquad)");

            // Get texture params
            gTex.utex1[n] = glGetUniformLocation(gTex.prog[n], "uTex1");
            gTex.utex2[n] = glGetUniformLocation(gTex.prog[n], "uTex2");
            gTex.utexflags[n] = glGetUniformLocation(gTex.prog[n], "uTexFlags");
            gTex.ucoords[n] = glGetUniformLocation(gTex.prog[n], "uCoords");
            gTex.uscreen[n] = glGetUniformLocation(gTex.prog[n], "uScreen");
        }
        glBindVertexArray(0);

        // Create a screen-space quad VBO
        std::vector<nvVertex> verts;
        std::vector<nvFace> faces;
        verts.push_back(nvVertex(-1, -1, 0, -1, 1, 0));
        verts.push_back(nvVertex(1, -1, 0, 1, 1, 0));
        verts.push_back(nvVertex(1, 1, 0, 1, -1, 0));
        verts.push_back(nvVertex(-1, 1, 0, -1, -1, 0));
        faces.push_back(nvFace(0, 1, 2));
        faces.push_back(nvFace(2, 3, 0));

        glGenBuffers(1, (GLuint*)&gTex.vbo[0]);
        glGenBuffers(1, (GLuint*)&gTex.vbo[1]);
        checkGL("glGenBuffers (init_screenquad)");
        glGenVertexArrays(1, (GLuint*)&gTex.vbo[2]);
        glBindVertexArray(gTex.vbo[2]);
        checkGL("glGenVertexArrays (init_screenquad)");
        glBindBuffer(GL_ARRAY_BUFFER, gTex.vbo[0]);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(nvVertex), &verts[0].x, GL_STATIC_DRAW_ARB);
        checkGL("glBufferData[V] (init_screenquad)");
        glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(nvVertex), 0);			// pos
        glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)12);	// norm
        glVertexAttribPointer(2, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)24);	// texcoord
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gTex.vbo[1]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.size() * 3 * sizeof(int), &faces[0].a, GL_STATIC_DRAW_ARB);
        checkGL("glBufferData[F] (init_screenquad)");
    }

    void createTexGL(int& glid, int w, int h, int clamp, int fmt, int typ, int filter)
    {
        if (glid != -1) glDeleteTextures(1, (GLuint*)&glid);
        glGenTextures(1, (GLuint*)&glid);

        int texDims[2];
        glBindTexture(GL_TEXTURE_2D, glid);
        checkGL("glBindTexture (createTexGL)");
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &texDims[0]);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &texDims[1]);
        if (texDims[0] == w && texDims[1] == h) return;
        checkGL("getTexLevelParam (createTexGL)");

        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, clamp);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, clamp);
        glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, GL_RGBA, typ, 0);
        checkGL("glTexImage2D (createTexGL)");

        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void clearGL()
    {
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void renderTexGL(int w, int h, int glid, char inv1)
    {
        renderTexGL((float)0, (float)0, (float)w, (float)h, glid, inv1);
    }
    void renderTexGL(float x1, float y1, float x2, float y2, int glid1, char inv1)
    {
        // Prepare pipeline   
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDepthMask(GL_FALSE);

        // Get viewport dimensions (actual pixels)
        float screen[4];
        glGetFloatv(GL_VIEWPORT, screen);

        glBindVertexArray(gTex.vbo[2]);                                // Select shader
        checkGL("glBindVertexArray");

        int s = 0;      // shader #
        glUseProgram(gTex.prog[s]);
        checkGL("glUseProgram");
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);

        glUniform4f(gTex.ucoords[s], x1, y1, x2, y2);                     // Select texture    
        glUniform2f(gTex.uscreen[s], (float)screen[2], (float)screen[3]);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, glid1);
        glUniform1i(gTex.utex1[s], 0);
        checkGL("glBindTexture");

        glBindBuffer(GL_ARRAY_BUFFER, gTex.vbo[0]);                     // Select VBO	
        glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(nvVertex), 0);
        glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)12);
        glVertexAttribPointer(2, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)24);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gTex.vbo[1]);
        checkGL("glBindBuffer");

        int flags = inv1;
        glUniform1i(gTex.utexflags[s], flags);    // inversion flag

        glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1);

        checkGL("renderTexGL");
        glUseProgram(0);
        glDepthMask(GL_TRUE);
    }

    void compositeTexGL(float blend, int w, int h, int glid1, int glid2, char inv1, char inv2)
    {
        // Prepare pipeline   
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDepthMask(GL_FALSE);

        glBindVertexArray(gTex.vbo[2]);                                // Select shader
        checkGL("glBindVertexArray");

        int s = 0;  // shader #
        glUseProgram(gTex.prog[s]);
        checkGL("glUseProgram");
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);

        glUniform4f(gTex.ucoords[s], 0, 0, float(w), float(h));                     // Select texture    
        glUniform2f(gTex.uscreen[s], float(w), float(h));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, glid1);                         // Bind two textures
        glUniform1i(gTex.utex1[s], 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, glid2);
        glUniform1i(gTex.utex2[s], 1);
        checkGL("glBindTexture");

        glBindBuffer(GL_ARRAY_BUFFER, gTex.vbo[0]);                     // Select VBO	
        glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(nvVertex), 0);
        glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)12);
        glVertexAttribPointer(2, 3, GL_FLOAT, false, sizeof(nvVertex), (void*)24);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gTex.vbo[1]);
        checkGL("glBindBuffer");

        int flags = inv1 | (inv2 << 1) | 4;
        glUniform1i(gTex.utexflags[s], flags);    // inversion flag

        glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1);

        checkGL("compositeTexGL");
        glUseProgram(0);
        glDepthMask(GL_TRUE);

    }

    #endif

#endif

  






        