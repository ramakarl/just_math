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
#ifndef DEF_COMMON
	#define DEF_COMMON

	#pragma warning ( disable: 4005)

	#ifdef _WIN32

		#define WIN32_LEAN_AND_MEAN
		#include <windows.h>
		#undef WIN32_LEAN_AND_MEAN

		#pragma warning ( disable : 4800 )			// cast to bool performance warning
		#pragma warning ( disable : 4996 )			// fopen_s, strcpy_s (not linux compatible)
		#pragma warning ( disable : 4244 )			// conversion from double to float
		#pragma warning ( disable : 4305 )			// truncation from double to float (constants)
        #pragma warning ( disable : 4251 )          // STL objects inside DLL-interface classes

        #if !defined ( LIBHELP_STATIC )
            #if defined ( LIBHELP_EXPORTS )				// inside DLL
                #if defined(_WIN32) || defined(__CYGWIN__)
                    #define HELPAPI		__declspec(dllexport)
                #else
                    #define HELPAPI		__attribute__((visibility("default")))
                #endif
            #else										// outside DLL
                #if defined(_WIN32) || defined(__CYGWIN__)
                    #define HELPAPI		__declspec(dllimport)
                #else
                    #define HELPAPI		//https://stackoverflow.com/questions/2164827/explicitly-exporting-shared-library-functions-in-linux
                #endif
               #endif          
            #else
            #define HELP_API
        #endif

		#include "inttypes.h"

        #define ALIGN(x)			__declspec(align(x))
        #define CACHE_ALIGNED       __declspec(align(64))

		typedef signed char			sint8_t;
		typedef signed short		sint16_t;
		typedef signed int			sint32_t;
		typedef signed long			sint64_t;

        typedef	unsigned char		        uchar;
		typedef uint64_t			xlong;
		typedef uint8_t				XCHAR;
		typedef uint8_t				XBYTE;
		typedef uint16_t			XBYTE2;
		typedef uint32_t			XBYTE4;
		typedef uint64_t			XBYTE8;
		typedef sint8_t				schar;
		typedef sint16_t			sshort;
		typedef sint32_t			sint;
		typedef sint64_t			slong;
		typedef	uint8_t				uchar;
		typedef uint16_t			ushort;
		typedef uint32_t			uint;
		typedef uint64_t			uxlong;     // note: keyword 'ulong' cannot be used with NV_ARM. 'slong' is signed, dont use here

		#define FALSE	0
		#define TRUE	1

		// DWORD included from windows.h (32-bit unsigned int)

      #else   // ANDOID and linux

            #define ALIGN(x)		__attribute__ ((aligned(x)))
            #define CACHE_ALIGNED   __attribute__ ((aligned(64)))

            #include "inttypes.h"

            // typedef __s64				xlong;
            typedef unsigned long long	xlong;
            typedef unsigned char		XCHAR;
            typedef unsigned char		XBYTE;	  // 8-bit
            typedef unsigned short		XBYTE2;	  // 16-bit
            typedef unsigned long		XBYTE4;   // 32-bit
            typedef long long	      	XBYTE8;	  // 64-bit
            typedef XBYTE4		      	DWORD;

            #define FALSE	0
            #define TRUE	1

            typedef uint8_t			    uchar;
            typedef uint16_t		    ushort;
            typedef uint32_t		    uint;
            typedef uint64_t		    uxlong;

            typedef int8_t			    schar;
            typedef int16_t			    sshort;
            typedef int32_t			    sint;
            typedef int64_t			    slong;

            // avoids Clang warnings
            #define __cdecl
            #define __stdcall

            #include <stdarg.h>  // for va_start, va_args

	#endif

    typedef	float	f32;
    typedef	double	f64;

    const f32 ROUNDING_ERROR_f32 = 0.000001f;
    const f64 ROUNDING_ERROR_f64 = 0.00000001;
    const f64 PI64 = 3.1415926535897932384626433832795028841971693993751;

    // Universal functions
    #include <vector>
    #include <string>    
    HELPAPI void checkMem( xlong& total, xlong& used, xlong& app);    
    HELPAPI char getPathDelim();
    HELPAPI void addSearchPath ( const char* path );
    HELPAPI bool getFileLocation ( const char* filename, char* outpath );
    HELPAPI bool getFileLocation ( const char* filename, char* outpath, std::vector<std::string> paths );
    HELPAPI bool getFileLocation ( const std::string filename, std::string &outpath );
    HELPAPI unsigned long getFileSize ( const std::string filename );
    HELPAPI unsigned long getFilePos ( FILE* fp );
    HELPAPI void dbgprintf(const char * fmt, ...);

    //--- OpenGL include
    #ifdef USE_OPENGL
        #define GLEW_STATIC             // make sure glew is static

        #if defined(__ANDROID__)
            #include <EGL/egl.h>
            #include <GLES3/gl3.h>
        #elif defined(__linux__)

        #elif defined(_WIN32)
            #include <GL/glew.h>
            #include <GL/gl.h>
        #endif

        // Basic OpenGL interface	        
        HELPAPI void initBasicGL();
        HELPAPI void checkGL(const char* msg, bool debug=false);        
        HELPAPI void clearGL();
        HELPAPI void createTexGL(int& glid, int w, int h, int clamp = 0x812D, int fmt = 0x8058, int typ = 0x1401, int filter = 0x2601);	// defaults: GL_CLAMP_TO_BORDER, GL_RGBA8, GL_UNSIGNED_BYTE, GL_LINEAR
        HELPAPI void renderTexGL(int w, int h, int glid, char inv1 = 0);
        HELPAPI void renderTexGL(float x1, float y1, float x2, float y2, int glid1, char inv1 = 0);
        HELPAPI void compositeTexGL(float blend, int w, int h, int glid1, int glid2, char inv1 = 0, char inv2 = 0);		// composite two textures	

        struct HELPAPI TexInterface {
            int	prog[3];
            int	vshader[3];
            int	fshader[3];
            int	vbo[3];
            int	utex1[3];
            int	utex2[3];
            int	utexflags[3];
            int	ucoords[3];
            int	uscreen[3];
        };  
    #else
        struct HELPAPI TexInterface {
            int handle;
        };
    #endif  

    HELPAPI void strncpy_sc ( char *dst, const char *src, size_t len);                      // cross-platform
    HELPAPI void strncpy_sc (char *dst, size_t dstsz, const char *src, size_t count );      // cross-platform

    // mathematical macros & inlines
	#ifndef imax
    	#define imax(a,b) (((a) > (b)) ? (a) : (b))
	#endif
	#ifndef imin
	    #define imin(a,b) (((a) < (b)) ? (a) : (b))
	#endif
    inline f64 clamp(f64 value, f64 low, f64 high)      {   return imin(imax(value, low), high);  }    
    
    inline bool fequal(double a, double b, double eps)  {   return (fabs(a - b) < eps);    }
    
    inline float fast_inv_squareroot (float number) {
        long i;
        float x2, y;
        const float threehalfs = 1.5F;
        x2 = number * 0.5F; y = number;
        i = *(long*)&y; i = 0x5f3759df - (i >> 1);
        y = *(float*)&i; y = y * (threehalfs - (x2 * y * y));
        #ifndef Q3_VM
            #ifdef __linux__
                    assert(!isnan(y));
            #endif
        #endif
        return y;
    }

    // color storage in 4-byte uint
	typedef uint32_t			CLRVAL;
    #ifndef COLOR
	    #define COLOR(r,g,b)	( (uint(r*255.0f)<<24) | (uint(g*255.0f)<<16) | (uint(b*255.0f)<<8) )
	#endif
    #ifndef COLORA
	    #define COLORA(r,g,b,a)	( (uint(a*255.0f)<<24) | (uint(b*255.0f)<<16) | (uint(g*255.0f)<<8) | uint(r*255.0f) )
    #endif
	#define ALPH(c)			(float((c>>24) & 0xFF)/255.0)
	#define BLUE(c)			(float((c>>16) & 0xFF)/255.0)
	#define GRN(c)			(float((c>>8)  & 0xFF)/255.0)
	#define RED(c)			(float( c      & 0xFF)/255.0)
    #ifndef CLRVEC
	    #define CLRVEC(c)			( Vector4DF( RED(c), GRN(c), BLUE(c), ALPH(c) ) )
    #endif
    #ifndef VECCLR
	    #define VECCLR(v)			( COLORA( v.x, v.y, v.z, v.w ) )
    #endif

	// math defs
    #ifndef PI
        #define PI					(3.14159265358979f)			// sometimes useful :)
    #endif
    #ifndef DEGtoRAD
	    #define DEGtoRAD			(3.14159265358979f/180.0f)
    #endif
    #ifndef RADtoDEG
	    #define RADtoDEG			(180.0f/3.14159265358979f)
    #endif

#endif
