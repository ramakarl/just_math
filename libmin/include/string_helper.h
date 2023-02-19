//--------------------------------------------------------------------------------
// NVIDIA(R) GVDB VOXELS
// Copyright 2017, NVIDIA Corporation
//
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this 
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this 
//    list of conditions and the following disclaimer in the documentation and/or 
//    other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may 
//    be used to endorse or promote products derived from this software without specific 
//   prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
// SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
// OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Version 1.0: Rama Hoetzlein, 5/1/2017
//----------------------------------------------------------------------------------

#ifndef DEF_STRING_HELPER
	#define DEF_STRING_HELPER

	#include "common_defs.h"
	#include "vec.h"
	#include <string>
	#include <vector>

	#ifndef DEF_OBJTYPE
		typedef	uint32_t		objType;
	#endif
	HELPAPI std::string strFilebase ( std::string str );	// basename of a file (minus ext)
	HELPAPI std::string strFilepath ( std::string str );	// path of a file

	// convert
	HELPAPI bool isFloat (std::string s);	// fast	
	HELPAPI int strToI (std::string s);
	HELPAPI float strToF (std::string s);
	HELPAPI double strToD (std::string s);
	HELPAPI float strToDateF( std::string s, int mp=0, int mc=2, int dp=3, int dc=2, int yp=6, int yc=4 );
	HELPAPI void strFromDateF ( float f, int& m, int& d, int& y );
	HELPAPI unsigned char strToC ( std::string s );
	HELPAPI unsigned long strToUL ( std::string s );
	HELPAPI unsigned long strToID ( std::string str );		// should only be used for 4-byte literals. for actual unsigned long see strToUL
	
	HELPAPI std::string cToStr ( char c );
	HELPAPI std::string iToStr ( int i );
	HELPAPI std::string fToStr ( float f );
	HELPAPI std::string xlToStr ( uint64_t v );
	HELPAPI objType strToType ( std::string str );
	HELPAPI std::string typeToStr ( objType t );
	HELPAPI std::string wsToStr ( const std::wstring& str );
	HELPAPI std::wstring strToWs (const std::string& s);

	HELPAPI bool strToVec(std::string& str, uchar lsep, uchar insep, uchar rsep, float* vec, int cpt = 3);
	HELPAPI bool strToVec3(std::string& str, uchar lsep, uchar insep, uchar rsep, float* vec);
	HELPAPI bool strToVec4(std::string& str, uchar lsep, uchar insep, uchar rsep, float* vec);
	HELPAPI Vector3DF strToVec3(std::string str, uchar sep);
	HELPAPI Vector4DF strToVec4(std::string str, uchar sep);

	//----------- Boolean returns
	
	HELPAPI bool		strSplit ( std::string str, std::string sep, std::string& left, std::string& right );		// "left,right" --> left="left", right="right", str=unchanged
	HELPAPI bool		strSplitLeft(std::string str, std::string sep, std::string& key, std::string& val);			// "left,right" --> key="left", val="right", str=unchanged
	HELPAPI bool		strParseOut(std::string str, std::string lsep, std::string rsep, std::string& result, std::string& rest); // "data<1492> | time", <> --> result 1492, str="date | time"
	HELPAPI bool		strParseKeyVal(std::string& str, uchar lsep, uchar rsep, std::string& key, std::string& val); // "obj<car>,more" --> key="obj", val="car", str="more"
	HELPAPI bool		strFileSplit(std::string str, std::string& path, std::string& name, std::string& ext);
	HELPAPI int			strSplitMultiple(std::string str, std::string sep, std::vector<std::string>& list);		// obj1,obj2,obj3.. list={ob1, obj2, obj3}

	//----------- String returns

	HELPAPI std::string strSplitLeft ( std::string& str, std::string sep );											// "left,right" --> return "left", str="right"
	HELPAPI std::string strSplitRight ( std::string& str, std::string sep );										// "left,right" --> return "right", str="left"	
	HELPAPI std::string strParseOut(std::string& str, std::string lsep, std::string rsep = " ");					// "data<1492> | time", <> --> return 1492, str="date | time"

	


	//----------- original api
	HELPAPI std::string strParse ( std::string str, std::string lstr, std::string rstr, std::string lsep, std::string rsep );
	HELPAPI std::string strParseArg ( std::string& tag, std::string valsep, std::string sep, std::string& str );
	HELPAPI std::string strParseFirst ( std::string& str, std::string sep, std::string others, char& ch );
	
	HELPAPI bool		strGet ( std::string str, std::string& result, std::string lsep, std::string rsep );
	HELPAPI bool		strGet ( const std::string& s, std::string lsep, std::string rsep, std::string& result, size_t& pos );
	
	HELPAPI bool		strSub ( std::string str, int first, int cnt, std::string cmp );
	HELPAPI std::string strReplace ( std::string str, std::string src, std::string dest );
	HELPAPI bool		strReplace ( std::string& str, std::string src, std::string dest, int& cnt );
	HELPAPI int			strExtract ( std::string& str, std::vector<std::string>& list );
	HELPAPI int			strFindFromList ( std::string str, std::vector<std::string>& list, int& pos );
	HELPAPI bool		strEmpty ( const std::string& s);

	//---------- Trimming
	HELPAPI std::string strLTrim ( std::string str );
	HELPAPI std::string strRTrim ( std::string str );
	HELPAPI std::string strTrim ( std::string str );
	HELPAPI std::string strTrim ( std::string str, std::string ch );
	HELPAPI std::string strLeft ( std::string str, int n );
	HELPAPI std::string strRight ( std::string str, int n );
	HELPAPI std::string strLeftOf ( std::string str, std::string sep );
	HELPAPI std::string strMidOf ( std::string str, std::string sep );
	HELPAPI std::string strRightOf ( std::string str, std::string sep );
	
	// alphanumeric
	HELPAPI bool strIsNum ( std::string str, float& f );
	HELPAPI float strToNum ( std::string str );
	HELPAPI int strCount ( std::string& str, char ch );

	HELPAPI bool readword ( char *line, char delim, char *word, int max_size  );
	HELPAPI std::string readword ( char *line, char delim );

#endif