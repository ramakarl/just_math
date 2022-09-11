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
#ifndef DEF_DIR_OBJECT
	#define DEF_DIR_OBJECT

	#include "common_defs.h"
	#include <vector>
        #include <string>

	#define FILE_TYPE_DIR 0
	#define FILE_TYPE_FILE 1

	// Hmmm - have to factor this specific stuff out.
	#define FILE_TYPE_BLOCKSAVE 2
	#define FILE_TYPE_BLOCKLEV 3

	typedef struct {
		int type;
		std::string text;
		std::string extension;
		int length;
	} dir_list_element;

	typedef std::vector< dir_list_element > dir_list;

	typedef struct {
		std::string text;
		float length;
	} text_element_t;

	typedef std::vector< text_element_t > elem_vec_t;


	class HELPAPI Directory {
	public:	
		Directory ();

		void LoadDir ( std::string path, std::string filter );
		int CreateDir ( std::string path );
		int getNumFiles ()					{ return (int) mList.size(); }
		dir_list_element getFile ( int n )	{ return mList[n]; }
		bool isDir ( int n )				{ return mList[n].type==0; }

		// Static functions
		static dir_list DirList( std::string path, std::string filter );
		static bool FileExists( std::string filename );
		static std::string ws2s(const std::wstring& s); 
		static std::wstring s2ws(const std::string& s);		
		static dir_list GetFileItems( dir_list input);
		static dir_list GetDirectoryItems( dir_list input);
		static std::string NormalizeSlashies( std::string path );
		static std::string GetExtension( std::string path );
		static std::string GetExecutablePath();
		static std::string GetCollapsedPath( std::string path );
		static std::string gPathDelim;

		std::string getPath ()		{ return mPath; }
		std::string getFilter ()	{ return mFileFilter; }

	private:

		std::string		mPath;
		std::string		mFileFilter;	
		

		dir_list		mList;

	};

#endif
