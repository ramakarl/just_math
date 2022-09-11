//--------------------------------------------------------------------------------
// Copyright 2007-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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

//#define BUILD_EVENT_POOLING			// Enable/disable compiliation of Event Pooling

//#define USE_EVENT_POOLING			// Enable or disable event pooling

#ifndef DEF_EVENT_SYSTEM_H
	#define DEF_EVENT_SYSTEM_H

	#include <map>
	#include <set>
	#include <queue>

	#include "event.h"

	// Static buffer (temporary)
	static char mbuf [ 16384 ];

	#define EVENT_LEN_OFFSET		20

	// Event memory management [required]
	char* new_event_data ( size_t size, int& max, EventPool* pool );
	Event new_event ( size_t size, eventStr_t targ, eventStr_t name, eventStr_t state, EventPool* pool );
	void  expand_event ( Event& e, size_t size );
	void  delete_event	(Event& e);			// aka. free_event
	void  free_event_data ( char* data, EventPool* pool );		

	// Event queue - maintains a queue of events
	class HELPAPI EventQueue {
	public:
		EventQueue ();
		
		void clear ();
		void push ( Event& e );
		void push_back (Event& e );
		void pop ();		
		inline Event front ()	 { return mList.front(); }
		inline Event back ()	 { return mList.back(); }
		int size ()				 { return (int) mList.size(); }

		//-- debugging
		void startTrace ( char* fn );
		void trace (); 
		EventQueue& operator= ( EventQueue &op );

	private:
		std::queue < Event >		mList;		
		FILE*						mTraceFile;
	};


  #ifdef BUILD_EVENT_POOLING
	//------------ Event Pooling [optional]
	// 	
	// Event Pooling was designed for very fast event memory allocations.
	// Whether it is actually faster than malloc/free must be evaluated per platform.
	// The technique uses ten bins of power-of-two fixed width memory pools.
	// Each set contains a linked list of pools. Within a single pool, events
	// are assigned sequentially. When an event is deleted/expanded, it is removed
	// from a pool and assigned to a different one. When all event slots in pools have
	// been used, a new pool inserted in the linked list. When a pool is depleted
	// then it is removed from the linked list. (Rama Hoetzlein)
	// 
	// Limitations:
	// - 10 bins. (limited by log table lookup)
	// - 64 min item size. (must be power of two)
	// - 32768 max item size. (largest bin, limited by log table lookup, and efficient use of 65544 block size.)
	// - 65544 block size. (ideal size to give 4x 16386 blocks, 2x 32770 blocks)
	// - 64 byte header. (size of MBlock struct below, plus little extra)

	#define BIN_CNT			10			// Bins: 0..BIN_CNT-1

	#define	MIN_WIDTH		64			// Smallest bin size. Minimum block width.
	#define MIN_WIDTH_BITS	6			// (Why: Pool header is 64)
	
	#define MAX_BINS		8			// Largest bin number. (0..MAX_BINS). Limited by lookup table.
	#define MAX_POOL_SIZE	16384		// 2^(MAX_BINS+6) = 2^(8+6) = 16384

	#define BLOCK_SIZE		65536		// Total block size

	// Definitions:
	// Pool -	Multiple blocks organized by bin
	// Bin -	Linked list of blocks of a particular width
	// Block -	List of items of a specific width
	// Item	-	Unit of allocation. 
	
	// Memory Pool structure
	//  - First item in a block is the MBlock Header
	//	- Each item in a block has a freeword at the end of it (uint32_t)
	// Block Size	Bin		Item Width 	Item Count	- Item Width = Data width + freeword (4 bytes)
	//	65536		0		64			1024
	//	65536		1		128			512
	//	65536		2		256			256
	//	65536		3		512			128
	//	65536		4		1024		64
	//	65536		5		2048		32
	//	65536		6		4096		16	
	//	65536		7		8192		8
	//  65536		8		16384		4	
	//  65536		9		32768		2
	
	typedef char*	itemPtr;	
	
	__declspec(align(64)) struct MBlock {
		uint32_t	mMagic;
		itemPtr		mPos;		// Position of next alloc item		4 bytes
		itemPtr		mEnd;		// End of block position			4 bytes
		int			mBin;		// Bin number						4 bytes
		int			mUsed;		// Number of used items in block	4 bytes
		int			mWidth;		// Width of bin						4 bytes
		int			mCount;		// Maximum items supported			4 bytes
		MBlock*		mPrev;		// Previous block (linked list)		8 bytes
		MBlock*		mNext;		// Next block (linked list)			8 bytes
		bool		mbFull;		// Is block in a full list?			1 byte
		char		mData;		// Start of data					1 byte
	};
	typedef MBlock*		blockPtr;

	class HELPAPI EventPool {
	public:
		EventPool();
		~EventPool();
		void clear ();

		// User functions
		void* allocItem ( int size );
		void freeItem ( void* item );

		blockPtr addBlock ( int bin );
		blockPtr makeFull ( blockPtr block );
		blockPtr makeFree ( blockPtr block );
		void setBlockHeader ( blockPtr block, int wid, int count, int bin );

		unsigned int getNumBins ()		{ return BIN_CNT; }
		int getHeaderSize ()			{ return sizeof(MBlock); }
		int getBinWidth ( int bin )		{ return ( 1 << (bin + MIN_WIDTH_BITS) ); }

		int getBlockSize ( int bin )	{ return BLOCK_SIZE; }					// total block size (not including header)
		int getBlockWidth ( int bin )	{ return getBinWidth( bin ) ; }			// size of block.. power of 2
		int getItemCount ( int bin )	{ return (BLOCK_SIZE / getBlockWidth ( bin )) - 1; } // number of items in block
		int getItemWidth ( int bin )	{ return getBinWidth ( bin ) - sizeof(int); }		 // max size of item in block
		int getItemMaxSize ( int sz )	{ return getItemWidth( getBin(sz) ); }		// maximum payload size (given intial size)
		
		int getAllocated ( int bin );

		void print ()			{ integrity (); }		
		void integrity ();
		int checkBlock ( blockPtr block );

		blockPtr getFullBin (int n )	{ return mFullBins[n]; }
		blockPtr getEmptyBin ( int n )	{ return mEmptyBins[n]; }
		unsigned char getBin ( unsigned int v );
		unsigned int getBinWidth ( unsigned char b );
						
	private:
		MBlock*		mFullBins[BIN_CNT];
		MBlock*		mEmptyBins[BIN_CNT];
		
		static const char logtable512[];	
		
	};
  #else
		// Not building EventPool. Define empty class.
		#define MAX_POOL_SIZE	0
		class EventPool {				
		};
  #endif

#endif	// DEF_EVENT_SYSTEM_H
