//--------------------------------------------------------------------------------
// Copyright 2007-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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

#include "event_system.h"

#include <assert.h>
#include <cmath>
#include <stdio.h>

//	void assign( eventPtr src, bool v )	{ e = src.e; memcpy(scope,src.scope,4); bOwn = v; }
//	void rescope( char* s )				{ memcpy(scope,s,4); }

char* new_event_data ( size_t size, int& max, EventPool* pool )
{
	char* data;

	max = size;										// maximum *payload* size

	size += Event::staticSerializedHeaderSize();	// additional memory for serial header

	// allocate payload
	if ( size > MAX_POOL_SIZE || pool==0x0 ) {
		// standard allocation
		data = (char*) malloc ( size );
		if ( data==0 ) {
			dbgprintf ("ERROR: Unable to allocate large event.\n");
			exit(-2);
		}
	} else {
		#ifdef BUILD_EVENT_POOLING
			// optional Event Pooling
			data = (char*) pool->allocItem ( (int) size );
			max = pool->getItemMaxSize ( (int) size );
			max -= Event::staticSerializedHeaderSize();
		#endif
	}
	// serial header is automatically set aside (preceeds the data start pos)
	// return the start of payload area
	return data + Event::staticSerializedHeaderSize();
}

Event new_event (size_t size, eventStr_t targ, eventStr_t name, eventStr_t state, EventPool* pool )
{
	// create event
	Event p;
	p.mTimeStamp = 0;
	p.mRefs = 0;
	p.mTargetID = NULL_TARGET;
	p.mTarget = targ;
	p.mName = name;
	p.mDataLen = 0;
	p.mMax = 0;	
	
	// provide room for seralized header
	p.mData = new_event_data ( size, p.mMax, pool );	// payload allocation
	p.bOwn = true;
	p.bDestroy = false;			// no kill on local destructor

	p.mPos = p.getData();	

	return p;
}

void expand_event (Event& p, size_t size)
{
	EventPool* pool = p.mOwner;
	char* old_data = p.mData;
	int old_pos = p.getPos() - p.getData();
	char* new_data;
	int new_max;

	// allocate new data
	new_data = new_event_data ( size, new_max, pool );

	// copy old payload data into new data
	memcpy ( new_data, old_data, p.mDataLen );		// will overwrite the event pool var

	// update event
	p.mMax = new_max;
	p.mData = new_data;
	p.mPos = new_data + old_pos;

	// free old payload memory
	free_event_data ( old_data, pool );
}

void free_event_data ( char* data, EventPool* pool )
{
	if ( data==0x0) return;		// nothing to free

	// adjust back to the original allocation pointer
	data -= Event::staticSerializedHeaderSize();

	if ( pool == 0x0 ) {
		free ( data );
	} else {
		#ifdef BUILD_EVENT_POOLING
			pool->freeItem ( data );
		#endif
	}
}

void delete_event (Event& p)
{
	if ( p.bOwn && p.bDestroy && p.mData != 0x0 ) 
		free_event_data ( p.mData, p.mOwner );

	p.mPos = 0;
	p.mData = 0;
	p.bOwn = false;
	p.bDestroy = true;
}

//---------------------------------------------- Event Queue

EventQueue::EventQueue ()
{
	mList.empty ();
	mTraceFile = 0x0;
}

void EventQueue::startTrace ( char* fn )
{
	mTraceFile = fopen ( fn, "w+t" );
}
void EventQueue::push ( Event& e )
{
	e.incRefs ();				// Added to queue. Increment ref count.
	mList.push ( e );

	//	trace ();
}

void EventQueue::push_back ( Event& e )
{
	std::queue < Event >	q;

	while ( mList.size() > 0 ) {
		q.push ( mList.front() );
		mList.pop ();
	}
	mList.push ( e );
	while ( q.size() > 0 ) {
		mList.push ( q.front() );
		q.pop ();
	}
}

EventQueue& EventQueue::operator= ( EventQueue &op )
{
	Event e;
	while ( op.size() > 0 ) {
		e = op.mList.front();
		mList.push ( e );
		op.mList.pop();
	}
	return *this;
}

void EventQueue::clear ()
{
	while ( mList.size() > 0 )
		pop ();
}

void EventQueue::pop ()
{
	if (mList.size()==0) return;
	mList.pop ();		// pop causes event & payload deletion!
}

void EventQueue::trace ()
{
	Event e;
	std::queue<Event> temp;

	if ( mTraceFile == 0x0 ) return;

	fflush ( mTraceFile );

	while (!mList.empty()) {
		e = mList.front ();
		fprintf ( mTraceFile, "%s ", e.getNameStr().c_str() );
		fflush ( mTraceFile );
		temp.push ( e );
		mList.pop ();
	}
	fprintf ( mTraceFile, "\n");
	fflush ( mTraceFile );
	while (!temp.empty() ) {
		e = temp.front ();
		mList.push ( e );
		temp.pop ();
	}
}


#ifdef BUILD_EVENT_POOLING

//------------------------------------------- EVENT POOLING [optional]

EventPool :: EventPool()
{
	// Preallocate empty bins.
	for (int n=0; n < BIN_CNT; n++) {
		mFullBins[n] = 0x0;
		mEmptyBins[n] = 0x0;
	}
	for (int n=0; n < BIN_CNT; n++)
		addBlock ( n );
}
EventPool :: ~EventPool()
{
	clear ();
}

void EventPool::clear ()
{
	blockPtr block;
	blockPtr next;

	for (int n=0; n < BIN_CNT; n++) {
		block = mFullBins[n];
		while (block != 0x0) {			// traverse linked list
			next = block->mNext;
			free ( block );
			block = next;
		}
		block = mEmptyBins[n];
		while (block != 0x0) {			// traverse linked list
			next = block->mNext;
			free ( block );
			block = next;
		}
		mFullBins[n] = 0x0;
		mEmptyBins[n] = 0x0;
	}
}

// Allocate.
// * Compiles down to 11 asm ops *
void* EventPool::allocItem ( int size )
{
	#ifdef DEBUG_MEMPOOL
		integrity();
	#endif

	size += sizeof(uint32_t);

	if ( size > MAX_POOL_SIZE ) {
		dbgprintf ( "  ERROR: Alloc size, %d, greater than MAX_POOL_SIZE (%d).\n", size, MAX_POOL_SIZE );
	}
	// include space for freebyte
	register blockPtr block = mEmptyBins[ getBin ( size ) ];	// Determine bin & block (given alloc size)

	if ( block->mPos == block->mEnd )		// Check if position is beyond end of block
		block = makeFull ( block );			//    ( if so, make full and make new block )

	block->mPos += block->mWidth;			// Get next item position
	block->mUsed++;							// Increment used items in block

	#ifdef MEM_CHECK
		if ( block->mUsed > block->mCount ) {
			dbgprintf ( "Bad end position. %p %p (%d/%d)\n", block->mPos, block->mEnd, block->mUsed, block->mCount );
			debug.Exit ( -2 );
		}
		dbgprintf ( 'memp', INFO, "  alloc: %p %d/%d, pos %d, item %p\n", block, block->mUsed, block->mCount, (char*) block->mPos - (char*) block, block->mPos );
		checkBlock ( block );
	#endif
	return block->mPos - block->mWidth;
}

// Free
void EventPool::freeItem ( void* item )
{
	#ifdef DEBUG_MEMPOOL
		integrity();
	#endif
	uint32_t* freeword = (uint32_t*) ((char*) item - sizeof(uint32_t));
	if ( *freeword == 0xFFFF ) { dbgprintf ( "Already freed, %p\n", item );  }
	register blockPtr block = (blockPtr) ((char*) item - *freeword);	// find the start of block

	*freeword = 0xFFFF;							// mark item as freed

	block->mUsed--;								// Decrement used items in block
	if ( block->mUsed == 0 ) {					// Check if used items goes to zero
		block = makeFree ( block );				//    (if so, free the block)
	}
	#ifdef MEM_CHECK
		printE ( 'memp', INFO, "   free: %p %d/%d, pos %d, item %p\n", block, block->mUsed, block->mCount, (char*) item - (char*) block, item );
		checkBlock ( block );
	#endif
}

// Add Block
blockPtr EventPool::addBlock ( int bin )
{
	// block = add block to front of EmptyBin list
	blockPtr block = (blockPtr) malloc ( getHeaderSize() + BLOCK_SIZE );	// Malloc block
	#ifdef MEM_CHECK
		dbgprintf ( 'memp', INFO,"  BLOCK addBlock: %p\n", block );
	#endif

	if ( block == 0x0 ) {
		dbgprintf ( "ERROR: Out of memory.\n" );
	}
	int w = getBlockWidth ( bin );				// Determine block bin, width & count
	int c = getItemCount ( bin );
	if ( mEmptyBins[bin] == 0x0 ) {				// Setup header
		setBlockHeader ( block, w, c, bin );
		mEmptyBins[bin] = block;
	} else {
		blockPtr next = mEmptyBins[bin];
		setBlockHeader ( block, w, c, bin);
		next->mPrev = block;
		block->mNext = next;
		mEmptyBins[bin] = block;
	}
	// Set freewords on all items
	// - Header item does not have a freeword
	// - Freeword is 4 bytes *before* the item, invading the end of previous item
	// - Freeword stores the negative offset to find the block (during freeItem)
	uint32_t* freeword = (uint32_t*) ((char*) block + getBlockWidth(bin) - sizeof(uint32_t));
	for (int n=0; n < c; n++) {
		*freeword = (char*) (freeword+1) - (char*) block;	// freeword+1 = next 4 bytes
		freeword += w / sizeof(uint32_t);				// next freeword (in units of uint32_t)
	}
	#ifdef MEM_CHECK
		integrity ();
	#endif
	return block;
}

// Make Block a Full block
blockPtr EventPool::makeFull ( blockPtr block )
{
	// block = guaranteed to be the front (head) of the EmptyBin linked list
	#ifdef MEM_CHECK
		dbgprintf ( "  BLOCK makeFull: %p\n", block );
	#endif

	int bin = block->mBin;
	blockPtr next_empty = block->mNext;

	// Make current block full
	//   (linked list add to front of Full)
	block->mbFull = true;					// Mark as full
	if ( mFullBins[bin] == 0x0 ) {
		block->mNext = 0x0;
	} else {
		blockPtr next = mFullBins[bin];		// Linked list insert into full bin
		next->mPrev = block;
		block->mNext = next;
	}
	block->mPrev = 0x0;
	mFullBins[bin] = block;

	// Make sure we still have empty blocks
	//   (linked list remove from front of Empty)
	if ( next_empty == 0x0 ) {
		mEmptyBins[bin] = 0x0;
		next_empty = addBlock ( bin );
	} else {
		next_empty->mPrev = 0x0;
		mEmptyBins[bin] = next_empty;
	}
	#ifdef MEM_CHECK
		integrity ();
	#endif
	return mEmptyBins[bin];
}


// Make Block a Free block
blockPtr EventPool::makeFree ( blockPtr block )
{
	// block = any block, in either Full or Empty list
	#ifdef MEM_CHECK
		dbgprintf ( "  BLOCK makeFree: %p\n", block );
	#endif

	int bin = block->mBin;
	blockPtr prev = block->mPrev;
	blockPtr next = block->mNext;

	if ( next != 0x0 ) {		// Linked list remove from free or full bin (block may be in either)
		next->mPrev = prev;
	}
	if ( prev != 0x0 ) {
		prev->mNext = next;
	} else {
		// First block
		if ( block->mbFull ) {
			mFullBins[bin] = 0x0;
		} else {
			mEmptyBins[bin] = 0x0;
		}
	}
	block->mPrev = 0x0;					// (just to be safe)
	block->mNext = 0x0;
	free ( block );						// Free the block

	#ifdef MEM_CHECK
		integrity ();
	#endif

	if ( mEmptyBins[bin] == 0x0 )
		addBlock ( bin );			// Allocate new empty bin if necessary
	return mEmptyBins[bin];
}

// Setup block header
void EventPool::setBlockHeader ( blockPtr block, int wid, int count, int bin )
{
	block->mMagic = 'LUNA';

	block->mPos = (itemPtr) block + wid;			// first item position skips the header item
	block->mEnd = (itemPtr) block + (count-1)*wid;
	block->mUsed = 0;
	block->mbFull = false;
	block->mBin = bin;
	block->mWidth = wid;
	block->mCount = count;
	block->mPrev = 0x0;
	block->mNext = 0x0;
}

int EventPool::checkBlock ( blockPtr block )
{
	int w = block->mWidth;
	int offset;

	uint32_t* freeword = (uint32_t*) ((char*) block + getBlockWidth(block->mBin) - sizeof(uint32_t));
	char* item = (char*) block + w;
	for (int n=0; n < block->mCount; n++) {
		if ( *freeword == 0xFFFF) continue;		// skip this one
		blockPtr block_chk = (blockPtr) (item - (char*) *freeword);	 // find the start of block
		if ( block_chk != block ) {
			dbgprintf ( "ERROR: Block %d, Item %d, Block (correct) %p, Freeword (wrong) %p\n", n, item, block, block_chk );
			return 0;
		}
		freeword += w / sizeof(uint32_t);		// next freeword (in units of uint32_t)
		item += w;								// next item
	}
	return block->mCount;
}


void EventPool::integrity ()
{
	char msg[50];
	blockPtr block;

	dbgprintf ( "MEMPOOL INTEGRITY\n" );
	int iOk;
	for (int n=0; n < BIN_CNT; n++) {
		block = mEmptyBins[n];
		dbgprintf ( "  Bin: %d, wid %d, cnt %d. ", n, getItemWidth(n), block->mUsed );
		iOk = 0;
		while (block != 0x0) {			// traverse linked list
            /*#ifdef _MSC_VER
			sprintf_s ( msg, 50, " %p (%d), ", block, block->mUsed );
            #else
			snprintf ( msg, 50, "%p (%d), ", block, block->mUsed );
            #endif
			debug.Print ( msg );			*/
			iOk += checkBlock ( block );
			block = block->mNext;
		}
		dbgprintf ( "%s. %d\n", iOk>0 ? "OK" : "BAD", iOk );

		/*printf ( "  Empty (bin %d, %05d): ", n, getItemWidth(n) );
		block = mEmptyBins[n];
		while (block != 0x0) {			// traverse linked list
            #ifdef _MSC_VER
			sprintf_s ( msg, 50, "%p (%d), ", block, block->mUsed );
            #else
			snprintf ( msg, 50, "%p (%d), ", block, block->mUsed );
            #endif
			debug.Print ( msg );
			checkBlock ( block );
			block = block->mNext;
		}
		debug.Print ( "\n" );*/
	}
}

// Compute number of allocated items in a bin
// (Add all items in both full and in use lists)
int EventPool::getAllocated ( int bin )
{
	blockPtr block;
	int bin_cnt=0;

	block = mFullBins[bin];
	while (block != 0x0) {
		bin_cnt += block->mUsed;
		block = block->mNext;
	}
	block = mEmptyBins[bin];
	while (block != 0x0) {
		bin_cnt += block->mUsed;
		block = block->mNext;
	}
	return bin_cnt;
}



//-- Standard log table. Matches width to bin
const char EventPool::logtable512[] = {
 0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9
};

unsigned char EventPool::getBin ( unsigned int v )
{
	// Find integer log base 2 of given width (v).
	return logtable512[ ((v-1) >> MIN_WIDTH_BITS) ];	// max ( 0, ceil[ log2(v / 64) ] )
}


#endif
