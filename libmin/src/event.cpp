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
#include "event.h"

static char mbuf [ 16384 ];


eventStr_t strToName (std::string str )
{
	char buf[5];
	strcpy ( buf, str.c_str() );
	eventStr_t name;
	((char*) &name)[3] = buf[0];
	((char*) &name)[2] = buf[1];
	((char*) &name)[1] = buf[2];
	((char*) &name)[0] = buf[3];
	return name;
}

std::string nameToStr ( eventStr_t name )			// static function
{
	char buf[5];
	buf[0] = ((char*) &name)[3];
	buf[1] = ((char*) &name)[2];
	buf[2] = ((char*) &name)[1];
	buf[3] = ((char*) &name)[0];
	buf[4] = '\0';
	return std::string ( buf );
}

Event::Event ()
{
	memcpy(mScope, "emem", 4); mScope[4]='\0';
	mRefs = 0;
	mTarget = 0;
	mName = 0;
	mDataLen = 0;
	mMax = 0;
	bOwn = false;
	bDestroy = false;
	mData = 0x0;
	mPos = 0x0;	
	mOwner = 0x0;
	mConnection = -1;

	// check member variable structure (important!)
	int headersz = (char*) &mData - (char*) &mDataLen;
	if ( headersz != Event::staticSerializedHeaderSize() ) {
		//dbgprintf ( "  ERROR: Event member header not %d on this platform.\n", Event::staticSerializedHeaderSize() );
		dbgprintf ( "  ERROR: Event member header not %d on this platform.\n", Event::staticSerializedHeaderSize() );
	}
}

Event::Event ( const Event& src )
{
	copyEventVars ( this, &src );
	bOwn = false;
	bDestroy = false;
}

extern void delete_event ( Event& e );

Event::~Event ()
{
	delete_event ( *this );
}



void Event::copyEventVars ( Event* dst, const Event* src )
{
	// this is NOT a deep copy. the data pointed to by mData is NOT copied.
	// only the member variables reference to data are transered.
	dst->mDataLen = src->mDataLen;
	dst->mName = src->mName;
	dst->mTarget = src->mTarget;
	dst->mConnection = src->mConnection;
	dst->mTimeStamp = src->mTimeStamp;
	dst->mData = src->mData;
	dst->mRefs = src->mRefs;
	dst->mSrcSock = src->mSrcSock;
	dst->mTargetID = src->mTargetID;
	dst->mMax = src->mMax;
	dst->mOwner = src->mOwner;
	dst->bOwn = src->bOwn;
	dst->bDestroy = src->bDestroy;
	for (int n=0; n < 5; n++)
		dst->mScope[n] = src->mScope[n];
	dst->mPos = src->mPos;
}

Event& Event::operator= ( const Event* src )
{
	copyEventVars ( this, src );	// dst.mData = src.mData. NOT a deep copy of payload		
	bOwn = false;					// dest ownership
	return *this;
}
// use this to pass Events between functions
// - Events in local scope are returned using copy operator
Event& Event::operator= ( const Event& src )
{
	copyEventVars ( this, &src );	// dst.mData = src.mData. NOT a deep copy of payload		
	bOwn = false;					// dest ownership
	return *this;
}

void Event::acquire ( Event& src)
{
	copyEventVars ( this, &src );	// NOT a deep copy
	bOwn = true;					// dest becomes new owner
	src.bOwn = false;				// src no longer owner
}


void Event::attachInt ( int i)
{
	if ( mDataLen+sizeof(int) > mMax ) expand ( mDataLen*2 );
	* (int*) mPos = i;
	mPos += sizeof(int);
	mDataLen += sizeof(int);
}
void Event::attachUChar (unsigned char i)
{
	if ( mDataLen+sizeof(char) > mMax ) expand ( mDataLen*2 );
	* (unsigned char*) mPos = i;
	mPos += sizeof(char);
	mDataLen += sizeof(char);
}
void Event::attachShort (short i)
{
	if ( mDataLen+sizeof(signed short) > mMax ) expand ( mDataLen*2 );
	* (signed short*) mPos = i;
	mPos += sizeof(signed short);
	mDataLen += sizeof(signed short);
}

void Event::attachUShort ( unsigned short i)
{
	if ( mDataLen+sizeof(short) > mMax ) expand ( mDataLen*2 );
	* (unsigned short*) mPos = i;
	mPos += sizeof(signed short);
	mDataLen += sizeof(signed short);
}

void Event::attachUInt (unsigned int i)
{
	if ( mDataLen+sizeof(int) > mMax ) expand ( mDataLen*2 );
	* (unsigned int*) mPos = i;
	mPos += sizeof(int);
	mDataLen += sizeof(int);
}
void Event::attachULong (unsigned long i)
{
	if ( mDataLen+sizeof(unsigned long) > mMax ) expand ( mDataLen*2 );
	* (unsigned long*) mPos = i;
	mPos += sizeof(unsigned long);
	mDataLen += sizeof(unsigned long);
}

void Event::attachInt64 (xlong i)
{
	if ( mDataLen+sizeof(xlong) > mMax ) expand ( mDataLen*2 );
	* (xlong*) mPos = i;
	mPos += sizeof(xlong);
	mDataLen += sizeof(xlong);
}
void Event::attachFloat (float f)
{
	if ( mDataLen+sizeof(float) > mMax ) expand ( mDataLen*2 );
	* (float*) mPos = f;
	mPos += sizeof(float);
	mDataLen += sizeof(float);
}

void Event::attachDouble (double f)
{
	if ( mDataLen+sizeof(double) > mMax ) expand ( mDataLen*2 );
	* (double*) mPos = f;
	mPos += sizeof(double);
	mDataLen += sizeof(double);
}

void Event::attachBool ( bool b)
{
	if ( mDataLen+sizeof(bool) > mMax ) expand ( mDataLen*2 );
	* (bool*) mPos = b;
	mPos += sizeof(bool);
	mDataLen += sizeof(bool);
}
void Event::attachVec4 ( Vector4DF v )
{
	attachFloat ( v.x );
	attachFloat ( v.y );
	attachFloat ( v.z );
	attachFloat ( v.w );
}


void Event::attachStr (std::string str )
{
	int len = str.length();
	attachInt ( len );
	if ( str.length() > 0 ) {
		if ( mDataLen + len > mMax ) expand ( mMax*2 + len );
		memcpy ( (char*) mPos, str.c_str(), len );
		mPos += len;
		mDataLen += len;
	}
}
void Event::attachPrintf (const char* format, ... )
{
	char buf[8192];
	va_list argptr;
    va_start(argptr, format);
    vsprintf( buf, format, argptr);
    va_end(argptr);
	attachStr ( buf );
}

void Event::attachMem (char* buf, int len )
{
	attachInt ( len );
	if ( mDataLen + len > mMax ) expand ( mMax*2 + len );
	memcpy ( mPos, buf, len );
	mPos += len;
	mDataLen += len;
}
void Event::attachBuf (char* buf, int len )
{
	if ( mDataLen + len > mMax ) expand ( mMax*2 + len );
	memcpy ( mPos, buf, len );
	mPos += len;
	mDataLen += len;
}
void Event::attachFromFile  (FILE* fp, int len )
{
	if ( mDataLen + len > mMax ) expand ( len );
	fread ( mPos, 1, len, fp );
	mPos += len;
	mDataLen += len;
}

void Event::attachBufAtPos (int pos, char* buf, int len )
{
	if ( pos+len > mMax  )
		expand ( imax(mMax*2, pos+len) );

	mPos = getData() + pos;
	memcpy ( mPos, buf, len );
	mPos += len;
	if ( pos+len > mDataLen ) mDataLen = pos+len;
}

Vector4DF Event::getVec4 ()
{
	Vector4DF v;
	v.x = getFloat ();
	v.y = getFloat ();
	v.z = getFloat ();
	v.w = getFloat ();
	return v;
}

void Event::writeUShort (int pos, unsigned short i)
{
	* (unsigned short*) (getData() + pos) = i;
}

int Event::getInt ()
{
	int i = * (int*) mPos;
	mPos += sizeof(int);
	return i;
}
unsigned char Event::getUChar ()
{
	unsigned char i = * (unsigned char*) mPos;
	mPos += sizeof(unsigned char);
	return i;
}
signed short Event::getShort ()
{
	signed short i = * (signed short*) mPos;
	mPos += sizeof(signed short);
	return i;
}
unsigned short Event::getUShort ()
{
	unsigned short i = * (unsigned short*) mPos;
	mPos += sizeof(unsigned short);
	return i;
}
unsigned int Event::getUInt ()
{
	unsigned int i = * (unsigned int*) mPos;
	mPos += sizeof(unsigned int);
	return i;
}
unsigned long Event::getULong ()
{
	unsigned long i = * (unsigned long*) mPos;
	mPos += sizeof(unsigned long);
	return i;
}

xlong Event::getInt64 ()
{
	xlong i = * (xlong*) mPos;
	mPos += sizeof(xlong);
	return i;
}
float Event::getFloat ()
{
	float f = * (float*) mPos;
	mPos += sizeof(float);
	return f;
}
double Event::getDouble ()
{
	double d = * (double*) mPos;
	mPos += sizeof(double);
	return d;
}
bool Event::getBool ()
{
	bool b = * (bool*) mPos;
	mPos += sizeof(bool);
	return b;
}

std::string Event::getStr ()
{
	if ( mPos >= getData() + mDataLen ) return "EVENT OVERFLOW";
	int i = getInt ();
	if ( i > mDataLen ) {
		dbgprintf ("ERROR: Event string length is corrupt.\n");
		return "ERROR";
	}
	if ( i > 0 ) {
		// dbgprintf  ( "  len: %d npos: %d\n", i, (int) (mPos-getData()) );
		memcpy ( mbuf, mPos, i ); *(mbuf + i) = '\0';
		mPos += i;
		if ( mPos >= getData() + mDataLen ) mPos = getData() + mDataLen;
		return std::string ( mbuf );
	}
	return "";
}

void Event::getStr (char* str)
{
	if ( mPos >= getData() + mDataLen ) {strcpy(str, "EVENT OVERFLOW"); return; }
	int i = getInt ();
	if ( i > 0 ) {
		strncpy ( str, mPos, i ); str[i] = '\0';
		mPos += i;
		if ( mPos >= getData() + mDataLen ) mPos = getData() + mDataLen;
	}
}

void Event::getMem ( char* buf, int maxlen )
{
	int len = getInt ();
	if ( mPos >= getData() + mDataLen ) return;
	memcpy ( buf, mPos, imin ( len, maxlen ) );
	mPos += len;
}
void Event::getBuf ( char* buf, int len )
{
	if ( mPos >= getData() + mDataLen ) return;
	memcpy ( buf, mPos, len );
	mPos += len;
}

void Event::getBufAtPos ( int pos, char* buf, int len )
{
	memcpy ( buf, getData() + pos, len ) ;
}

void Event::startRead ()
{
	mPos = mData;
}
void Event::startWrite ()
{
	mPos = mData;
	mDataLen = 0;
}

std::string	Event::getNameStr ()
{
	return nameToStr ( getName () );
}
std::string	Event::getSysStr ()
{
	return nameToStr ( mTarget );
}
char* Event::serialize ()
{
	// start of serialized header information (member vars) for this event
	char* header = (char*) this + Event::staticOffsetLenInfo();

	// memory location where data is being prepared with payload
	char* serial_data = mData - Event::staticSerializedHeaderSize();

	// transfer current serialized header values into payload area
	memcpy ( serial_data, header, Event::staticSerializedHeaderSize() );

	// data is now complete. attachments in payload are already seralized

	return serial_data;
}

void Event::deserialize (char* buf, int len )
{
	// NOTE: Incoming data includes the serialized event header
	//  (so, we cannot use attachBuf to do this)
	char* serial_data = mData - Event::staticSerializedHeaderSize();
	int serial_len = len;		// incoming length is the actual buffer size from network (amount of data)

	memcpy ( serial_data, buf, serial_len );

	// Transfer serialized header into member variables
	char* header = (char*) this + Event::staticOffsetLenInfo();
	memcpy ( header, serial_data, Event::staticSerializedHeaderSize() );

	mDataLen = len -  Event::staticSerializedHeaderSize();

	mPos = (serial_data + Event::staticSerializedHeaderSize()) + mDataLen;
}

void Event::setTime ( unsigned long t )
{
	mTimeStamp.SetSJT ( t );
}

extern void expand_event ( Event& e, size_t size );

void Event::expand ( int size)
{
	expand_event ( *this, (size_t) size );
}

