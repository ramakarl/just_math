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
#ifndef DEF_EVENT_H
	#define DEF_EVENT_H

    #include "common_defs.h"
	#include "vec.h"
	#include "timex.h"
	#include <string>

	#define NULL_TARGET		65535
	#define ID(y)		( (const xlong) *( (const xlong*) y ) )		// for 64-bit names, but not a const expression

	// Event typedefs
	typedef TimeX			timeStamp_t;
	typedef	uint32_t		eventStr_t;
	typedef unsigned long	netIP;
	typedef signed int		netPort;
	typedef signed int		netSock;
	typedef uint16_t		sysID_t;			// locally assigned ID
	typedef	uint32_t		objType;
	typedef	uint8_t			datType;

	class EventPool;

	// Event names
	HELPAPI std::string	nameToStr ( eventStr_t name );
	HELPAPI eventStr_t	strToName (	std::string str );

	// Event
	struct HELPAPI CACHE_ALIGNED Event {
	public:
		Event ();		
		Event ( const Event& src );
		Event& operator= ( const Event* op );	// assignment (not acquire)
		Event& operator= ( const Event& op );		
		~Event ();
		void copyEventVars ( Event* dst, const Event* src );
		void acquire ( Event& esrc);			// acquire - transfer of ownership

		// Event Accessors
		std::string			getNameStr ();
		std::string			getSysStr ();
		inline eventStr_t	getName ()			{ return mName; }
		inline eventStr_t	getTarget ()		{ return mTarget; }
		inline sysID_t		getTargetID ()		{ return mTargetID; }
		inline timeStamp_t	getTimeStamp()		{ return mTimeStamp; }
		inline EventPool*	getPool()			{ return mOwner; }
		inline void			set ( eventStr_t targ, eventStr_t name ) { mTarget = targ; mName = name; }
		inline void			setName ( eventStr_t x )		{ mName = x; }
		inline void			setTarget ( eventStr_t x )		{ mTarget = x; }
		inline void			setTargetID ( sysID_t t )		{ mTargetID = t; }
		inline void			setTimeStamp ( timeStamp_t t )	{ mTimeStamp = t; }

		// Event Reference counting
		inline int			incRefs ()			{ return ++mRefs; }
		inline int			decRefs ()			{ return --mRefs; }
		inline int			getRefs ()			{ return mRefs; }

		// Data Access
		void				startRead ();
		void				startWrite ();
		void				expand ( int s );
		char*				serialize ();
		void				deserialize ( char* buf, int len );		
		void				rescope ( char* scope )		{ memcpy ( mScope, scope, 4 ); mScope[4]='\0'; }
		//int				getEventLenOffs ()			{ return int((char*) &mDataLen - (char*) &mTarget); }
		char*				getData ()					{ return mData; }
		char*				getPos()					{ return mPos; }
		unsigned long		getPosInt()					{ return mPos - mData; }
		char*				getEndPos ()				{ return getData() + mMax; }
		bool				isEnd ()					{ return mPos >= getData() + mDataLen;  }
		bool				isEmpty ()					{ return mData == 0x0; }
		int					getDataLength ()			{ return mDataLen; }	// daata payload
		int					getPayloadLength()			{ return mDataLen; }	// synonym
		int					getSerializedLength ()		{ return mDataLen + Event::staticSerializedHeaderSize(); }	// length of network packet data
		char*				getSerializedData ()		{ return mData - Event::staticSerializedHeaderSize(); }

		// Get/set
		inline void			setDataLength (int n )		{ mDataLen = n; }
		inline void			setPos (int offs)			{ mPos = mData + offs; }
		inline void			setPos (char* pos )			{ mPos = pos; }
		inline timeStamp_t	getTime ()					{ return mTimeStamp; }
		inline void			setTime ( timeStamp_t t )	{ mTimeStamp = t; }
		void				setTime ( unsigned long t );
		inline void			setConnection ( unsigned short id ) { mConnection = id; }
		inline ushort		getConnection ()			{ return mConnection; }
		inline void			setSrcIP ( netIP ip )		{ mSrcIP = ip; }
		inline void			setSrcSock ( netSock s )	{ mSrcSock = s; }
		inline netIP		getSrcIP ()					{ return mSrcIP; }
		inline netSock		getSrcSock ()				{ return mSrcSock; }

		// Data Attach/Retrieve
		void				attachBool		(bool b);
		void				attachInt		(int i);
		void				attachShort		(signed short i );
		void				attachUChar		(unsigned char i );
		void				attachUShort	(unsigned short i );
		void				attachULong		(unsigned long i );
		void				attachUInt		(unsigned int i );
		void				attachInt64		(xlong i);
		void				attachFloat		(float f);
		void				attachDouble	(double d);
		void				attachStr		(std::string str );
		void				attachPrintf	(const char* format, ... );
		void				attachVec4		(Vector4DF i );
		void				attachMem		(char* buf, int len );
		void				attachBuf		(char* buf, int len );
		void				attachBufAtPos	(int pos, char* buf, int len );
		void				writeUShort		(int pos, unsigned short i );  // does not increase length
		void				attachFromFile  (FILE* fp, int len );

		bool				getBool ();
		int					getInt ();
		signed short		getShort ();
		unsigned char		getUChar ();
		unsigned short		getUShort ();
		unsigned int		getUInt ();
		unsigned long		getULong ();
		xlong				getInt64 ();
		float				getFloat ();
		double				getDouble ();
		Vector4DF			getVec4 ();
		std::string			getStr ();
		void				getStr (char* str );
		void				getMem (char* buf, int len );
		void				getBuf (char* buf, int len );
		void				getBufAtPos (int pos, char* buf, int len );

		// Serialized header length. Must match platform size of member vars
		static int		staticSerializedHeaderSize()	{ return 2*sizeof(int) + 2*sizeof(eventStr_t) + sizeof(timeStamp_t); }
		static int		staticOffsetLenInfo()			{ return 0; }   // <-- assumes mDataLen is first

		// **** NOTE ***
		// !! ORDER OF MEMBERS IS IMPORTANT HERE !!
		// Serialized first. mDataLen first member.

		// Serialized members
		// Header:														Offset	Bytes
		int				mDataLen;			// Event length				32+ 0	4			// <-- offsetDataLen (!)
		eventStr_t		mName;				// Event name				32+ 4	4
		eventStr_t		mTarget;			// Event target system		32+ 8	4
		int				mConnection;		// Event connection			32+ 12	4
		timeStamp_t		mTimeStamp;			// Event time stamp			32+ 16	8
		//																56      24 byte		// <-- offsetHeader (!)
		// Data payload is pre-pended with serialized members
		char*			mData;				// Data payload (elsewhere in memory)

		// Non-serialized members
		ushort			mRefs;				// Ref counting				(max: 65535)
		ushort			mSrcSock;			// Source Socket			(max: 65535)
		netIP			mSrcIP;				// Source IP
		sysID_t			mTargetID;			// Target ID
		int				mMax;				// Data max
		EventPool*		mOwner;				// Memory pool owner
		bool			bOwn;				// Owner info
		bool			bDestroy;			// Destroy
		char			mScope[5];			// Scope info
		char*			mPos;				// Data pos
	};



#endif
