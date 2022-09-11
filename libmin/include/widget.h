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

#ifndef DEF_WIDGET
	#define DEF_WIDGET

	#include "nv_gui.h"
	#include "event_system.h"
	#include "vec.h"
	#include "quaternion.h"
	#include "main_includes.h"

	#define BTN_OFF			0
	#define BTN_ON			1		

	#define ACT_OFF			0
	#define ACT_X			1
	#define ACT_Y			2
	#define ACT_Z			3
	#define ACT_ROTX		4
	#define ACT_ROTY		5
	#define ACT_ROTZ		6
	#define ACT_SCALE		7

	#define OP_EMPTY		0
	#define OP_ON			1
	#define OP_ENABLED		2
	#define OP_VISIBLE		4
	#define OP_INVISIBLE	8
	#define OP_FIT_W		16
	#define OP_FIT_H		32
	#define OP_TEXT			64
	#define OP_TEXT_ENTRY	128	
	#define OP_CLICK		256
	#define OP_BUTTON		256
	#define OP_TOGGLE		512
	#define OP_SLIDER		1024
	#define OP_BOX			2048		// 3D options
	#define OP_MOVE			4096
	#define OP_SCALE		8192
	#define OP_ROTATE		16384

	struct HELPAPI Widget {
		Vector4DF		rpos;			// relative position
		Vector4DF		pos;			// abs. screen coordinates		
		ushort			opts;	
		Vector4DF		foreClr;		// foreground (text)	
		Vector4DF		backClr;		// background
		std::string		text;		
		std::string		textEntry;		
		nvImg*			img;			// icon
		ushort			buttonType;		// button type
		ushort			buttonState;	// button state
		ushort			triggered;
		ushort			parent;
		ushort			child, next;
		float*			val;
		Vector3DF		vrange;
	};

	struct HELPAPI Widget3D {
		void UpdateXform() {
			xform.Identity();		
			xform.Translate ( pos );				
			xform.Rotate ( rot.getMatrix() );				
			xform.Scale ( scale );		
			xform.Translate ( pivot );	
		}
		ushort			opts;
		Vector3DF		rpos;		// relative position [0,1]
		Vector3DF		pos;		// absolute position [Xres,Yres]
		Quaternion		rot;
		Vector3DF		scale;
		Vector3DF		pivot;
		Vector3DI		sel;
		Matrix4F		xform;
		Vector4DF		clr;
		std::string		text;
	};

	class HELPAPI Widgets {
	public:
		Widgets();
		void ClearAll (  EventPool* pool );

		// 2D Setup
		void Init			( int w, int h );
		void Create 		( std::string txt, ushort ops, Vector4DF rpos, Vector4DF fclr, Vector4DF bclr, std::string img="");		
		int  AddWidget		( bool is3D=false );
		void Arrange		();
		void SetPos			( int i, Vector4DF pos );
		void SetOption		( int start, int end, ushort op, bool on_off );
		void SetColor		( int i, Vector4DF bclr, Vector4DF border );
		void SetImage		( int i, std::string png_name );
		void SetText		( int i, std::string txt );
		void SetTextEntry	( int i, std::string txt );
		void SetButtonType	( int i, int typ );
		void SetButton		( int i, int state );
		void SetVisible		( int i, bool vis );
		void SetVisible 	( int start, int end, bool vis );	// make multiple visible
		void SetSlider		( int i, float* v, float vmin, float vmax);
		void SetFocus 		( int i=-1 )	{ mFocus = i; }
		bool hasOp			( Widget* w, ushort op )	 { return (w->opts & op)!=0; }
		Widget* getWidget(int i)	{ return &mWidgets[i]; }

		// 2D Drawing
		void Draw ();		

		// Event handling
		bool OnMouse ( AppEnum button, AppEnum state, int mods, int x, int y, Widget3D& dw, bool& finish);		// called from app on mouse activity
		bool OnMotion ( AppEnum button, int x, int y, int dx, int dy );
		bool OnKeyboard ( int key );
		void PushEvent ( eventStr_t name, int w, int k );							// called by widgets to push event back to app
		bool hasEvents ();
		Event getNextEvent();														// get next outgoing event to app

		// 3D Widgets
		void Create3D		( ushort ops, Vector3DF pos, Vector3DF angs, Vector3DF scal, Vector4DF clr, std::string txt );		
		void Draw3D			();
		void SetOpt3D		( int w, ushort opt, bool on );
		bool hasOpt3D		( int w, ushort op )	{ return (mWidgets3D[w].opts & op)!=0; }
		void SetRes			( int x, int y )	{ mXres=x; mYres=y; Arrange(); }
		void SetCamera3D	( Camera3D* cam )	{ mCam = cam; }
		bool OnAction3D		( int w, int x, int y );
		bool OnUpdate3D		( int w, int x, int y );
		void SetText3D		(int w, std::string txt) { mWidgets3D[w].text = txt; }
		bool IntersectBox3D ( Vector3DF pos, Vector3DF piv, Vector3DF scale, Quaternion rot, float x, float y );
		Widget3D* getWidget3D( int i )	{ return &mWidgets3D[i]; }
		bool isActive3D()	{ return mActWidget >= 0; }
		void SetActiveWidget3D (int i=-1, std::string name="", Vector4DF clr=Vector4DF(1,1,1,1));
		void DisableWidget3D();
		Vector3DF getCenter3D ( int i )	{ return mWidgets3D[i].pos; }

	public:
		int						mXres, mYres;
		std::vector< Widget >	mWidgets;
		std::vector< Widget3D > mWidgets3D;
		int						mFocus, mFocusPos;
		float					mFocusX;

		EventPool*				mPool;
		EventQueue				mOutEvents;
		EventQueue				mInEvents;

		Camera3D*				mCam;
		char					mAct;
		int						mActWidget;		
		Vector3DF				mActP, mP2;
		Widget3D				mActWgOrig;
		float					mWidgetSize3D;
		Vector3DI				mMod;
	};

	HELPAPI extern Widgets* gInterface;

#endif