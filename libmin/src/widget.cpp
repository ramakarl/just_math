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

#include "widget.h"
#include "event_system.h"
#include "camera3d.h"
#include "geom_helper.h"
#include <string>

#include "nv_gui.h"

Widgets* gInterface = 0x0;

Widgets::Widgets()
{
	mFocus = -1;
	mFocusPos = 0;
}
void Widgets::ClearAll ( EventPool* pool )
{
	mFocus = -1;
	mPool = pool;
	for (int n=0; n < mWidgets.size(); n++) {
		if (mWidgets[n].img!=0x0)
			delete mWidgets[n].img;
	}
	mWidgets.clear ();
}

void Widgets::Init ( int w, int h )
{
	mWidgetSize3D = 1.0;			// size of 3D widget

	mCam = 0x0;
	mXres = w;
	mYres = h;
	mAct = ACT_OFF;
	mActWidget = -1;
	gInterface = this;		// singleton
}

void Widgets::Arrange ()
{
	Widget* w;

	for (int n=0; n < mWidgets.size(); n++) {
		
		w = &mWidgets[n];
		w->pos.x = w->rpos.x * mXres;
		w->pos.y = w->rpos.y * mYres;
		w->pos.z = w->rpos.z * mXres;
		w->pos.w = w->rpos.w * mYres;

		if ( hasOp( w, OP_FIT_W ) ) {
			w->pos.z = mXres - w->pos.x;
		}
		if ( hasOp( w, OP_FIT_H ) ) {
			w->pos.w = mYres - w->pos.w;
		}
	}
}


void Widgets::Create (std::string txt, ushort ops, Vector4DF rpos, Vector4DF foreclr, Vector4DF backclr, std::string img  )
{
	//while ( mWidgets.size() <= i )
	int i = AddWidget ();

	if ((ops & OP_INVISIBLE) == 0) ops |= OP_VISIBLE;
	if (txt.length() > 0) ops |= OP_TEXT;
	
	mWidgets[i].buttonState = (ops & OP_ON);
	mWidgets[i].opts = ops ;
	mWidgets[i].val = 0x0;	
	rpos.z += rpos.x;
	rpos.w += rpos.y;
	rpos.x /= mXres; rpos.y /= mYres;
	rpos.z /= mXres; rpos.w /= mYres;

	SetPos ( i, rpos );
	SetColor ( i, foreclr, backclr );	
	SetText ( i, txt );
	if ( !img.empty() )
		SetImage( i, img );
}
int Widgets::AddWidget ( bool is3D )
{	
	if ( is3D ) {
		Widget3D w3;
		w3.opts = 0;
		w3.pos = Vector3DF(0,0,0);
		w3.scale= Vector3DF(1,1,1);
		w3.clr = Vector4DF(1,1,1,1.f);
		mWidgets3D.push_back ( w3 );
		return mWidgets3D.size()-1;
	} else {
		Widget w;
		w.pos = Vector4DF(0,0,.1,.1);
		w.img = 0x0;
		w.opts = OP_VISIBLE;		
		w.buttonType = BTN_OFF;
		w.foreClr = Vector4DF(1,1,1,1.f);
		w.backClr = Vector4DF(1,0,0,1.f);	
		//w.bordClr = Vector4DF(.5f,.5f,.5f,1.f);		
		w.triggered = 0;
		mWidgets.push_back ( w );
		return mWidgets.size()-1;
	}
}
void Widgets::SetPos		( int i, Vector4DF rpos )
{
	mWidgets[i].rpos = rpos;
}
void Widgets::SetColor		( int i, Vector4DF fclr, Vector4DF bclr )
{
	mWidgets[i].foreClr = fclr;
	mWidgets[i].backClr = bclr;	
}
void Widgets::SetImage		( int i, std::string png_name )
{
	if (mWidgets[i].img != 0x0 ) {
		delete mWidgets[i].img;
		mWidgets[i].img = 0x0;
	}
	if (png_name.empty()) return;

	// load png
	mWidgets[i].img = new nvImg;
	char fname[1024];
	strcpy ( fname, png_name.c_str() );
	mWidgets[i].img->LoadPng ( fname );	
}

void Widgets::SetText		( int i, std::string txt )
{
	mWidgets[i].text = txt;
}
void Widgets::SetTextEntry	( int i, std::string txt)
{
	mWidgets[i].textEntry = txt;
	
	float y;
	mFocusPos = txt.length();
	mFocusX = mFocusPos;

	#ifdef USE_OPENGL
		getTextSize (txt.substr(0, mFocusPos).c_str(), mFocusX, y);		// update cursor position
	#endif
}
void Widgets::SetButtonType	( int i, int typ )
{
	mWidgets[i].buttonType = typ;
	mWidgets[i].buttonState = BTN_OFF;
}
void Widgets::SetButton		( int i, int state )
{
	mWidgets[i].buttonState = state;
}
void Widgets::SetVisible ( int i, bool on )
{
	SetOption ( i, i, OP_VISIBLE, on );
}
void Widgets::SetSlider(int i, float* v, float vmin, float vmax)
{
	mWidgets[i].val = v;
	mWidgets[i].vrange.Set(vmin, *v, vmax);
}

void Widgets::SetOption ( int start, int end, ushort op, bool on )
{
	if ( start < 0 ) start = 0;
	if ( start >= mWidgets.size()) start = mWidgets.size()-1;
	if ( end >= mWidgets.size()) end = mWidgets.size()-1;

	for (int n=start; n <= end; n++ )
		if ( on )
			mWidgets[n].opts |= op;		// set
		else
			mWidgets[n].opts &= ~op;	// clear
}

void Widgets::Draw ()
{
	float h, valx, entryx;
	Widget* w;
	char txt[1024];
	Vector4DF clr;

	#ifdef USE_OPENGL
		start2D ();

		// draw widgets
		for (int n=0; n < mWidgets.size(); n++) {
			w = &mWidgets[n];

			if ( hasOp(w, OP_VISIBLE) ) {
				drawFill ( w->pos.x, w->pos.y, w->pos.z, w->pos.w, w->backClr.x, w->backClr.y, w->backClr.z, w->backClr.w );	// background

				if ( w->buttonState == BTN_ON ) {
					if ( hasOp(w, OP_CLICK)) w->buttonState = BTN_OFF;
					drawFill ( w->pos.x, w->pos.y, w->pos.z, w->pos.w, 1, 1, 1, 0.75 );						// highlighting				
				}
				//drawRect ( w->pos.x, w->pos.y, w->pos.z, w->pos.w, w->bordClr.x, w->bordClr.y, w->bordClr.z, w->bordClr.w );	// border

				if ( hasOp(w, OP_TEXT) && !w->text.empty() ) {
					strncpy ( txt, w->text.c_str(), 1024 );					// *NOTE* Need to cache this for performance! (strncpy on every widget/frame)
					setText ( (w->pos.w - w->pos.y)*0.95f, 1 );
					drawText ( w->pos.x + 4, w->pos.y + 4, txt, w->foreClr.x, w->foreClr.y, w->foreClr.z, w->foreClr.w );
				}
				if ( hasOp(w, OP_TEXT_ENTRY) || hasOp(w, OP_SLIDER) ) {
					entryx = w->pos.x + 0.30 * (w->pos.z - w->pos.x);		// this widget has an entry portion (30% indented)
					drawFill(entryx, w->pos.y, w->pos.z, w->pos.w, 0.5, 0.5, 0.5, 0.5);				
					if (w->val != 0x0) {									// slider values
						valx = entryx + (*w->val - w->vrange.x) / (w->vrange.z - w->vrange.x) * (w->pos.z - entryx);					
						drawFill(entryx, w->pos.y + 2, valx, w->pos.w - 2, 1, 1, 1, 0.75);
					}
					if (!w->textEntry.empty()) {
						strncpy(txt, w->textEntry.c_str(), 1024);			// text entry values. *NOTE* Need to cache this for performance!
						setText((w->pos.w - w->pos.y) * 0.95f, 1);
						drawText(entryx, w->pos.y + 4, txt, w->foreClr.x, w->foreClr.y, w->foreClr.z, w->foreClr.w);
					}
				}
			
				if ( w->img != 0 ) drawImg ( w->img->getTex(), w->pos.x ,w->pos.y, w->pos.z, w->pos.w, 1,1,1,1);
			
		
			}
		}
		// draw cursor
		if (mFocus != -1) {
			w = &mWidgets[ mFocus ];
			if (hasOp(w, OP_VISIBLE)) {
				entryx = w->pos.x + 0.30 * (w->pos.z - w->pos.x);		// this widget has an entry portion (30% indented)
				drawFill(entryx + mFocusX, w->pos.y + 2, entryx + mFocusX + 2, w->pos.w - 2, 1, 1, 1, 1);
				drawRect(entryx, w->pos.y, w->pos.z, w->pos.w, 1, 1, 1, 1);
			}
		}

		//sprintf (txt, "Active: %d", mActWidget );
		//drawText ( 20, 180, txt, 1,0,0,1 );				// debug

		end2D();
	#endif
}

bool Widgets::OnMouse ( AppEnum button, AppEnum state, int mods, int x, int y, Widget3D& dw, bool& finish)
{
	// 2D widgets
	Widget* w;
	finish = false;
	mMod.z = -1;

    for (int n=0; n < mWidgets.size(); n++) {
		w = &mWidgets[n];
		if ( hasOp(w, OP_VISIBLE) ) {
			if ( x > w->pos.x && y > w->pos.y && x < w->pos.z && y < w->pos.w ) {
				if ( button == AppEnum::BUTTON_LEFT && state == AppEnum::BUTTON_PRESS ) {

					if ( hasOp(w, OP_TOGGLE) )	
						w->buttonState = 1 - w->buttonState;
					
					if ( hasOp(w, OP_CLICK) )	w->buttonState = 1;
					if ( hasOp(w, OP_SLIDER) && w->val != 0x0) {						
						mMod.x = x; mMod.y = y; mMod.z = n;
						w->vrange.y = *w->val;		// save starting val
						dbgprintf("Start slider: %d, %f", mActP.y, *w->val);
					}
					PushEvent ( 'wBtn', n, 0 );
				}
				return true;
			}
		}
	}

	// 3D widgets
	if ( mCam != 0x0 ) {		
		if ( button == AppEnum::BUTTON_LEFT && state == AppEnum::BUTTON_PRESS && mActWidget >= 0 ) {	// left-mouse click.. start a 3D interaction
			if ( OnAction3D ( mActWidget, x, y ) ) {
				mActWgOrig = mWidgets3D[mActWidget];		// make a copy of widget data
				return true;
			}
		}		
		if ( button == AppEnum::BUTTON_LEFT && state == AppEnum::BUTTON_RELEASE ) {						// left-mouse release.. finish a 3D interaction
			if ( mActWidget != -1 ) {
				dw.text = mWidgets3D[mActWidget].text;
				dw.sel = mWidgets3D[mActWidget].sel;
				dw.pos = mWidgets3D[mActWidget].pos - mActWgOrig.pos;									// pos delta
				dw.rot = mActWgOrig.rot.inverse() * mWidgets3D[mActWidget].rot; dw.rot.normalize();		// rotational delta
				dw.scale = mWidgets3D[mActWidget].scale - mActWgOrig.scale;								// scale delta
				finish = true;
			}
			return true;
		}
	}
	return false;
}

bool Widgets::OnMotion ( AppEnum button, int x, int y, int dx, int dy )
{
	Widget* w;
	if (mMod.z >= 0 && mMod.z < mWidgets.size() ) {
		w = &mWidgets[mMod.z];			// check currently active slider
		if (hasOp(w, OP_SLIDER) && hasOp(w, OP_VISIBLE)) {
			if (x > w->pos.x && y > w->pos.y && x < w->pos.z && y < w->pos.w) {
				if (button == AppEnum::BUTTON_LEFT) {
					float dv = (w->vrange.z - w->vrange.x) / (0.70f * (w->pos.z - w->pos.x));	// delta in value with respect to delta in mouse x
					dv = w->vrange.y + dv * (x - mMod.x);									// new value = original + change in value
					if (dv < w->vrange.x) dv = w->vrange.x;
					if (dv > w->vrange.z) dv = w->vrange.z;
					*w->val = dv;		// update value
					PushEvent('wBtn', mMod.z, 0);
					return true;
				}
			}
		}
	}
	// 3D widgets
	if ( mCam != 0x0 && mActWidget >= 0) {
		if ( button == AppEnum::BUTTON_NONE ) 		// hover			
			if ( OnAction3D ( mActWidget, x, y ) ) return true;

		if ( button == AppEnum::BUTTON_LEFT ) 		// drag			
			if ( OnUpdate3D ( mActWidget, x, y ) ) return true;
	}
	return false;
}

void Widgets::DisableWidget3D()
{
	SetOpt3D(mActWidget, OP_VISIBLE, false);	// hide previous
	mActWidget = -1;							// none
}

void Widgets::SetActiveWidget3D (int i, std::string name, Vector4DF clr )
{
	SetOpt3D( mActWidget, OP_VISIBLE, false);	// hide previous
	mActWidget = i;								// set new
	if (i != -1) {
		mWidgets3D[i].text = name;
		mWidgets3D[i].clr = clr;
		SetOpt3D(mActWidget, OP_VISIBLE, true);	// enable new
	}
}


bool Widgets::OnKeyboard ( int key )
{
	if ( mFocus == -1 || mFocus >= mWidgets.size() ) return false;

	// get current entry text (to be modified)
	// on the focused widget
	std::string txt = mWidgets[mFocus].textEntry;
	std::string ch; ch = key;
	float y;

	//dbgprintf ( "KEY: %d, %s\n", key, ch.c_str ());		//-- debugging
	
	switch (key) {
	case KEY_BACKSPACE:
		if (mFocusPos <= txt.length() && mFocusPos > 0) {
			txt = txt.substr(0, mFocusPos - 1) + txt.substr(mFocusPos);
			mFocusPos--;
		}
		break;
	case KEY_DELETE:
		if (mFocusPos < txt.length()) {
			txt = txt.substr(0, mFocusPos) + txt.substr(mFocusPos + 1);
		}
		break;
	case KEY_RIGHT:
		mFocusPos++;		if (mFocusPos >= txt.length()) mFocusPos = txt.length();
		break;
	case KEY_LEFT:
		mFocusPos--;		if (mFocusPos < 0) mFocusPos = 0;
		break;
	case KEY_HOME:
		mFocusPos = 0;
		break;
	case KEY_END:
		mFocusPos = txt.length();
		break;
	case KEY_ENTER:
		PushEvent('wKey', mFocus, key);
		break;
	default:
		// modify the entry text
		if (key >= 32 && key <= 126 ) {				// all standard keys: 0-9, a-z, A-Z, space, punct.
			if (mFocusPos >= txt.length()) {
				mFocusPos = txt.length();
				txt = txt + ch;				// append key
			}
			else {
				txt = txt.substr(0, mFocusPos) + ch + txt.substr(mFocusPos);		// insert key
			}			
			PushEvent('wKey', mFocus, key);
			mFocusPos++;		// advance the cursor
		} else {
			return false;		// KEY WAS NOT PROCESSED
		}
		break;
	}	
	mWidgets[mFocus].textEntry = txt;								// update widget text
	mFocusX = mFocusPos;
	#ifdef USE_OPENGL
		getTextSize(txt.substr(0, mFocusPos).c_str(), mFocusX, y);		// update cursor position
	#endif

	return true;
}

void Widgets::PushEvent ( eventStr_t name, int w, int k)
{
	// push event to OUT queue (back to app)
	Event e = new_event ( 128, 'app ', name, 0, mPool );
	e.attachInt ( w );
	e.attachInt ( k );
	e.bOwn = false;
	mOutEvents.push ( e );
}

bool Widgets::hasEvents ()
{
	return ( mOutEvents.size() > 0 );
}
Event Widgets::getNextEvent()
{
	Event e = mOutEvents.front ();
	e.bOwn = true;	
	mOutEvents.pop();
	e.startRead();
	return e;
}


//----------------------------------------- 3D Widgets

void Widgets::Create3D ( ushort ops, Vector3DF pos, Vector3DF angs, Vector3DF scal, Vector4DF clr, std::string txt )
{
	int i = AddWidget ( true );  // true=3D

	// invisible flag - default state of visibility. e.g. start out invisible
	//   visible flag - current state of visibility. e.g. currently visible, or NOT visible
	if ((ops & OP_INVISIBLE) == 0) ops |= OP_VISIBLE;			// not invisible. make visible.

	mWidgets3D[i].opts = ops;
	mWidgets3D[i].pos = pos;
	mWidgets3D[i].rot.set ( angs );
	mWidgets3D[i].scale = scal;
	mWidgets3D[i].pivot = scal;
	mWidgets3D[i].clr = clr;
	mWidgets3D[i].text = txt;
	if ( !txt.empty() ) mWidgets3D[i].opts |= OP_TEXT;
}

void Widgets::SetOpt3D ( int w, ushort op, bool on )
{
	if (w==-1) return;
	if ( on )
		mWidgets3D[w].opts |= op;
	else
		mWidgets3D[w].opts &= ~op;
}

#define max3(a,b,c)		( (a>b) ? ((a>c) ? a : c) : ((b>c) ? b : c) )

// Draw3D -
// This function draws a set of 3D GUI interface widgets
// Each widget is drawn with translation handles and rotation rings.
// Axes or rings are drawn highlighted if they are currently active (undergoing update)
void Widgets::Draw3D ()
{
	#ifdef USE_OPENGL

		Widget3D* w;
		Vector3DF p0, p1, p2, p3;
		Vector4DF gray (.7,.7,.7,1);
		Vector4DF act;

		float sz = mWidgetSize3D * std::min(1.0f, mCam->getOrbitDist() / 32.0f);				// widget size, limited by zoom
		float sc = sz * 0.025f;

		// activations
		act.x = (mAct==ACT_X) ? 2 : 1;
		act.y = (mAct==ACT_Y) ? 2 : 1;
		act.z = (mAct==ACT_Z) ? 2 : 1;
		act.w = (mAct==ACT_SCALE) ? 2 : 1;	

		start3D ( mCam );

		glLineWidth ( 2);

		for (int n=0; n < mWidgets3D.size(); n++) {
			w = &mWidgets3D[n];

			if ( hasOpt3D(n, OP_VISIBLE) ) {	

				p0 = w->pos;	// +  w->pivot * w->scale;
				p1.Set(sz, 0, 0); p1 = w->rot.rotateVec(p1);
				p2.Set(0, sz, 0); p2 = w->rot.rotateVec(p2);
				p3.Set(0, 0, sz); p3 = w->rot.rotateVec(p3);

				if (hasOpt3D(n, OP_MOVE)) {				
					drawCyl3D(p0, p0 + p1 * .8f, act.x * sc, act.x * sc, Vector4DF(1, 0, 0, 1));	drawCyl3D(p0 + p1 * .8f, p0 + p1, act.x * sc * 2.f, 0.0, Vector4DF(1, 0, 0, 1));
					drawCyl3D(p0, p0 + p2 * .8f, act.y * sc, act.y * sc, Vector4DF(0, 1, 0, 1));	drawCyl3D(p0 + p2 * .8f, p0 + p2, act.y * sc * 2.f, 0.0, Vector4DF(0, 1, 0, 1));
					drawCyl3D(p0, p0 + p3 * .8f, act.z * sc, act.z * sc, Vector4DF(0, 0, 1, 1));	drawCyl3D(p0 + p3 * .8f, p0 + p3, act.z * sc * 2.f, 0.0, Vector4DF(0, 0, 1, 1));
				}
				if (hasOpt3D(n, OP_SCALE)) {
					//drawCyl3D ( p0+p2*1.0f, p0+p2*1.1f, 0.0f, act.w*sc*2.f, Vector4DF(1,1,0,1) );
					drawCyl3D(p0 + p2 * 1.1f, p0 + p2 * 1.2f, act.w * sc * 2.f, act.w * sc * 2.f, Vector4DF(1, 1, 0, 1));
					drawCyl3D(p0 + p2 * 1.1f, p0 + p2 * 1.1f, act.w * sc * 2.f, 0.0f, Vector4DF(1, 1, 0, 1));
				}
				if (hasOpt3D(n, OP_ROTATE)) {
					drawCircle3D(p0, p0 + p1, sz, (mAct == ACT_ROTX) ? Vector4DF(1, 0, 0, 1) : gray);
					drawCircle3D(p0, p0 + p2, sz, (mAct == ACT_ROTY) ? Vector4DF(0, 1, 0, 1) : gray);
					drawCircle3D(p0, p0 + p3, sz, (mAct == ACT_ROTZ) ? Vector4DF(0, 0, 1, 1) : gray);
				}
				if ( hasOpt3D(n, OP_BOX))
					drawBox3DXform ( Vector3DF(0,0,0), Vector3DF(1,1,1), w->clr, w->xform );
			}
		}

		//drawBox3D ( mActP-Vector3DF(.002f,.002f,.002f), mActP+Vector3DF(.002f,.002f,.002f), 1,1,1,1 );		// action point (white)
		//drawBox3D ( mP2-Vector3DF(.002f,.002f,.002f), mP2+Vector3DF(.002f,.002f,.002f), 1,1,0,1 );		// debug point (yellow)

		end3D();
	#endif
}

// Action3D -
// This function detects interaction with a 3D GUI interface widget.
// Detects the hit condition with translation axes or with rotation rings.
bool Widgets::OnAction3D(int wi, int x, int y)
{
	Widget3D* w = &mWidgets3D[wi];

	Vector3DF c1 = mCam->getPos();
	Vector3DF c2 = c1 + mCam->inverseRay(x, y, mXres, mYres);
	Vector3DF p, p0, p1, p2, p3, q;
	float t;

	bool move = hasOpt3D(wi, OP_MOVE);
	bool scale = hasOpt3D(wi, OP_SCALE);
	bool rotate = hasOpt3D(wi, OP_ROTATE);

	float sz = mWidgetSize3D * std::min(1.0f, mCam->getOrbitDist() / 32.0f);				// widget size, limited by zoom

	float sc = sz * 0.02f * 16.f;			// axis hits
	float scr = sc; // /2.f;						// rotation hits
	float d;
	p0 = w->pos;
	p1.Set(sz, 0, 0); p1 = w->rot.rotateVec(p1);
	p2.Set(0, sz, 0); p2 = w->rot.rotateVec(p2);
	p3.Set(0, 0, sz); p3 = w->rot.rotateVec(p3);

	// y norm, xz plane
	p = intersectLinePlane(c1, c2, p0, p2);
	mActP = p;
	q = p0 + projectPointLine(p - p0, p1, t);										// project onto X-axis of motion
	if (move && p.Dist(q) < sc && t > 0 && t <= 1.0) { mAct = ACT_X;	return true; }		// hit X-handle
	q = p0 + projectPointLine(p - p0, p3, t);										// project onto Z-axis of motion
	if (move && p.Dist(q) < sc && t > 0 && t <= 1.0) { mAct = ACT_Z;	return true; }		// hit Z-handle
	d = p.Dist(p0);
	if (rotate && d > sz - scr && d < sz + scr) { mAct = ACT_ROTY;	return true; }				// hit Y-axis rotation ring

	// x norm, yz plane
	p = intersectLinePlane(c1, c2, p0, p1);
	mActP = p;
	q = p0 + projectPointLine(p - p0, p2, t);										// project onto Y-axis of motion
	if (p.Dist(q) < sc) {
		if (move && t > 0 && t <= 1.0) { mAct = ACT_Y;	return true; }				// hit Y-handle
		if (scale && t >= 1.1 && t <= 1.2) { mAct = ACT_SCALE; return true; }			// hit scaling handle
	}
	if (rotate) {
		// x norm, yz plane
		d = p.Dist(p0);
		if (d > sz - scr && d < sz + scr) { mAct = ACT_ROTX;	return true; }				// hit X-axis rotation ring

		// z norm, xy plane
		p = intersectLinePlane(c1, c2, p0, p3);
		mActP = p;
		d = p.Dist(p0);
		if (d > sz - scr && d < sz + scr) { mAct = ACT_ROTZ;	return true; }		// hit Z-axis rotation ring
	}
		
	mAct = ACT_OFF;
	return false;
}

// Update3D -
// This function computes the transformations for a 3D GUI interface widget,
// with translation along x/y/z axes, and rotation using x/y/z rings.
// Proper computation here uses the saved widget transformation state on mousedown (mActWgOrig),
// and then computes the stable translation or rotation delta from that.
bool Widgets::OnUpdate3D ( int wi, int x, int y )
{
	Widget3D* w = &mWidgets3D[wi];
	
	// get the camera ray
	Vector3DF c1 = mCam->getPos ();
	Vector3DF c2 = c1 + mCam->inverseRay (x, y, mXres, mYres);
	Vector3DF p, p0, p1, p2, p3, r, s;
	float t;
	Quaternion q;
	
	// get the widget local axes
	float sz = mWidgetSize3D * std::min(1.0f, mCam->getOrbitDist() / 32.0f) ;				// widget size, limited by zoom

	float d;
	p0 = mActWgOrig.pos;
	p1.Set(sz,0,0); p1 = mActWgOrig.rot.rotateVec(p1);		// x-norm
	p2.Set(0,sz,0); p2 = mActWgOrig.rot.rotateVec(p2);		// y-norm
	p3.Set(0,0,sz); p3 = mActWgOrig.rot.rotateVec(p3);		// z-norm

	switch ( mAct ) {
	case ACT_X:
		p = intersectLinePlane ( c1, c2, p0, p2 );	// intersect y-norm, xz plane
		p -= mActP;									// distance of translation
		p1.Normalize();
		s = projectPointLine ( p, p1, t );			// project onto X-axis of motion
		w->pos = mActWgOrig.pos + s;		
		break;
	case ACT_Y:
		p = intersectLinePlane ( c1, c2, p0, p1 );	// intersect x-norm, yz plane
		p -= mActP;									// distance of translation
		p2.Normalize();
		s = projectPointLine ( p, p2, t );			// project onto Y-axis of motion
		w->pos = mActWgOrig.pos + s;			
		break;
	case ACT_Z:
		p = intersectLinePlane ( c1, c2, p0, p2 );	// intersect y-norm, xz plane
		p -= mActP;									// distance of translation
		p3.Normalize();			
		s = projectPointLine ( p, p3, t );			// project onto Z-axis of motion
		w->pos = mActWgOrig.pos + s;			
		break;
	case ACT_SCALE:
		p = intersectLinePlane ( c1, c2, p0, p1 );	// intersect x-norm, yz plane
		p -= mActP;									// distance of translation
		p2.Normalize();								// project onto Y-axis of motion
		s = projectPointLine ( p, p2, t );			
		w->scale = mActWgOrig.scale * (t/sz+1.0f);			// *same motion as Y-translation, but affects scaling*
		break;
	case ACT_ROTX:					
		p = intersectLinePlane ( c1, c2, p0, p1 ); p -= p0; p.Normalize();	// normalize movement angle
		r = mActP - p0; r.Normalize();										// normalize original angle
		p1.Normalize();														// normalize axis of rotation (*after* using it for intersection)
		s = r.Cross(p); s.Normalize();										// use cross product for +/- determination		
		q.fromAngleAxis ( (s.Dot(p1)>0 ? 1 : -1)*acos(p.Dot(r)), p1 );		// angle of X-rotation
		w->rot = q * mActWgOrig.rot;	
		w->rot.normalize();
		break;
	case ACT_ROTY:
		p = intersectLinePlane ( c1, c2, p0, p2 ); p -= p0; p.Normalize();	// normalize movement angle
		r = mActP - p0; r.Normalize();										// normalize original angle
		p2.Normalize();														// normalize axis of rotation (*after* using it for intersection)
		s = r.Cross(p); s.Normalize();										// use cross product for +/- determination		
		q.fromAngleAxis ( (s.Dot(p2)>0 ? 1 : -1)*acos(p.Dot(r)), p2 );		// angle of Y-rotation
		w->rot = q * mActWgOrig.rot;	
		w->rot.normalize();
		break;
	case ACT_ROTZ:
		p = intersectLinePlane ( c1, c2, p0, p3 ); p -= p0; p.Normalize();	// normalize movement angle
		r = mActP - p0; r.Normalize();										// normalize original angle
		p3.Normalize();														// normalize axis of rotation (*after* using it for intersection)
		s = r.Cross(p); s.Normalize();										// use cross product for +/- determination		
		q.fromAngleAxis ( (s.Dot(p3)>0 ? 1 : -1)*acos(p.Dot(r)), p3 );		// angle of Z-rotation
		w->rot = q * mActWgOrig.rot;	
		w->rot.normalize();
		break;
	};

	return true;
}

bool Widgets::IntersectBox3D ( Vector3DF pos, Vector3DF piv, Vector3DF scale, Quaternion rot, float x, float y )
{
	// get the camera ray
	Vector3DF c1 = mCam->getPos ();
	Vector3DF c2 = mCam->inverseRay (x, y, mXres, mYres);
	
	rot = rot.inverse();
	c1 -= pos; c1 = rot.rotateVec(c1);
	c2 = rot.rotateVec(c2);	c2.Normalize();

	float t;

	return  intersectLineBox ( c1, c1+c2, piv*scale, scale, t );
}
