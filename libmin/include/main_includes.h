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

#ifndef MAIN_INCLUDES
	#define MAIN_INCLUDES

	enum AppEnum {
		BUTTON_NONE = 0,					// mouse states
		BUTTON_PRESS = 1,
		BUTTON_RELEASE = 2,
		BUTTON_REPEAT = 3,

		BUTTON_LEFT = 4,					// mouse buttons
		BUTTON_RIGHT = 5,
		BUTTON_MIDDLE = 6,

		ACTION_DOWN = 0,					// mobile actions
		ACTION_MOVE = 1,
		ACTION_UP = 2,
		ACTION_CANCEL = 3,
		GESTURE_SINGLE_TAP = 4,
		GESTURE_DOUBLE_TAP = 5,
		GESTURE_SCALE_BEGIN = 6,
		GESTURE_SCALE = 7,
		GESTURE_SCALE_END = 8,
		ACTION_GLIDE = 9,
		SOFT_KEY_PRESS = 10,

		EVT_XTARGET = 0,
		EVT_YTARGET = 1,
		EVT_XFOCUS = 2,
		EVT_YFOCUS = 3,
		EVT_XSPAN = 4,
		EVT_YSPAN = 5,

		UNDEF = 255
	};
	struct guiEvent {
		int typeOrdinal;
		float xtarget;
		float ytarget;
		float xfocus; // the center of a pinch gesture
		float yfocus;
		float xspan; // the width of a pinch gesture
		float yspan;
	};

	// non-character keys (non printing)
	#define	KEY_UNKNOWN     		-1
	#define KEY_BACKSPACE          8			// keys with ascii equivalents
	#define KEY_TAB                9	
	#define KEY_ENTER              13	
	#define KEY_ESCAPE             27
	#define KEY_DELETE             127

	#define KEY_WORLD_1            161			// virtual keys (non-ascii)
	#define KEY_WORLD_2            162
	#define KEY_INSERT             260	
	#define KEY_RIGHT              262
	#define KEY_LEFT               263
	#define KEY_DOWN               264
	#define KEY_UP                 265
	#define KEY_PAGE_UP            266
	#define KEY_PAGE_DOWN          267
	#define KEY_HOME               268
	#define KEY_END                269
	#define KEY_CAPS_LOCK          280
	#define KEY_SCROLL_LOCK        281
	#define KEY_NUM_LOCK           282
	#define KEY_PRINT_SCREEN       283
	#define KEY_PAUSE              284
	#define KEY_F1                 290
	#define KEY_F2                 291
	#define KEY_F3                 292
	#define KEY_F4                 293
	#define KEY_F5                 294
	#define KEY_F6                 295
	#define KEY_F7                 296
	#define KEY_F8                 297
	#define KEY_F9                 298
	#define KEY_F10                299
	#define KEY_F11                300
	#define KEY_F12                301
	#define KEY_F13                302
	#define KEY_F14                303
	#define KEY_F15                304
	#define KEY_F16                305
	#define KEY_F17                306
	#define KEY_F18                307
	#define KEY_F19                308
	#define KEY_F20                309
	#define KEY_KP_0               320
	#define KEY_KP_1               321
	#define KEY_KP_2               322
	#define KEY_KP_3               323
	#define KEY_KP_4               324
	#define KEY_KP_5               325
	#define KEY_KP_6               326
	#define KEY_KP_7               327
	#define KEY_KP_8               328
	#define KEY_KP_9               329
	#define KEY_KP_DECIMAL         330
	#define KEY_KP_DIVIDE          331
	#define KEY_KP_MULTIPLY        332
	#define KEY_KP_SUBTRACT        333
	#define KEY_KP_ADD             334
	#define KEY_KP_ENTER          335
	#define KEY_KP_EQUAL          336
	#define KEY_LEFT_SHIFT        340
	#define KEY_LEFT_CONTROL      341
	#define KEY_LEFT_ALT          342
	#define KEY_LEFT_SUPER        343
	#define KEY_RIGHT_SHIFT       344
	#define KEY_RIGHT_CONTROL     345
	#define KEY_RIGHT_ALT         346
	#define KEY_RIGHT_SUPER       347
	#define KEY_MENU              348
	#define KMOD_SHIFT            0x0001
	#define KMOD_CONTROL          0x0002
	#define KMOD_ALT              0x0004
	#define KMOD_SUPER            0x0008

#endif