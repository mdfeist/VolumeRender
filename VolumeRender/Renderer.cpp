#include "StdAfx.h"
#include "Renderer.h"

#include <stdio.h>
#include <stdlib.h>

#include "Actor.h"

LRESULT	CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);	// Declaration For WndProc

bool Msg(const char* msg) {
	printf(msg);
	printf("\n");
	return true;
}

Renderer::Renderer(void) {
	g_hMutex = CreateMutex(
		NULL,
		//(LPSECURITY_ATTRIBUTES)SYNCHRONIZE, 
		FALSE, 
		NULL);

	clearColor[0] = 0.f;
	clearColor[1] = 0.f;
	clearColor[2] = 0.f;
	clearColor[3] = 0.5f;

	start();
}

Renderer::~Renderer(void) {
	// Free the mutex
	CloseHandle(g_hMutex);
}

// Lock
bool Renderer::lock() {
	// Request ownership of mutex
	DWORD  dwWaitResult;
	while(true) {
		// Wait for Mutex to be free
		dwWaitResult = WaitForSingleObject(g_hMutex, INFINITE);
		switch (dwWaitResult) {
			// The thread got ownership of the mutex
		case WAIT_OBJECT_0: 
			return true;
			break; 

			// The thread got ownership of an abandoned mutex
			// The database is in an indeterminate state
		case WAIT_ABANDONED: 
			return false; 
			break;
		}
	}

	return false;
}

void Renderer::unlock() {
	ReleaseMutex(g_hMutex);
}

int Renderer::initGL()									// All Setup For OpenGL Goes Here
{
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(										// Set Background Color
		clearColor[0], 
		clearColor[1], 
		clearColor[2], 
		clearColor[3]);
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	return TRUE;										// Initialization Went OK
}

void Renderer::setClearColor(float r, float g, float b, float a) {
	lock();

	if (wglMakeCurrent( hDC, hRC )) {
		glClearColor(r, g, b, a);
		wglMakeCurrent( NULL, NULL);
	}

	clearColor[0] = r;
	clearColor[1] = g;
	clearColor[2] = b;
	clearColor[3] = a;

	unlock();
}

void Renderer::addActor(Actor* a) {
	actors.push_back(a);
}

void Renderer::render() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	for each (Actor* a in actors)
		a->render();
}

HWND Renderer::getWindow() {
	return hWnd;
}

void Renderer::setActive(bool value) {
	active = value;
}

void Renderer::setKey(WPARAM key, bool value) {
	keys[key] = value;
}

void Renderer::resize(int width, int height) {
	if (height == 0) height = 1;

	lock();							// Lock renderer
	wglMakeCurrent( hDC, hRC );		// Make current context
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)width/(GLfloat)height, 0.01, 400.0);
	glMatrixMode(GL_MODELVIEW);
	unlock();						// Unlock Render
}

void Renderer::KillGLWindow()							// Properly Kill The Window
{
	if (fullscreen)										// Are We In Fullscreen Mode?
	{
		ChangeDisplaySettings(NULL,0);					// If So Switch Back To The Desktop
		ShowCursor(TRUE);								// Show Mouse Pointer
	}

	if (hRC)											// Do We Have A Rendering Context?
	{
		if (!wglMakeCurrent(NULL,NULL))					// Are We Able To Release The DC And RC Contexts?
		{
			Msg("Release Of DC And RC Failed.");
		}

		if (!wglDeleteContext(hRC))						// Are We Able To Delete The RC?
		{
			Msg("Release Rendering Context Failed.");
		}
		hRC=NULL;										// Set RC To NULL
	}

	if (hDC && !ReleaseDC(hWnd,hDC))					// Are We Able To Release The DC
	{
		Msg("Release Device Context Failed.");
		hDC=NULL;										// Set DC To NULL
	}

	if (hWnd && !DestroyWindow(hWnd))					// Are We Able To Destroy The Window?
	{
		Msg("Could Not Release hWnd.");
		hWnd=NULL;										// Set hWnd To NULL
	}

	if (!UnregisterClass(L"OpenGL",hInstance))			// Are We Able To Unregister Class
	{
		Msg("Could Not Unregister Class.");
		hInstance=NULL;									// Set hInstance To NULL
	}
}

/*	This Code Creates Our OpenGL Window.  Parameters Are:					*
 *	title			- Title To Appear At The Top Of The Window				*
 *	width			- Width Of The GL Window Or Fullscreen Mode				*
 *	height			- Height Of The GL Window Or Fullscreen Mode			*
 *	bits			- Number Of Bits To Use For Color (8/16/24/32)			*
 *	fullscreenflag	- Use Fullscreen Mode (TRUE) Or Windowed Mode (FALSE)	*/
 
BOOL Renderer::CreateGLWindow(LPCWSTR title, int width, int height, int bits, bool fullscreenflag)
{
	GLuint		PixelFormat;			// Holds The Results After Searching For A Match
	WNDCLASS	wc;						// Windows Class Structure
	DWORD		dwExStyle;				// Window Extended Style
	DWORD		dwStyle;				// Window Style
	RECT		WindowRect;				// Grabs Rectangle Upper Left / Lower Right Values
	WindowRect.left=(long)0;			// Set Left Value To 0
	WindowRect.right=(long)width;		// Set Right Value To Requested Width
	WindowRect.top=(long)0;				// Set Top Value To 0
	WindowRect.bottom=(long)height;		// Set Bottom Value To Requested Height

	fullscreen=fullscreenflag;			// Set The Global Fullscreen Flag

	hInstance			= GetModuleHandle(NULL);				// Grab An Instance For Our Window
	wc.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;	// Redraw On Size, And Own DC For Window.
	wc.lpfnWndProc		= (WNDPROC) WndProc;					// WndProc Handles Messages
	wc.cbClsExtra		= 0;									// No Extra Window Data
	wc.cbWndExtra		= 0;									// No Extra Window Data
	wc.hInstance		= hInstance;							// Set The Instance
	wc.hIcon			= LoadIcon(NULL, IDI_WINLOGO);			// Load The Default Icon
	wc.hCursor			= LoadCursor(NULL, IDC_ARROW);			// Load The Arrow Pointer
	wc.hbrBackground	= NULL;									// No Background Required For GL
	wc.lpszMenuName		= NULL;									// We Don't Want A Menu
	wc.lpszClassName	= L"OpenGL";							// Set The Class Name

	RegisterClass(&wc);
	/*
	if (!RegisterClass(&wc))									// Attempt To Register The Window Class
	{
		Msg("Failed To Register The Window Class.");
		return FALSE;											// Return FALSE
	}
	*/
	if (fullscreen)												// Attempt Fullscreen Mode?
	{
		/*
		DEVMODE dmScreenSettings;								// Device Mode
		memset(&dmScreenSettings,0,sizeof(dmScreenSettings));	// Makes Sure Memory's Cleared
		dmScreenSettings.dmSize=sizeof(dmScreenSettings);		// Size Of The Devmode Structure
		dmScreenSettings.dmPelsWidth	= width;				// Selected Screen Width
		dmScreenSettings.dmPelsHeight	= height;				// Selected Screen Height
		dmScreenSettings.dmBitsPerPel	= bits;					// Selected Bits Per Pixel
		dmScreenSettings.dmFields=DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT;

		// Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
		if (ChangeDisplaySettings(&dmScreenSettings,CDS_FULLSCREEN)!=DISP_CHANGE_SUCCESSFUL)
		{
			// If The Mode Fails, Offer Two Options.  Quit Or Use Windowed Mode.
			if (Msg("The Requested Fullscreen Mode Is Not Supported By\nYour Video Card. Use Windowed Mode Instead?"))
			{
				fullscreen=FALSE;		// Windowed Mode Selected.  Fullscreen = FALSE
			}
			else
			{
				// Pop Up A Message Box Letting User Know The Program Is Closing.
				Msg("Program Will Now Close.");
				return FALSE;									// Return FALSE
			}
		}
		*/
	}

	if (fullscreen)												// Are We Still In Fullscreen Mode?
	{
		dwExStyle=WS_EX_APPWINDOW;								// Window Extended Style
		dwStyle=WS_POPUP;										// Windows Style
		ShowCursor(FALSE);										// Hide Mouse Pointer
	}
	else
	{
		dwExStyle=WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;			// Window Extended Style
		dwStyle=WS_OVERLAPPEDWINDOW;							// Windows Style
	}

	AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);		// Adjust Window To True Requested Size

	hWnd=CreateWindowEx( dwExStyle,							// Extended Style For The Window
						L"OpenGL",							// Class Name
						title,								// Window Title
						dwStyle |							// Defined Window Style
						WS_CLIPSIBLINGS |					// Required Window Style
						WS_CLIPCHILDREN,					// Required Window Style
						0, 0,								// Window Position
						WindowRect.right-WindowRect.left,	// Calculate Window Width
						WindowRect.bottom-WindowRect.top,	// Calculate Window Height
						NULL,								// No Parent Window
						NULL,								// No Menu
						hInstance,							// Instance
						NULL );								// Dont Pass Anything To WM_CREATE

	// Create The Window
	if (!hWnd)
	{
		KillGLWindow();								// Reset The Display
		Msg("Window Creation Error.");
		return FALSE;								// Return FALSE
	}

	static	PIXELFORMATDESCRIPTOR pfd=				// pfd Tells Windows How We Want Things To Be
	{
		sizeof(PIXELFORMATDESCRIPTOR),				// Size Of This Pixel Format Descriptor
		1,											// Version Number
		PFD_DRAW_TO_WINDOW |						// Format Must Support Window
		PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,							// Must Support Double Buffering
		PFD_TYPE_RGBA,								// Request An RGBA Format
		bits,										// Select Our Color Depth
		0, 0, 0, 0, 0, 0,							// Color Bits Ignored
		0,											// No Alpha Buffer
		0,											// Shift Bit Ignored
		0,											// No Accumulation Buffer
		0, 0, 0, 0,									// Accumulation Bits Ignored
		16,											// 16Bit Z-Buffer (Depth Buffer)  
		0,											// No Stencil Buffer
		0,											// No Auxiliary Buffer
		PFD_MAIN_PLANE,								// Main Drawing Layer
		0,											// Reserved
		0, 0, 0										// Layer Masks Ignored
	};
	
	if (!(hDC=GetDC(hWnd)))							// Did We Get A Device Context?
	{
		KillGLWindow();								// Reset The Display
		Msg("Can't Create A GL Device Context.");
		return FALSE;								// Return FALSE
	}

	if (!(PixelFormat=ChoosePixelFormat(hDC,&pfd)))	// Did Windows Find A Matching Pixel Format?
	{
		KillGLWindow();								// Reset The Display
		Msg("Can't Find A Suitable PixelFormat.");
		return FALSE;								// Return FALSE
	}

	if(!SetPixelFormat(hDC,PixelFormat,&pfd))		// Are We Able To Set The Pixel Format?
	{
		KillGLWindow();								// Reset The Display
		Msg("Can't Set The PixelFormat.");
		return FALSE;								// Return FALSE
	}

	if (!(hRC=wglCreateContext(hDC)))				// Are We Able To Get A Rendering Context?
	{
		KillGLWindow();								// Reset The Display
		Msg("Can't Create A GL Rendering Context.");
		return FALSE;								// Return FALSE
	}

	if(!wglMakeCurrent(hDC,hRC))					// Try To Activate The Rendering Context
	{
		KillGLWindow();								// Reset The Display
		Msg("Can't Activate The GL Rendering Context.");
		return FALSE;								// Return FALSE
	}

	ShowWindow(hWnd,SW_SHOW);						// Show The Window
	SetForegroundWindow(hWnd);						// Slightly Higher Priority
	SetFocus(hWnd);									// Sets Keyboard Focus To The Window
	resize(width, height);							// Set Up Our Perspective GL Screen

	if (!initGL())									// Initialize Our Newly Created GL Window
	{
		KillGLWindow();								// Reset The Display
		Msg("Initialization Failed.");
		return FALSE;								// Return FALSE
	}

	wglMakeCurrent( NULL, NULL );

	return TRUE;									// Success
}

LRESULT CALLBACK WndProc(	HWND	hWnd,			// Handle For This Window
							UINT	uMsg,			// Message For This Window
							WPARAM	wParam,			// Additional Message Information
							LPARAM	lParam)			// Additional Message Information
{
	
	switch (uMsg)											// Check For Windows Messages
	{
		case WM_ACTIVATE:									// Watch For Window Activate Message
		{
			return 0;										// Return To The Message Loop
		}

		case WM_SYSCOMMAND:									// Intercept System Commands
		{
			switch (wParam)									// Check System Calls
			{
				case SC_SCREENSAVE:							// Screensaver Trying To Start?
				case SC_MONITORPOWER:						// Monitor Trying To Enter Powersave?
				return 0;									// Prevent From Happening
			}
			break;											// Exit
		}

		case WM_CLOSE:										// Did We Receive A Close Message?
		{
			PostQuitMessage(0);								// Send A Quit Message
			return 0;										// Jump Back
		}

		case WM_KEYDOWN:									// Is A Key Being Held Down?
		{				
			return 0;										// Jump Back
		}

		case WM_KEYUP:										// Has A Key Been Released?
		{
			return 0;										// Jump Back
		}

		case WM_SIZE:										// Resize The OpenGL Window
		{
			return 0;										// Jump Back
		}
	}

	// Pass All Unhandled Messages To DefWindowProc
	return DefWindowProc(hWnd,uMsg,wParam,lParam);
}

DWORD Renderer::runThread() {
	MSG	msg;									// Windows Message Structure
	done = FALSE;

	hDC = NULL;
	hRC = NULL;
	hWnd = NULL;

	active = TRUE;
	fullscreen = FALSE;

	float fps = (1000.f/60.f);
	float dwCurrentTime = 0.f;
	float dwElapsedTime = 0.f;
	float dwLastUpdateTime = 0.f;

	memset( keys, FALSE, 256 );

	bool created = CreateGLWindow(L"OpenGL",640,480,16,fullscreen);

	initialized = true;

	// Create Our OpenGL Window
	if (!created)
	{
		Msg("Failed to create Window.");
		return 0;									// Quit If Window Was Not Created
	}

	while(!done)									// Loop That Runs While done=FALSE
	{
		// Is There A Message Waiting?
		if (PeekMessage(&msg,NULL,0,0,PM_REMOVE))
		{
			if (msg.message==WM_QUIT)				// Have We Received A Quit Message?
			{
				done=TRUE;							// If So done=TRUE
			}
			else if (msg.message==WM_ACTIVATE) 
			{
				if (!HIWORD(msg.wParam))			// Check Minimization State
				{
					setActive(TRUE);				// Program Is Active
				}
				else
				{
					setActive(FALSE);				// Program Is No Longer Active
				}
			}
			else if (msg.message==WM_KEYDOWN) 
			{
				setKey(msg.wParam, TRUE);
			}
			else if (msg.message==WM_KEYUP) 
			{
				setKey(msg.wParam, FALSE);
			}
			else if (msg.message==WM_SIZE) 
			{
				resize(LOWORD(msg.lParam),HIWORD(msg.lParam));	// LoWord=Width, HiWord=Height
			}
			else									// If Not, Deal With Window Messages
			{
				TranslateMessage(&msg);				// Translate The Message
				DispatchMessage(&msg);				// Dispatch The Message
			}
		}
		
		// Draw The Scene.  Watch For ESC Key And Quit Messages From DrawGLScene()
		if (active)								// Program Active?
		{
			if (keys[VK_ESCAPE])				// Was ESC Pressed?
			{
				done=TRUE;						// ESC Signalled A Quit
			}
			else								// Not Time To Quit, Update Screen
			{

				// Get Current Time
				SYSTEMTIME time;
				GetSystemTime(&time);
				dwCurrentTime = (float)(time.wHour*60.f*60.f*1000.f + time.wMinute*60.f*1000.f + time.wSecond*1000.f + time.wMilliseconds);
				// Calculate time Elapsed time
				dwElapsedTime = dwCurrentTime - dwLastUpdateTime;
				
				// If the elapsed time is less then the fps
				if (dwElapsedTime > fps)
				{
					lock();
					wglMakeCurrent( hDC, hRC );	// Make current context
					render();					// Draw The Scene
					SwapBuffers(hDC);			// Swap Buffers (Double Buffering)
					wglMakeCurrent( NULL, NULL );
					unlock();

					// Set LastUpdateTime to CurrentTime
					dwLastUpdateTime = dwCurrentTime;
				}
			}
		}

		if (keys[VK_F1])						// Is F1 Being Pressed?
		{
			keys[VK_F1]=FALSE;					// If So Make Key FALSE
			KillGLWindow();						// Kill Our Current Window
			fullscreen=!fullscreen;				// Toggle Fullscreen / Windowed Mode
			// Recreate Our OpenGL Window
			if (!CreateGLWindow(L"OpenGL",640,480,16,fullscreen))
			{
				//return 0;						// Quit If Window Was Not Created
			}
		}
	}

	// Shutdown
	KillGLWindow();									// Kill The Window

	return 0;
}