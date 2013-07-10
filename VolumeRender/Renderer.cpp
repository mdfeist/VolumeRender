#include "StdAfx.h"
#include "Renderer.h"

#include <stdio.h>
#include <stdlib.h>

#include "Actor.h"
#include "Camera.h"

extern "C" {
    _declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}

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

	camera = new Camera();

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

Camera* Renderer::getActiveCamera() {
	return camera;
}

int Renderer::initGL()									// All Setup For OpenGL Goes Here
{
	// Init GLEW
	if ( glewInit() != GLEW_OK )
	{
		Msg("Failed to initialize GLEW.");
		return FALSE;
	}

	if ( !glewIsSupported("GL_VERSION_1_5") && !glewIsSupported( "GL_ARB_vertex_buffer_object" ) )
	{
		Msg("ARB_vertex_buffer_object not supported!");
		return FALSE;
	}

	glewGetExtension("glMultiTexCoord2fvARB");  
	if(glewGetExtension("GL_EXT_framebuffer_object") ) Msg("GL_EXT_framebuffer_object support ");
	if(glewGetExtension("GL_EXT_renderbuffer_object")) Msg("GL_EXT_renderbuffer_object support ");
	if(glewGetExtension("GL_ARB_vertex_buffer_object"))  Msg("GL_ARB_vertex_buffer_object support");
	if(GL_ARB_multitexture) Msg("GL_ARB_multitexture support \n");
	
	if (glewGetExtension("GL_ARB_fragment_shader")      != GL_TRUE ||
		glewGetExtension("GL_ARB_vertex_shader")        != GL_TRUE ||
		glewGetExtension("GL_ARB_shader_objects")       != GL_TRUE ||
		glewGetExtension("GL_ARB_shading_language_100") != GL_TRUE)
	{
		 Msg("Driver does not support OpenGL Shading Language");
	}

	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(										// Set Background Color
		clearColor[0], 
		clearColor[1], 
		clearColor[2], 
		clearColor[3]);
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glDisable(GL_LIGHTING);
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

void Renderer::addActor(Actor* a) {						// Add actor to scene
	actors.push_back(a);								// Push actor into actors vector
}

void Renderer::render() {								// Render the scene
	static float rot = 0.0f;							// Rotation value
	rot += 0.75f;										// Update Rotation

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear screen (Color and Depth)
	glLoadIdentity();									// Load Identity
			
	glTranslatef(0, 0, -2.25);							// Set Camera Position
	glRotatef(25,1,0,0);
	glRotatef(rot,0,1,0);

	for each (Actor* a in actors) {						// Loop through actors
		if (a->needsInit())								// Check if actor needs set up
			a->init();									// Setup actor

		a->render(camera);								// Render actor
	}
}

void Renderer::setWindow(HWND win) {					// Set the window handle
	hWnd = win;											// Update window handle
}

HWND Renderer::getWindow() {							// Get the window handle
	return hWnd;										// return hWnd
}

void Renderer::setActive(bool value) {					// Updated if window is active
	active = value;										// Set if active
}

void Renderer::setKey(WPARAM key, bool value) {			// Update keyboard input
	keys[key] = value;									// Set if key pressed
}

void Renderer::resize(int width, int height) {
	if (height == 0) height = 1;						// Handles case 1/0 when calculating aspect ratio

	camera->setWidth(width);							// Update screen width
	camera->setHeight(height);							// Update screen height

	lock();												// Lock renderer
	wglMakeCurrent( hDC, hRC );							// Make current context
	glViewport(0, 0, width, height);					// Set Viewport
	glMatrixMode(GL_PROJECTION);						// Update projection
	glLoadIdentity();									// Load Identity
	gluPerspective(camera->getFOV(),					// Set Field of View
		(GLfloat)width/(GLfloat)height,					// Set aspect ratio
		camera->getNearClipping(),						// Set near clipping
		camera->getFarClipping());						// Set far clipping
	glMatrixMode(GL_MODELVIEW);							// Change back to model view mode
	unlock();											// Unlock Render
}

void Renderer::destroyContext() {
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
}

void Renderer::destroyWindow() {
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

void Renderer::KillGLWindow()							// Properly Kill The Window
{
	/*
	if (fullscreen)										// Are We In Fullscreen Mode?
	{
		ChangeDisplaySettings(NULL,0);					// If So Switch Back To The Desktop
		ShowCursor(TRUE);								// Show Mouse Pointer
	}
	*/
	destroyContext();
	destroyWindow();
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
	wc.lpfnWndProc		= (WNDPROC) Renderer::StaticWndProc;	// WndProc Handles Messages
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

	SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)this);

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

	wglMakeCurrent( NULL, NULL );					// Deactivate The Rendering Context

	return TRUE;									// Success
}

LRESULT CALLBACK Renderer::StaticWndProc(	HWND	hWnd,			// Handle For This Window
											UINT	uMsg,			// Message For This Window
											WPARAM	wParam,			// Additional Message Information
											LPARAM	lParam)			// Additional Message Information
{
	Renderer *ren = NULL;

	// Get pointer to window
	if(uMsg == WM_CREATE)
	{
		//ren = (Renderer*)((LPCREATESTRUCT)lParam)->lpCreateParams;
		//SetWindowLongPtr(hWnd, GWL_USERDATA, (LONG_PTR)ren);
		Msg("Create Window.");
	}
	else
	{
		ren = (Renderer*)GetWindowLongPtr(hWnd, GWL_USERDATA);
		if(!ren) return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}
	
	return ren->WndProc(uMsg,wParam,lParam);
}
LRESULT Renderer::WndProc(	UINT	uMsg,			// Message For This Window
							WPARAM	wParam,			// Additional Message Information
							LPARAM	lParam)			// Additional Message Information
{
	switch (uMsg)									// Check For Windows Messages
	{
		case WM_ACTIVATE:							// Watch For Window Activate Message
		{
			if (!HIWORD(wParam))					// Check Minimization State
			{
				setActive(TRUE);					// Program Is Active
			}
			else
			{
				setActive(FALSE);					// Program Is No Longer Active
			}

			return 0;								// Return To The Message Loop
		}

		case WM_SYSCOMMAND:							// Intercept System Commands
		{
			switch (wParam)							// Check System Calls
			{
				case SC_SCREENSAVE:					// Screensaver Trying To Start?
				case SC_MONITORPOWER:				// Monitor Trying To Enter Powersave?
				return 0;							// Prevent From Happening
			}
			break;									// Exit
		}

		case WM_CLOSE:								// Did We Receive A Close Message?
		{
			PostQuitMessage(0);						// Send A Quit Message
			return 0;								// Jump Back
		}

		case WM_KEYDOWN:							// Is A Key Being Held Down?
		{				
			setKey(wParam, TRUE);
			return 0;								// Jump Back
		}

		case WM_KEYUP:								// Has A Key Been Released?
		{
			setKey(wParam, FALSE);
			return 0;								// Jump Back
		}

		case WM_SIZE:								// Resize The OpenGL Window
		{
			resize(LOWORD(lParam),HIWORD(lParam));	// LoWord=Width, HiWord=Height
			return 0;								// Jump Back
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

	BOOL created = CreateGLWindow(L"OpenGL",640,480,16,fullscreen);

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
				dwCurrentTime = timeGetTime();
				// Calculate time Elapsed time
				dwElapsedTime = dwCurrentTime - dwLastUpdateTime;
				
				// If the elapsed time is less then the fps
				if (dwElapsedTime > fps)
				{
					// Set LastUpdateTime to CurrentTime
					dwLastUpdateTime = dwCurrentTime;

					//printf("Elapsed Time: %f\n", dwElapsedTime);
					//printf("FPS Time: %f\n", fps);

					lock();							// Lock Render
					wglMakeCurrent( hDC, hRC );		// Make current context
					render();						// Draw The Scene
					SwapBuffers(hDC);				// Swap Buffers (Double Buffering)
					wglMakeCurrent( NULL, NULL );	// Deactivate context
					unlock();						// Unlock Render
				}
			}
		}

		if (keys[VK_F1])						// Is F1 Being Pressed?
		{
			keys[VK_F1]=FALSE;					// If So Make Key FALSE
			//destroyWindow();					// Kill Our Current Window
			fullscreen=!fullscreen;				// Toggle Fullscreen / Windowed Mode
			// Adjust Our OpenGL Window
			DWORD style = GetWindowLong(hWnd, GWL_STYLE);		// Get current window style

			if (fullscreen)										// Are We Still In Fullscreen Mode?
			{
				style &= ~WS_EX_STATICEDGE;						// Remove Static Edge Style
				style &= ~WS_OVERLAPPEDWINDOW;					// Remove Overlapped Window Style
				style |= WS_POPUP;								// Add Pop Up Style
			}
			else
			{
				style &= ~WS_POPUP;								// Remove Pop Up Style
				style |= WS_EX_STATICEDGE;						// Add Static Edge Style
				style |= WS_OVERLAPPEDWINDOW;					// Add Overlapped Window Style
			}

			SetWindowLong(hWnd, GWL_STYLE, style);				// Set New Style
			SetWindowPos(hWnd, HWND_TOP,						// Update Window Position
				0, 0, 1024, 480, 
				SWP_SHOWWINDOW);
		}
	}

	// Shutdown
	KillGLWindow();									// Kill The Window

	return 0;
}