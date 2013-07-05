#pragma once
#include <windows.h>		// Header File For Windows
#include <process.h>

#include <GL/glew.h>

#include <Cg/cg.h>
#include <Cg/cgGL.h>

#include <GL\glut.h>

#include <vector>

#include "Thread.h"

class Actor;

class Renderer : public Thread
{
public:
	Renderer(void);
	~Renderer(void);

	HWND getWindow();
	void setClearColor(float r, float g, float b, float a);

	void addActor(Actor* a);
protected:
	DWORD runThread();
private:
	HDC			hDC;			// Private GDI Device Context
	HGLRC		hRC;			// Permanent Rendering Context
	HWND		hWnd;			// Holds Our Window Handle
	HINSTANCE	hInstance;		// Holds The Instance Of The Application

	bool		keys[256];		// Array Used For The Keyboard Routine
	bool		active;			// Window Active Flag Set To TRUE By Default
	bool		fullscreen;		// Fullscreen Flag Set To Fullscreen Mode By Default
	bool		done;			// Bool Variable To Exit Loop

	bool		initialized;

	HANDLE		g_hMutex;

	// Frame Buffer Object
	GLuint FBO;
	// Textures
	GLuint FBO_texture;

	// CG variables
	CGcontext context;
	CGprofile fragmentProfile;//CG_PROFILE_ARBFP1;
	CGprogram fProgram;
	CGparameter cgTexData;

	// GL Clear Color
	float clearColor[4];

	std::vector<Actor*> actors;

	bool lock();
	void unlock();

	int initGL();
	void KillGLWindow();
	BOOL CreateGLWindow(LPCWSTR title, int width, int height, int bits, bool fullscreenflag);

	void render();
	void resize(int width, int height);

	void setActive(bool value);
	void setKey(WPARAM key, bool value);
};


