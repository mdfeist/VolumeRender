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
class Camera;

class Renderer : public Thread
{
public:
	Renderer(void);
	~Renderer(void);

	static LRESULT CALLBACK StaticWndProc(HWND, UINT, WPARAM, LPARAM);	// Declaration For WndProc
	LRESULT WndProc( UINT, WPARAM, LPARAM);

	void setWindow(HWND win);
	HWND getWindow();
	void setClearColor(float r, float g, float b, float a);

	void addActor(Actor* a);

	Camera* getActiveCamera();
protected:
	DWORD runThread();
private:
	HDC			hDC;			// Private GDI Device Context
	HGLRC		hRC;			// Permanent Rendering Context
	HWND		hWnd;			// Holds Our Window Handle
	HINSTANCE	hInstance;		// Holds The Instance Of The Application

	Camera*		camera;			// Camera for rendering

	bool		keys[256];		// Array Used For The Keyboard Routine
	bool		active;			// Window Active Flag Set To TRUE By Default
	bool		fullscreen;		// Full screen Flag Set To Full screen Mode By Default
	bool		done;			// Boolean Variable To Exit Loop

	HANDLE		g_hMutex;

	// GL Clear Color
	float clearColor[4];
	// Render Actors
	std::vector<Actor*> actors;

	// Mutex Functions
	bool lock();
	void unlock();

	// GL Window
	int initGL();
	void destroyContext();
	void destroyWindow();
	void KillGLWindow();
	BOOL CreateGLWindow(LPCWSTR title, int width, int height, int bits, bool fullscreenflag);

	void render();
	void resize(int width, int height);

	void setActive(bool value);
	void setKey(WPARAM key, bool value);
};


