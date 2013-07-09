#pragma once
#include "Actor.h"

#include <GL/glew.h>

#include <Cg/cg.h>
#include <Cg/cgGL.h>

#include <GL\glut.h>

class Volume : public Actor
{
public:
	Volume(void);
	~Volume(void);

	void init();
	bool needsInit();
	void reset();

	void render(int w, int h);
private:
	bool initialized;

	// Vertex Buffer Object
	GLuint g_uiVerticesVBO;

	// Frame Buffer Object
	GLuint FBO;
	// Textures
	GLuint front_facing;
	GLuint back_facing;
	GLuint volume_texture;
	
	// CG variables
	CGcontext context;
	CGprofile fragmentProfile;//CG_PROFILE_ARBFP1;
	CGprogram fProgram;
	CGparameter cgFrontTexData;
	CGparameter cgBackTexData;
	CGparameter cgVolumeTexData;
	CGparameter cgStepSize;

	void createCube(float x, float y, float z);

	// FBO
	GLuint setupFBO();
	bool bindFBO(GLuint fbo_handle, GLuint fbo_texture);
	void unbindFBO();

	// CG
	int setupCg(CGcontext *context, CGprogram *fProgram, 
			CGprofile *fragmentProfile, char *file);
};

