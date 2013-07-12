#pragma once
#include "Actor.h"

#include <GL/glew.h>

#include <Cg/cg.h>
#include <Cg/cgGL.h>

#include <GL\glut.h>

#include <vector>

class TransferControlPoint;

class Volume : public Actor
{
public:
	Volume(void);
	~Volume(void);

	int loadVolume(char *directory);

	void init();
	bool needsInit();

	void filter(int z, int n);

	void render(Camera*);
private:
	bool initialized;

	// Vertex Buffer Object
	GLuint cubeVerticesVBO;

	// Frame Buffer Object
	GLuint FBO;
	
	// CG variables
	CGcontext	context;
	CGprofile	fragmentProfile;//CG_PROFILE_ARBFP1;

	// First Pass
	CGprogram	fProgramFirstPass;
	CGparameter cgFrontTexData;
	CGparameter cgBackTexData;
	CGparameter cgDepthTexData;
	CGparameter cgVolumeTexData;
	CGparameter cgTransferTexData;
	CGparameter cgStepSize;
	// Textures
	GLuint front_facing;
	GLuint back_facing;
	GLuint volume_texture;
	GLuint transferTexture;
	GLuint depthrenderbuffer;

	// Second Pass
	CGprogram	fProgramSecondPass;
	CGparameter cgColorTexData;
	CGparameter cgPositionTexData;
	// Textures
	GLuint colorTexture;
	GLuint positionTexture;

	void createCube(float x, float y, float z);
	GLuint createVolume();
	void computeTransferFunction();

	// FBO
	GLuint setupFBO();
	bool bindFBO(GLuint fbo_handle, GLuint *fbo_texture, int size);
	void unbindFBO();

	// CG
	int setupCg(CGcontext *context, CGprogram *fProgram, 
			CGprofile *fragmentProfile, char *file);

	// Volume
	int pixelCount;
	GLubyte *data;
	int volumeWidth, volumeHeight, volumeDepth;

	GLubyte* transfer;

	std::vector<TransferControlPoint*> colorKnots;
	std::vector<TransferControlPoint*> alphaKnots;
};

