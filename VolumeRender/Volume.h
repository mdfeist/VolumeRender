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

	void render(Camera*);
private:
	bool initialized;

	// Vertex Buffer Object
	GLuint cubeVerticesVBO;

	// Frame Buffer Object
	GLuint FBO;
	// Textures
	GLuint front_facing;
	GLuint back_facing;
	GLuint volume_texture;
	GLuint transferTexture;
	
	// CG variables
	CGcontext context;
	CGprofile fragmentProfile;//CG_PROFILE_ARBFP1;
	CGprogram fProgram;
	CGparameter cgFrontTexData;
	CGparameter cgBackTexData;
	CGparameter cgVolumeTexData;
	CGparameter cgTransferTexData;
	CGparameter cgStepSize;

	void createCube(float x, float y, float z);
	GLuint createVolume();
	void computeTransferFunction();
	void generateGradients(int sampleSize);
	void filterNxNxN(int n);
	Eigen::Vector3f sampleNxNxN(int x, int y, int z, int n);
	float sampleVolume(int x, int y, int z);
	Eigen::Vector3f sampleGradients(int x, int y, int z);
	bool isInBounds(int x, int y, int z);

	// FBO
	GLuint setupFBO();
	bool bindFBO(GLuint fbo_handle, GLuint fbo_texture);
	void unbindFBO();

	// CG
	int setupCg(CGcontext *context, CGprogram *fProgram, 
			CGprofile *fragmentProfile, char *file);

	// Volume
	int pixelCount;
	GLubyte *data;
	int volumeWidth, volumeHeight, volumeDepth;

	GLubyte* transfer;
	float *mSamples;

	std::vector<TransferControlPoint*> colorKnots;
	std::vector<TransferControlPoint*> alphaKnots;
};

