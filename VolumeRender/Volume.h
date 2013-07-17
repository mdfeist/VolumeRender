#pragma once
#include <Windows.h>

#include "Actor.h"

#include <GL/glew.h>

#include <Cg/cg.h>
#include <Cg/cgGL.h>

#include <GL\glut.h>

#include <vector>

class TransferControlPoint;
class VolumeCube;

struct VertexPositionColor;

class Volume : public Actor
{
public:
	Volume(void);
	~Volume(void);

	int loadRaw(char *directory);
	int loadVolume(char *directory);

	void init();
	bool needsInit();
	void setup();
	
	void recursiveVolumeBuild(VolumeCube C);

	void setIsoValue(float value);
	void increaseIsoValue(float value);

	void render(Camera*);
private:
	bool initialized;

	// Vertex Buffer Object
	GLuint cubeVerticesVBO;
	GLuint cubeIndicesVBO;
	
	unsigned int mNumIndices;

	// Frame Buffer Object
	GLuint FBO;
	
	// CG variables
	CGcontext	context;
	CGprofile	fragmentProfile;//CG_PROFILE_ARBFP1;

	// First Pass
	CGprogram	fProgramFirstPass;
	CGparameter cgFrontTexData;
	CGparameter cgBackTexData;
	CGparameter cgVolumeTexData;
	CGparameter cgTransferTexData;
	CGparameter cgStepSize;
	CGparameter cgisoValue;
	// Textures
	GLuint front_facing;
	GLuint back_facing;
	GLuint volume_texture;
	GLuint transferTexture;
	GLuint depthrenderbuffer;

	// Second Pass
	CGprogram	fProgramSecondPass;
	CGparameter cgColorTexData;
	// Textures
	GLuint colorTexture;

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
	float spacingX, spacingY, spacingZ;
	int volumeWidth, volumeHeight, volumeDepth;

	GLubyte* transfer;
	float isoValue;

	std::vector<VertexPositionColor> mVertices;

	void createCube(float x, float y, float z);
	GLuint createVolume();
	void computeTransferFunction();

	int sampleVolume(int x, int y, int z);
	float sampleVolume3DWithTransfer(Eigen::Vector3f& min, Eigen::Vector3f& max);
	void buildVertBuffer();

	std::vector<TransferControlPoint*> colorKnots;
	std::vector<TransferControlPoint*> alphaKnots;
};

