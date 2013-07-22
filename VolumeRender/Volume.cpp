#include "StdAfx.h"
#include "Volume.h"

#include <Windows.h>
#include <math.h>
#include <stddef.h>
#include <iostream>
#include <iomanip>

#include "TransferFunction.h"

#include "VolumeCube.h"
#include "VolumeLoader.h"
#include "VolumeStructs.h"

#include "BuildCubesManager.h"

// Minimum size of each sub cube of the Empty Space Leaping algorithm
#define EMPTY_SPACE_SIZE 0.05f
// All textures for the buffer are TEXTURE_WIDTHxTEXTURE_HEIGHT in dimensions
#define RES_512 0
#define RES_720p 1

#if RES_720p || RES_512
#if RES_720p
#define TEXTURE_WIDTH 1280
#define TEXTURE_HEIGHT 720
#else if RES_512
#define TEXTURE_WIDTH 512
#define TEXTURE_HEIGHT 512
#endif
#else
#define TEXTURE_WIDTH 256
#define TEXTURE_HEIGHT 256
#endif

// Used to find the offset of variables in a structure (found when binding the VBO)
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))

float length(float3 p) {							// Get the length of a float3
	return sqrtf(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline int clamp(int x, int a, int b) {				// Clamp so x is between a and b inclusively
	if (x < a)
		x = a;
	if (x > b)
		x = b;

    return x;
}

inline void loadingBar(int x, int n, int w) {
	// Calculuate the ratio of complete-to-incomplete.
	float ratio = x/(float)n;
	int c = ratio * w;

	std::cout << std::setw(3) << (int)(ratio*100) << "% [";
	for (int x=0; x<c; x++) std::cout << "=";
	for (int x=c; x<w; x++) std::cout << " ";
	std::cout << "]\r" << std::flush;
}

//Check for any GL Errors
void errcheck() {
	static GLenum errCode;
	const GLubyte *errString;

	if ((errCode = glGetError()) != GL_NO_ERROR) {
		errString = gluErrorString(errCode);
		fprintf(stderr, "OpenGL Error: %s\n", errString);
	}
}

//Check for any Cg Errors
void CheckCgError(void) {
	CGerror err = cgGetError();
	if (err != CG_NO_ERROR) {
		fprintf(stderr, "CG error: %s\n", cgGetErrorString(err));
		Sleep(1000);
	}
}

// Create a new 2D texture
GLuint newTexture(int width, int height) {
	GLuint texture;
	glGenTextures(1, &texture);												// Generate texture
	glBindTexture(GL_TEXTURE_2D, texture);									// Bind texture
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);				// Use the texture color when rendering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);		// Set the MAG_FILTER to interpolate linearly between the closest pixels
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);		// Set the MIN_FILTER to interpolate linearly between the closest pixels
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);	// Clamp the X value to boarder when doing texture look ups
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);	// Clamp the Y value to boarder when doing texture look ups
	glTexImage2D(															// Create texture
		GL_TEXTURE_2D,														// Texture is 2D
		0,																	// Use base level of detail
		GL_RGBA16F_ARB,														// Use internal format GL_RGBA16F_ARB
		width,																// Set the texture width
		height,																// Set the texture height
		0,																	// Set the boarder to zero (Needs to be 0)
		GL_RGBA,															// Use format GL_RGBA
		GL_FLOAT,															// Use type GL_FLOAT
		NULL);																// Set data to NULL
	return texture;															// Return texture
}

Volume::Volume(void) : Actor()												// Constructor
{
	volumeData = NULL;
	releaseDataAfterSetup = true;

	volumeWidth = 1;														// Set the default width to one
	volumeHeight = 1;														// Set the default height to one
	volumeDepth = 1;														// Set the default depth to one

	spacingX = 1.f;															// By default there is no spacing
	spacingY = 1.f;															// And all the dimensions of the
	spacingZ = 1.f;															// voxels are the same

	cubeVerticesVBO = 0;													// Set cubeVerticesVBO to NULL
	initialized = false;													// Set initialized to false

	scale = Eigen::Vector3f::Ones();										// Set scale to all ones
	position = Eigen::Vector3f(0, 0, 0);									// Set position to the origin
	rotation = Eigen::Quaternionf::Identity();								// Set rotation to the identity

	transferFunction = NULL;

	isoValue = 0.0f;														// Set the iso value threshold to 0
}


Volume::~Volume(void)														// Destructor
{
	if ( cubeVerticesVBO != 0 )												// Check if cubeVerticesVBO is not NULL
	{
		glDeleteBuffersARB( 1, &cubeVerticesVBO );							// Delete cubeVerticesVBO from GPU buffer
		cubeVerticesVBO = 0;												// Set cubeVerticesVBO to NULL
	}
}

void Volume::init() {														// Initialize OpenGL properties
	if(!volumeData) {
		std::cout << "Failed."<< std::endl;
		std::cout <<"*********************************************************************" <<std::endl;
		std::cout << "No Volume Given." << std::endl;
		std::cout << "*********************************************************************" <<std::endl;
		return;
	}

	FBO = setupFBO();														// Create Frame Buffer Object
	printf("- FBO created\n");

	front_facing = newTexture(TEXTURE_WIDTH, TEXTURE_HEIGHT);				// Create the texture to hold the render of the front faces
	back_facing = newTexture(TEXTURE_WIDTH, TEXTURE_HEIGHT);				// Create the texture to hold the render of the back faces

	colorTexture = newTexture(TEXTURE_WIDTH, TEXTURE_HEIGHT);				// Create the texture to hold the final render
	printf("- Textures created\n");

	cgSetErrorCallback(CheckCgError);										// Create callback for CG errors

	char shaderFirstPassFile[] = "shader/raycastDiffuse.cg";				// Get the path to the ray casting shader
	if (setupCg(&context, &fProgramFirstPass, &fragmentProfile,				// Load shader
		shaderFirstPassFile)) { 
		fprintf(stderr, "Error: %s\n", "Initializing Cg");					// Give message if load failed
		CheckCgError();
	}
																			// Find and bind the shaders parameter
	cgFrontTexData = cgGetNamedParameter(fProgramFirstPass, "frontTexData");
	cgBackTexData = cgGetNamedParameter(fProgramFirstPass, "backTexData");
	cgVolumeTexData = cgGetNamedParameter(fProgramFirstPass, "VolumeS");
	cgTransferTexData = cgGetNamedParameter(fProgramFirstPass, "TransferS");
	cgStepSize = cgGetNamedParameter(fProgramFirstPass, "stepSize");
	cgisoValue = cgGetNamedParameter(fProgramFirstPass, "isoValue");

	cgGLSetParameter1f(cgStepSize, 1.0f/1024.0f);							// Set the incremental step size of the ray cast

	char shaderSecondPassFile[] = "shader/multiSample.cg";					// Get the path to the post render shader
	if (setupCg(&context, &fProgramSecondPass, &fragmentProfile,			// Load shader
		shaderSecondPassFile)) {
		fprintf(stderr, "Error: %s\n", "Initializing Cg");					// Give message if load failed
		CheckCgError();
	}
																			// Find and bind the shaders parameter
	cgColorTexData = cgGetNamedParameter(fProgramSecondPass, "texData");
	
	buildVertBuffer();														// Create Vertex Buffer Objects
	printf("- VBO created\n");												// Used to store the volume cubes in GPU memory

	volume_texture = createVolume();										// Load the volume into GPU memory
	printf("- Volume texture created\n");

	glGenTextures(1, &transferTexture);										// Create 1D texture for transfer function
	glBindTexture(GL_TEXTURE_1D, transferTexture);							// Bind texture
	glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);	// Clamp texture to boarder
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);		// During texture look up grab the nearest value
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);		// During texture look up grab the nearest value
	glTexImage1D(															// Create texture
		GL_TEXTURE_1D,														// Texture is 1D
		0,																	// Use base level of detail															
		GL_RGBA,															// Use internal format GL_RGBA
		256,																// Set width
		0,																	// Set boarder to 0
		GL_RGBA,															// Use format GL_RGBA
		GL_UNSIGNED_BYTE,													// Use type GL_UNSIGNED_BYTES
		transferFunction->getTransferData());								// Copy transfer function data to GPU

	errcheck();																// Check for OpenGL errors

	volumeWidth = volumeData->getVolumeWidth();								// Set the width of the volume
	volumeHeight = volumeData->getVolumeHeight();							// Set the height of the volume
	volumeDepth = volumeData->getVolumeDepth();								// Set the depth of the volume

	spacingX = volumeData->getSpacingX();									// Set the spacing in the x dimension
	spacingY = volumeData->getSpacingY();									// Set the spacing in the y dimensione
	spacingZ = volumeData->getSpacingZ();									// Set the spacing in the z dimension

	if(releaseDataAfterSetup)
		delete volumeData;

	initialized = true;														// Set initialized flag to true
}

bool Volume::needsInit() {													// Check if volume was initialized
	return !initialized;
}

void Volume::setTransferFunction(TransferFunction* data) {					// Gives the volume a transfer function
	transferFunction = data;
}

void Volume::setVolumeData(VolumeLoader* data,								// Sets the volume data loaded from a file
	bool shouldReleaseDataAfterSetup) {										// Flag telling if the data can be released when we are done with it
	releaseDataAfterSetup = shouldReleaseDataAfterSetup;
	volumeData = data;
}

void Volume::setup() {														// Pre-compute volume properties
	if(!volumeData) {														// Check if any volume data was set
		std::cout << "Failed."<< std::endl;
		std::cout <<"*********************************************************************" <<std::endl;
		std::cout << "No Volume Given." << std::endl;
		std::cout << "*********************************************************************" <<std::endl;
		return;
	}

	if (!transferFunction) {												// Check if a transfer function was given
		transferFunction = new TransferFunction();							// If not create a new one
		transferFunction->buildDefault();									// Build the default transfer function
		transferFunction->computeTransferFunction();						// Compute the transfer function
	}

	std::cout << 
		"- Recursively Building Cubes for Empty Space Leaping" 
		<< std::endl; 
	VolumeCube C(0.f, 0.f, 0.f, 1.f, 1.f, 1.f);								// Define cube
	recursiveVolumeBuild(C);												// Recursively divided cube and remove empty space
	std::cout << "- Adding Vertices for Cubes" << std::endl; 
	buildCubes();															// Build the vertex buffer object for all the sub cubes
	std::cout << "- Cubes Created" << std::endl;
}
GLuint Volume::createVolume() {												// Loads the volume data onto the GPU
	GLuint volume_texture;													// ID for GPU texture
	glGenTextures(1, &volume_texture);										// Generate texture
	glBindTexture(GL_TEXTURE_3D, volume_texture);							// Bind texture
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);	
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);		// Set the MAG_FILTER to interpolate linearly between the closest pixels
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);		// Set the MIN_FILTER to interpolate linearly between the closest pixels
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);	// Clamp the X value to boarder when doing texture look ups
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);	// Clamp the Y value to boarder when doing texture look ups
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);	// Clamp the Z value to boarder when doing texture look ups
	glTexImage3D(															// Create texture
		GL_TEXTURE_3D,														// Texture is of type TEXTURE_3D
		0,																	// Use base level of detail
		GL_RED,																// Internal format is GL_RED (one color value for each pixel)
		volumeData->getVolumeWidth(),										// Set the width of the volume
		volumeData->getVolumeHeight(),										// Set the height of the volume
		volumeData->getVolumeDepth(),										// Set the depth of the volume
		0,																	// Set the boarder to zero
		GL_RED,																// Texture format is GL_RED (one color value for each pixel)
		GL_UNSIGNED_BYTE,													// Size of each voxel in the texture
		volumeData->getData());												// Copy over the volume data from the CPU to the GPU

	return volume_texture;													// Return texture ID
}

int Volume::sampleVolume(int x, int y, int z)								// Sample the volume
{
	int width = volumeData->getVolumeWidth();								// Get the volume width
	int height = volumeData->getVolumeHeight();								// Get the volume height
	int depth = volumeData->getVolumeDepth();								// Get the volume depth

	GLubyte* data = volumeData->getData();									// Get the volume data

	x = (int)clamp(x, 0, width - 1);										// Find the x value clamped to the boarder
	y = (int)clamp(y, 0, height - 1);										// Find the y value clamped to the boarder
	z = (int)clamp(z, 0, depth - 1);										// Find the z value clamped to the boarder

	return (int)data[x + (y * width) + (z * width * height)];				// Get the data value at the given coordinates (x, y, z)
}

float Volume::sampleVolume3DWithTransfer(Eigen::Vector3f& min, Eigen::Vector3f& max)
{
	float result = 0.0f;

	for (int x = (int)floorf(min.x()); x <= (int)ceilf(max.x()); x += 3)
	{
		for (int y = (int)floorf(min.y()); y <= (int)ceilf(max.y()); y += 3)
		{
			for (int z = (int)floorf(min.z()); z <= (int)ceilf(max.z()); z += 3)
			{
				//sample the volume to get the iso value
				int isovalue = sampleVolume(x, y, z);

				//accumulate the opacity from the transfer function
				result += (float)transferFunction->getTransferData()[4*isovalue + 3];
			}
		}
	}

	return result;
}

void Volume::recursiveVolumeBuild(VolumeCube C)
{
	int width = volumeData->getVolumeWidth();
	int height = volumeData->getVolumeHeight();
	int depth = volumeData->getVolumeDepth();

	//stop when the current cube is less than or equal to the EMPTY_SPACE_SIZE
	if (C.Width <= EMPTY_SPACE_SIZE)
	{
		//add the min/max vertex to the list
		Eigen::Vector3f min = Eigen::Vector3f(C.X, C.Y, C.Z);
		Eigen::Vector3f max = Eigen::Vector3f(
			C.X + C.Width, 
			C.Y + C.Height, 
			C.Z + C.Depth);
		Eigen::Vector3f min_pi = Eigen::Vector3f(
			C.X * width,
			C.Y * height,
			C.Z * depth);
		Eigen::Vector3f max_pi = Eigen::Vector3f(
			(C.X + C.Width) * width,
			(C.Y + C.Height) * height,
			(C.Z + C.Depth) * depth);

		//additively sample the transfer function and check if there are any
		//samples that are greater than zero
		float opacity = sampleVolume3DWithTransfer(min_pi, max_pi);

		if (opacity > 0.0f)
		{
			mCubes.push_back(C);
		}
		return;
	}

	float newWidth = C.Width / 2.f;
	float newHeight = C.Height / 2.f;
	float newDepth = C.Depth / 2.f;

	///  SubGrid        r | c | d
	///  Front:
	///  Top-Left    :  0   0   0
	///  Top-Right   :  0   1   0
	///  Bottom-Left :  1   0   0
	///  Bottom-Right:  1   1   0
	///  Back:
	///  Top-Left    :  0   0   1
	///  Top-Right   :  0   1   1
	///  Bottom-Left :  1   0   1
	///  Bottom-Right:  1   1   1
	for (float r = 0; r < 2; r++)
	{
		for (float c = 0; c < 2; c++)
		{
			for (float d = 0; d < 2; d++)
			{
				VolumeCube cube = VolumeCube(C.Left() + c * (newWidth),
					C.Top() + r * (newHeight),
					C.Front() + d * (newDepth),
					newWidth,
					newHeight,
					newDepth);

				recursiveVolumeBuild(cube);
			}
		}
	}
}

void Volume::buildCubes() {							// Calculates the vertices and indices of the polygon used in Empty Space Leaping
	BuildCubesManager manager;						// Create ne BuildCubesManager
	manager.setup(&mCubes, &mVertices, &mIndices);	// Setup the manager
	manager.buildCubes();							// Build the polygon
	
	mCubes.clear();									// Done and clear cubes from vector
}

void Volume::buildVertBuffer()
{
	VertexPositionColor* data = new VertexPositionColor[mVertices.size()];		// Build array for vertices
	std::copy(mVertices.begin(), mVertices.end(), data);						// Copy vertices from vector

	GLuint* indices = new GLuint [mIndices.size()];								// Build array for the indices
	std::copy(mIndices.begin(), mIndices.end(), indices);						// Copy indices from vector

	unsigned int mNumVertices = mVertices.size();								// Get the number of vertices
	mNumIndices = mIndices.size();												// Get the number of indices

	// Create VBO for vertices
	glGenBuffersARB( 1, &cubeVerticesVBO );										// Create VBO

	glBindBufferARB( GL_ARRAY_BUFFER_ARB, cubeVerticesVBO );					// Bind VBO buffer
	glBufferDataARB( GL_ARRAY_BUFFER_ARB,										// Copy the vertex data to the VBO
		mNumVertices * sizeof(VertexPositionColor),								// Get the size of data
		&data[0],																// Give the data for the vertices
		GL_STATIC_DRAW_ARB );													// Tell the buffer that the data is not going to change
	
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );									// Unbind VBO buffer
	
	// Create VBO for indices
	glGenBuffersARB( 1, &cubeIndicesVBO );										// Create buffer

	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, cubeIndicesVBO );				// Bind buffer
	glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,								// Copy the indices data to the buffer
		mNumIndices * sizeof(GLuint),											// Get the size of indices
		&indices[0],															// Give the data for the indices
		GL_STATIC_DRAW_ARB );													// Tell the buffer that the data is not going to change

	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, 0 );							// Unbind buffer

	delete[] indices;															// Delete indices data from CPU since it's now saved on the GPU
	delete[] data;																// Delete vertices data from CPU since it's now saved on the GPU

	mVertices.clear();															// Clear vertices vector
	mIndices.clear();															// Clear indices vector
}

void Volume::unbindFBO() {
	// 'unbind' the FBO. things will now be drawn to screen as usual
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

GLuint Volume::setupFBO() {														// Create a new frame buffer for off screen rendering
	GLuint fbo_handle;

	glGenFramebuffersEXT(1, &fbo_handle);										// Create buffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);						// Bind buffer

	// The depth buffer
	glGenRenderbuffers(1, &depthrenderbuffer);									// Create depth buffer
	glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);						// Bind buffer

	glRenderbufferStorage(GL_RENDERBUFFER,										// Create storage
		GL_DEPTH_COMPONENT,														// Specify that the internal format is the depth component
		TEXTURE_WIDTH, TEXTURE_HEIGHT);											// Set storage width and height
	glFramebufferRenderbuffer(GL_FRAMEBUFFER,									// Attach depth buffer to frame buffer
		GL_DEPTH_ATTACHMENT,													// Specify that the internal format is the depth component
		GL_RENDERBUFFER, depthrenderbuffer);									// Attach depth render buffer texture to the frame buffer

	errcheck();																	// Check for errors

	unbindFBO();																// Unbind frame buffer object

	return fbo_handle;															// Return new frame buffer
}

bool Volume::bindFBO(GLuint fbo_handle, GLuint *fbo_texture, GLsizei size) {	// Bind frame buffer and attach textures for rendering
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);						// Bind frame buffer

	if (size > GL_MAX_COLOR_ATTACHMENTS)										// Limit number of color attachments to the GL_MAX_COLOR_ATTACHMENTS
		size = GL_MAX_COLOR_ATTACHMENTS;

	for (int i = 0; i < size; i++)
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,							// Set fbo_texture as our color attachment #0
			GL_COLOR_ATTACHMENT0_EXT + i,
			GL_TEXTURE_2D, fbo_texture[i], 0);

	//errcheck();

	GLenum dbuffers[GL_MAX_COLOR_ATTACHMENTS];						 			// Set the list of draw buffers.

	for (int i = 0; i < size; i++)
		dbuffers[i] = GL_COLOR_ATTACHMENT0_EXT + i;

	glDrawBuffers(size, dbuffers);												// Set Draw buffers
	
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)		// Always check that our frame buffer is ok
		return false;

	return true;
}

int Volume::setupCg(CGcontext *context, CGprogram *fProgram,					// Setup new CG shader program
			CGprofile *fragmentProfile, char *file) {											
	*context = cgCreateContext();												// Create context for program

	if (!cgIsContext(*context)) {												// Check if context was successfully created
		fprintf(stderr, "Error: %s\n", "Failed To Create Cg Context");
		return 1;
	}

	*fragmentProfile = cgGLGetLatestProfile(CG_GL_FRAGMENT);					// Get the latest fragment profile

	printf("fragment profile: %s\n",
	       cgGetProfileString(cgGLGetLatestProfile(CG_GL_FRAGMENT)));

	if (*fragmentProfile == CG_PROFILE_UNKNOWN) {								// Check if fragment profile was valid
		fprintf(stderr, "Error: %s\n", "Invalid profile type");
		return 1;
	}

	cgGLSetOptimalOptions(*fragmentProfile);									// Get optimal options for the fragment profile

	*fProgram = cgCreateProgramFromFile(*context, CG_SOURCE, file,				// Create CG program from the given file
			*fragmentProfile, "main", NULL);

	if (!*fProgram) {															// Check if program was successfully created
		printf("Couldn't create fragment program.\n");
		return 1;
	}

	if (!cgIsProgramCompiled(*fProgram)) {										// Check if program was compiled
		cgCompileProgram(*fProgram);											// If not compile
	}

	if (!cgIsProgramCompiled(*fProgram)) {										// Check if there was an error in compiling the code
		printf("Couldn't compile fragment program.\n");
	}
	
	cgGLEnableProfile(*fragmentProfile);										// Enable the fragment profile

	if (*fProgram) {
		cgGLLoadProgram(*fProgram);												// Load the program
	} else {
		printf("Couldn't load fragment program.\n");
		CheckCgError();
		return 1;
	}

	cgGLDisableProfile(*fragmentProfile);										// Disable the fragment profile

	return 0;
}

void Volume::setIsoValue(float value) {											// Set the threshold for the isoValue
	isoValue = value;

	if (isoValue > 1.0f)
		isoValue = 1.0f;
	else if (isoValue < 0.0f)
		isoValue = 0.0f;
}


void Volume::increaseIsoValue(float value) {									// Increase the threshold for the isoValue
	isoValue += value;

	if (isoValue > 1.0f)
		isoValue = 1.0f;
	else if (isoValue < 0.0f)
		isoValue = 0.0f;
}

void Volume::render(Camera* camera) {
	int viewWidth = camera->getWidth();						// Get Camera Width
	int viewHeight = camera->getHeight();					// Get Camera Height
	
	glViewport(0, 0, TEXTURE_WIDTH, TEXTURE_HEIGHT);		// Set Viewport
	glMatrixMode(GL_PROJECTION);							// Update projection
	glPushMatrix();
	glLoadIdentity();										// Load Identity
	gluPerspective(camera->getFOV(),						// Set Field of View
		(GLfloat)viewWidth/(GLfloat)viewHeight,				// Set aspect ratio
		camera->getNearClipping(),							// Set near clipping
		camera->getFarClipping());							// Set far clipping
	glMatrixMode(GL_MODELVIEW);								// Change back to model view mode
	
	glPushMatrix();											// set where to start the current object
	
	glScalef(												// Scale Volume to proper proportions
		1.0f,												// Scale width (Always 1.0f)
		(spacingY/spacingX)*								// Scale the voxel spacing height relative to spacing width
			((float)volumeHeight/volumeWidth),				// Scale the height dimension relative to the width
		(spacingZ/spacingX)*								// Scale the voxel spacing depth relative to spacing width
			((float)volumeDepth/volumeWidth));				// Scale the depth dimension relative to the width
			
	glScalef(scale.x(), scale.y(), scale.z());				// Scale the volume

	glTranslatef(position.x(), position.y(), position.z());	// set position of the texture cube

	// Convert rotation quaternion into axis angle
	float rotationScale = sqrt(								// Get length of quaternion
		rotation.x() * rotation.x() + 
		rotation.y() * rotation.y()  + 
		rotation.z() * rotation.z());

	float x = rotation.x() / rotationScale;					// Get axis x value
	float y = rotation.y() / rotationScale;					// Get axis y value
	float z = rotation.z() / rotationScale;					// Get axis z value
	float angle = (acos(rotation.w())*2.0f)*(180.f/M_PI);	// Get rotation angle

	glRotatef(angle, x, y, z);								// set rotation of the texture cube
	
	glTranslatef(-0.5, -0.5, -0.5);							// center the texture cube

	// Bind the vertices's VBO
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, cubeVerticesVBO );
	glVertexPointer( 3, GL_FLOAT, sizeof(VertexPositionColor), MEMBER_OFFSET(VertexPositionColor,m_Pos) );
	glColorPointer( 3, GL_FLOAT, sizeof(VertexPositionColor), MEMBER_OFFSET(VertexPositionColor,m_Color) );

	// We need to enable the client stats for the vertex attributes we want 
	// to render even if we are not using client-side vertex arrays.
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_INDEX_ARRAY);

	// Bind the indices VBO
	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, cubeIndicesVBO );

	glEnable( GL_TEXTURE_2D );							// Enable 2D textures
	
	bindFBO(FBO, &back_facing,  1);						// Render to our frame buffer using the back_facing texture

	glClearDepth(0.0f);									// Depth Buffer Setup
	glDepthFunc(GL_GREATER);							// The Type Of Depth Testing To Do

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_FRONT);								// Remove front facing face's
	glDrawElements(	GL_TRIANGLES,						// Render Back Facing
		mNumIndices, GL_UNSIGNED_INT, 0);
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer

	bindFBO(FBO, &front_facing, 1);						// Render to our frame buffer using the front_facing texture
	
	glClearDepth(1.0f);									// Depth Buffer Setup
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_BACK);								// Remove back facing face's
	glDrawElements(	GL_TRIANGLES,						// Render Front Facing
		mNumIndices, GL_UNSIGNED_INT, 0);				
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer

	// Unbind buffers so client-side vertex arrays still work.
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );
	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, 0 );

	// Disable the client side arrays again.
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);;
	glDisableClientState(GL_INDEX_ARRAY);

	glPopMatrix();												// Restore state

	// First Pass Render Volume
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, TEXTURE_WIDTH, TEXTURE_HEIGHT, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	bindFBO(FBO, &colorTexture, 1);								// Render to our frame buffer using the front_facing texture

	cgGLEnableProfile(fragmentProfile);
	cgGLBindProgram(fProgramFirstPass);
	CheckCgError();

	// enable Cg shader and texture (a 'compute' fragment program)
	cgGLSetTextureParameter(cgFrontTexData, front_facing);		// Bind front facing render to cgFrontTexData
	cgGLSetTextureParameter(cgBackTexData, back_facing);		// Bind back facing render to cgBackTexData
	cgGLSetTextureParameter(cgVolumeTexData, volume_texture);	// Bind the voulume_texture to cgVolumeTexData
	cgGLSetTextureParameter(cgTransferTexData, transferTexture);// Bind the transferTexture to cgTransferTexData
	cgGLEnableTextureParameter(cgFrontTexData);					// Enable cgFrontTexData
	cgGLEnableTextureParameter(cgBackTexData);					// Enable cgBackTexData
	cgGLEnableTextureParameter(cgVolumeTexData);				// Enable cgVolumeTexData
	cgGLEnableTextureParameter(cgTransferTexData);				// Enable cgTransferTexData

	cgGLSetParameter1f(cgisoValue, isoValue);					// Set the threshold for the isoValue

	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	
	glBegin(GL_TRIANGLE_FAN);
	glTexCoord2f(0, 1);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1, 1);
	glVertex3f(TEXTURE_WIDTH, 0, 0);
	glTexCoord2f(1, 0);
	glVertex3f(TEXTURE_WIDTH, TEXTURE_HEIGHT, 0);
	glTexCoord2f(0, 0);
	glVertex3f(0, TEXTURE_HEIGHT, 0);
	glEnd();

	cgGLDisableTextureParameter(cgFrontTexData);				// Disable cgFrontTexData
	cgGLDisableTextureParameter(cgBackTexData);					// Disable cgBackTexData
	cgGLDisableTextureParameter(cgVolumeTexData);				// Disable cgVolumeTexData
	cgGLDisableTextureParameter(cgTransferTexData);				// Disable cgTransferTexData
	
	// disable shader
	cgGLDisableProfile(fragmentProfile);
	CheckCgError();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);						// Unbind frame buffer
	
	glPopMatrix();												// end the current object transformations
	
	// Second Pass Render Volume
	glViewport(0, 0, viewWidth, viewHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, TEXTURE_WIDTH, TEXTURE_HEIGHT, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	cgGLEnableProfile(fragmentProfile);
	cgGLBindProgram(fProgramSecondPass);
	CheckCgError();

	// enable Cg shader and texture (a 'compute' fragment program)
	cgGLSetTextureParameter(cgColorTexData, colorTexture);		// Bind color render to cgColorTexData
	cgGLEnableTextureParameter(cgColorTexData);					// Enable cgColorTexData

	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	
	glBegin(GL_TRIANGLE_FAN);
	glTexCoord2f(0, 1);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1, 1);
	glVertex3f(TEXTURE_WIDTH, 0, 0);
	glTexCoord2f(1, 0);
	glVertex3f(TEXTURE_WIDTH, TEXTURE_HEIGHT, 0);
	glTexCoord2f(0, 0);
	glVertex3f(0, TEXTURE_HEIGHT, 0);
	glEnd();
	
	cgGLDisableTextureParameter(cgColorTexData);				// Disable cgColorTexData
	
	// disable shader
	cgGLDisableProfile(fragmentProfile);
	CheckCgError();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();												//end the current object transformations
	
}
