#include "StdAfx.h"
#include "Volume.h"

#include <math.h>
#include <stddef.h>

#define TEXTURE_SIZE 2048
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))

#define VOLUME_TEX_SIZE 128

struct float3 {
	float x, y, z;
	
	float3() {
		x = 0.f;
		y = 0.f;
		z = 0.f;
	}

	float3(float _x, float _y, float _z) {
		x = _x;
		y = _y;
		z = _z;
	}

	float3 operator - (const float3 &b) const {
		return float3(x - b.x, y - b.y, z - b.z);
	}
};

struct Vertex
{
    float3 m_Pos;
    float3 m_Color;
	float3 m_Normal;

	Vertex() {}

	Vertex(float3 pos, float3 norm) {
		m_Pos = pos;
		m_Color = pos;
		m_Normal = norm;
	}
};

float length(float3 p) {
	return sqrtf(p.x*p.x + p.y*p.y + p.z*p.z);
}

// create a test volume texture, here you could load your own volume
GLuint create_volumetexture()
{
	int size = VOLUME_TEX_SIZE*VOLUME_TEX_SIZE*VOLUME_TEX_SIZE* 4;
	GLubyte *data = new GLubyte[size];

	for(int x = 0; x < VOLUME_TEX_SIZE; x++)
	{for(int y = 0; y < VOLUME_TEX_SIZE; y++)
	{for(int z = 0; z < VOLUME_TEX_SIZE; z++)
	{
		data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = z%250;
		data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%250;
		data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 230;
	  	
		float3 p =	float3(x,y,z)- float3(VOLUME_TEX_SIZE-20,VOLUME_TEX_SIZE-30,VOLUME_TEX_SIZE-30);
		bool test = (length(p) < 42);
		if(test)
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 0;

		p =	float3(x,y,z)- float3(VOLUME_TEX_SIZE/2,VOLUME_TEX_SIZE/2,VOLUME_TEX_SIZE/2);
		test = (length(p) < 24);
		if(test)
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 0;

		
		if(x > 20 && x < 40 && y > 0 && y < VOLUME_TEX_SIZE && z > 10 &&  z < 50)
		{
			
			data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 100;
		    data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%100;
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		}

		if(x > 50 && x < 70 && y > 0 && y < VOLUME_TEX_SIZE && z > 10 &&  z < 50)
		{
			
			data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%100;
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		}

		if(x > 80 && x < 100 && y > 0 && y < VOLUME_TEX_SIZE && z > 10 &&  z < 50)
		{
			
			data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 70;
		    data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%100;
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		}

		p =	float3(x,y,z)- float3(24,24,24);
		test = (length(p) < 40);
		if(test)
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 0;

			
	}}}


	GLuint volume_texture;
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glGenTextures(1, &volume_texture);
	glBindTexture(GL_TEXTURE_3D, volume_texture);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, VOLUME_TEX_SIZE, VOLUME_TEX_SIZE, VOLUME_TEX_SIZE, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

	delete []data;
	printf("volume texture created\n");

	return volume_texture;
}

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
	}
}

GLuint newTexture(int width, int height) {
	GLuint texture;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F_ARB, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
	return texture;
}

Volume::Volume(void)
{
	g_uiVerticesVBO = 0;
	initialized = false;
}


Volume::~Volume(void)
{
	if ( g_uiVerticesVBO != 0 )
	{
		glDeleteBuffersARB( 1, &g_uiVerticesVBO );
		g_uiVerticesVBO = 0;
	}
}

void Volume::init() {
	FBO = setupFBO();

	front_facing = newTexture(TEXTURE_SIZE, TEXTURE_SIZE);
	back_facing = newTexture(TEXTURE_SIZE, TEXTURE_SIZE);

	char file[] = "shader/raycast.cg";
	if (setupCg(&context, &fProgram, &fragmentProfile, file)) {
		fprintf(stderr, "Error: %s\n", "Initializing Cg");
		CheckCgError();
	}

	cgFrontTexData = cgGetNamedParameter(fProgram, "frontTexData");
	cgBackTexData = cgGetNamedParameter(fProgram, "backTexData");
	cgVolumeTexData = cgGetNamedParameter(fProgram, "volume_tex");
	cgStepSize = cgGetNamedParameter(fProgram, "stepSize");

	createCube(1.0f, 1.0f, 1.0f);

	volume_texture = create_volumetexture();

	initialized = true;
}

bool Volume::needsInit() {
	return !initialized;
}

void Volume::reset() {
	if ( g_uiVerticesVBO != 0 )
	{
		glDeleteBuffersARB( 1, &g_uiVerticesVBO );
		g_uiVerticesVBO = 0;
	}
}

void Volume::createCube(float x, float y, float z) {
	// Define the 24 vertices of a unit cube
	Vertex g_Vertices[24] = {
		// Back side
		Vertex( float3( 0.0, 0.0, 0.0), float3(0.0, 0.0, -1.0)),
		Vertex( float3( 0.0, y, 0.0), float3(0.0, 0.0, -1.0)),
		Vertex( float3( x, y, 0.0), float3(0.0, 0.0, -1.0)),
		Vertex( float3( x, 0.0, 0.0), float3(0.0, 0.0, -1.0)),
		// Front side
		Vertex( float3( 0.0, 0.0, z), float3(0.0, 0.0, 1.0)),
		Vertex( float3( x, 0.0, z), float3(0.0, 0.0, 1.0)),
		Vertex( float3( x, y, z), float3(0.0, 0.0, 1.0)),
		Vertex( float3( 0.0, y, z), float3(0.0, 0.0, 1.0)),
		// Top side
		Vertex( float3( 0.0, y, 0.0), float3(0.0, 1.0, 0.0)),
		Vertex( float3( 0.0, y, z), float3(0.0, 1.0, 0.0)),
		Vertex( float3( x, y, z), float3(0.0, 1.0, 0.0)),
		Vertex( float3( x, y, 0.0), float3(0.0, 1.0, 0.0)),
		// Bottom side
		Vertex( float3( 0.0, 0.0, 0.0), float3(0.0, -1.0, 0.0)),
		Vertex( float3( x, 0.0, 0.0), float3(0.0, -1.0, 0.0)),
		Vertex( float3( x, 0.0, z), float3(0.0, -1.0, 0.0)),
		Vertex( float3( 0.0, 0.0, z), float3(0.0, -1.0, 0.0)),
		// Left Side
		Vertex( float3( 0.0, 0.0, 0.0), float3(-1.0, 0.0, 0.0)),
		Vertex( float3( 0.0, 0.0, z), float3(-1.0, 0.0, 0.0)),
		Vertex( float3( 0.0, y, z), float3(-1.0, 0.0, 0.0)),
		Vertex( float3( 0.0, y, 0.0), float3(-1.0, 0.0, 0.0)),
		// Right Side
		Vertex( float3( x, 0.0, 0.0), float3(1.0, 0.0, 0.0)),
		Vertex( float3( x, y, 0.0), float3(1.0, 0.0, 0.0)),
		Vertex( float3( x, y, z), float3(1.0, 0.0, 0.0)),
		Vertex( float3( x, 0.0, z), float3(1.0, 0.0, 0.0))
	};

	// Create VBO
	glGenBuffersARB( 1, &g_uiVerticesVBO );

	// Copy the vertex data to the VBO
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, g_uiVerticesVBO );
	glBufferDataARB( GL_ARRAY_BUFFER_ARB, sizeof(g_Vertices), g_Vertices, GL_STATIC_DRAW_ARB );
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );
}

void Volume::unbindFBO() {
	// 'unbind' the FBO. things will now be drawn to screen as usual
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

GLuint Volume::setupFBO() {									// Create a new frame buffer for off screen rendering
	GLuint fbo_handle;

	glGenFramebuffersEXT(1, &fbo_handle);					// Create buffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);	// Bind buffer

	// The depth buffer
	GLuint depthrenderbuffer;
	glGenRenderbuffers(1, &depthrenderbuffer);				// Create depth buffer
	glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);	// Bind buffer
	glRenderbufferStorage(GL_RENDERBUFFER,					// Create storage
		GL_DEPTH_COMPONENT, 
		TEXTURE_SIZE, TEXTURE_SIZE);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER,				// Attach depth buffer to frame buffer
		GL_DEPTH_ATTACHMENT, 
		GL_RENDERBUFFER, depthrenderbuffer);

	errcheck();												// Check for errors

	unbindFBO();											// Unbind frame buffer object

	return fbo_handle;										// Return new frame buffer
}

bool Volume::bindFBO(GLuint fbo_handle, GLuint fbo_texture) {	// Bind frame buffer and attach texture for rendering
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);		// Bind frame buffer

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,				// Set fbo_texture as our color attachment #0
		GL_COLOR_ATTACHMENT0_EXT,
		GL_TEXTURE_2D, fbo_texture, 0);

	//errcheck();

	GLenum dbuffers[2] = { GL_COLOR_ATTACHMENT0_EXT };			// Set the list of draw buffers.
	glDrawBuffers(1, dbuffers);									// Set Draw buffers
	
	// Always check that our frame buffer is ok
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		return false;

	return true;
}

int Volume::setupCg(CGcontext *context, CGprogram *fProgram, 
			CGprofile *fragmentProfile, char *file) {
	cgSetErrorCallback(CheckCgError);

	*context = cgCreateContext();

	if (!cgIsContext(*context)) {
		fprintf(stderr, "Error: %s\n", "Failed To Create Cg Context");
		return 1;
	}

	*fragmentProfile = cgGLGetLatestProfile(CG_GL_FRAGMENT);

	printf("fragment profile: %s\n",
	       cgGetProfileString(cgGLGetLatestProfile(CG_GL_FRAGMENT)));

	if (*fragmentProfile == CG_PROFILE_UNKNOWN) {
		fprintf(stderr, "Error: %s\n", "Invalid profile type");
		return 1;
	}

	cgGLSetOptimalOptions(*fragmentProfile);

	*fProgram = cgCreateProgramFromFile(*context, CG_SOURCE, file,
			*fragmentProfile, "main", NULL);

	if (!*fProgram) {
		printf("Couldn't create fragment program.\n");
		return 1;
	}

	if (!cgIsProgramCompiled(*fProgram)) {
		cgCompileProgram(*fProgram);
	}

	if (!cgIsProgramCompiled(*fProgram)) {
		printf("Couldn't compile fragment program.\n");
	}
	
	cgGLEnableProfile(*fragmentProfile);

	if (*fProgram) {
		cgGLLoadProgram(*fProgram);
	} else {
		printf("Couldn't load fragment program.\n");
		CheckCgError();
		return 1;
	}

	cgGLDisableProfile(*fragmentProfile);

	return 0;
}

void Volume::render(int w, int h) {
	glPushMatrix(); //set where to start the current object

	glTranslatef(-0.5,-0.5,-0.5); // center the texturecube

	// We need to enable the client stats for the vertex attributes we want 
	// to render even if we are not using client-side vertex arrays.
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	// Bind the vertices's VBO
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, g_uiVerticesVBO );
	glVertexPointer( 3, GL_FLOAT, sizeof(Vertex), MEMBER_OFFSET(Vertex,m_Pos) );
	glColorPointer( 3, GL_FLOAT, sizeof(Vertex), MEMBER_OFFSET(Vertex,m_Color) );
	glNormalPointer( GL_FLOAT, sizeof(Vertex), MEMBER_OFFSET(Vertex,m_Normal) );
	
	glEnable(GL_TEXTURE_2D);							// Enable 2D textures
	
	bindFBO(FBO, front_facing);							// Render to our frame buffer using the front_facing texture
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_BACK);								// Remove back facing face's
	glDrawArrays( GL_QUADS, 0, 24 );					// Render Front Facing
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's

	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer
	
	bindFBO(FBO, back_facing);							// Render to our frame buffer using the back_facing texture
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_FRONT);								// Remove front facing face's
	glDrawArrays( GL_QUADS, 0, 24 );					// Render Back Facing
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer

	// Unbind buffers so client-side vertex arrays still work.
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

	// Disable the client side arrays again.
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glPopMatrix();										// Restore state

	// Render Volume
	glViewport(0, 0, TEXTURE_SIZE, TEXTURE_SIZE);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, TEXTURE_SIZE, TEXTURE_SIZE, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	cgGLEnableProfile(fragmentProfile);
	cgGLBindProgram(fProgram);
	CheckCgError();

	cgGLSetParameter1f(cgStepSize, 1.0f/50.0f);					// Set the incremental step size of the ray cast

	// enable Cg shader and texture (a 'compute' fragment program)
	cgGLSetTextureParameter(cgFrontTexData, front_facing);		// Bind front facing render to cgFrontTexData
	cgGLSetTextureParameter(cgBackTexData, back_facing);		// Bind back facing render to cgBackTexData
	cgGLSetTextureParameter(cgVolumeTexData, volume_texture);	// Bind the voulume_texture to cgVolumeTexData
	cgGLEnableTextureParameter(cgFrontTexData);					// Enable cgFrontTexData
	cgGLEnableTextureParameter(cgBackTexData);					// Enable cgBackTexData
	cgGLEnableTextureParameter(cgVolumeTexData);				// Enable cgVolumeTexData
	
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	
	glBegin(GL_TRIANGLE_FAN);
	glTexCoord2f(0, 1);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1, 1);
	glVertex3f(TEXTURE_SIZE, 0, 0);
	glTexCoord2f(1, 0);
	glVertex3f(TEXTURE_SIZE, TEXTURE_SIZE, 0);
	glTexCoord2f(0, 0);
	glVertex3f(0, TEXTURE_SIZE, 0);
	glEnd();
	
	// disable shader
	cgGLDisableProfile(fragmentProfile);
	CheckCgError();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix(); //end the current object transformations
}
