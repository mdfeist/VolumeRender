#include "StdAfx.h"
#include "Volume.h"

#include <stddef.h>

#define TEXTURE_SIZE 2048
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))

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

void errcheck() {
	static GLenum errCode;
	const GLubyte *errString;

	if ((errCode = glGetError()) != GL_NO_ERROR) {
		errString = gluErrorString(errCode);
		fprintf(stderr, "OpenGL Error: %s\n", errString);
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

	createCube(1.0f, 1.0f, 1.0f);

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

GLuint Volume::setupFBO() {								// Create a new frame buffer for off screen rendering
	GLuint fbo_handle;

	glGenFramebuffersEXT(1, &fbo_handle);					// Create buffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);	// Bind buffer

	// The depth buffer
	GLuint depthrenderbuffer;
	glGenRenderbuffers(1, &depthrenderbuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, TEXTURE_SIZE, TEXTURE_SIZE);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);

	errcheck();												// Check for errors

	unbindFBO();											// Unbind frame buffer object

	return fbo_handle;
}

bool Volume::bindFBO(GLuint fbo_handle, GLuint fbo_texture) {	// Bind frame buffer and attach texture for rendering
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);		// Bind frame buffer

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,				// Set fbo_texture as our color attachment #0
		GL_COLOR_ATTACHMENT0_EXT,
		GL_TEXTURE_2D, fbo_texture, 0);

	//errcheck();

	GLenum dbuffers[2] = { GL_COLOR_ATTACHMENT0_EXT };			// Set the list of draw buffers.
	glDrawBuffers(1, dbuffers);									// Set Draw buffers
	
	// Always check that our framebuffer is ok
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		return false;

	return true;
}

int Volume::setupCg(CGcontext *context, CGprogram *fProgram, 
			CGprofile *fragmentProfile, char *file) {
	*context = cgCreateContext();

	if (*context == NULL) {
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

	if (*fProgram)
		cgGLLoadProgram(*fProgram);
	else {
		printf("Couldn't load fragment program.\n");
		return 1;
	}

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
	
	// Render to our framebuffer
	bindFBO(FBO, front_facing);
	glEnable(GL_TEXTURE_2D);

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	// Render Front Facing
	glDrawArrays( GL_QUADS, 0, 24 );

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// Render to our framebuffer
	bindFBO(FBO, back_facing);

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	// Render Back Facing
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	glDrawArrays( GL_QUADS, 0, 24 );
	glDisable(GL_CULL_FACE);
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// Unbind buffers so client-side vertex arrays still work.
	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, 0 );
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

	// Disable the client side arrays again.
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glPopMatrix();

	// Render Volume
	glViewport(0, 0, TEXTURE_SIZE, TEXTURE_SIZE);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, TEXTURE_SIZE, TEXTURE_SIZE, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	glBindTexture(GL_TEXTURE_2D, back_facing);
	
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
	
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix(); //end the current object transformations

}
