/*
 * OpenGL.cpp
 *
 *  Created on: 2012-11-12
 *      Author: mdfeist
 */
#include "StdAfx.h"
#include "OpenGL.h"

#include <stdio.h>

void errcheck() {
	static GLenum errCode;
	const GLubyte *errString;

	if ((errCode = glGetError()) != GL_NO_ERROR) {
		errString = gluErrorString(errCode);
		fprintf(stderr, "OpenGL Error: %s\n", errString);
	}
}

GLuint newTexture(int w, int h, uint16_t* buffer) {
	GLuint tex_handle;

	glEnable(GL_TEXTURE_2D);

	glGenTextures(1, &tex_handle);
	glBindTexture(GL_TEXTURE_2D, tex_handle);

	// set texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	// define texture
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F_ARB, w, h, 0, GL_RGBA, GL_FLOAT, buffer);

	glBindTexture(GL_TEXTURE_2D, 0);

	return tex_handle;
}

void unbindFBO() {
	// 'unbind' the FBO. things will now be drawn to screen as usual
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

GLuint setupFBO(int w, int h) {
	GLuint fbo_handle;

	glGenFramebuffersEXT(1, &fbo_handle);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);
	errcheck();

	unbindFBO();

	return fbo_handle;
}

void bindFBO(GLuint fbo_handle, GLuint fbo_texture) {
	static int first_time = 1;

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
			GL_TEXTURE_2D, fbo_texture, 0);

	//errcheck();

	//GLenum dbuffers[] = { GL_COLOR_ATTACHMENT0_EXT };
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	if (first_time) {
		first_time = 0;
	}

}

int setupCg(CGcontext *context, CGprogram *fProgram, 
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
		printf("Couldn't load vertex program.\n");
		return 1;
	}

	return 0;
}
