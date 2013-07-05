/*
 * OpenGL.h
 *
 *  Created on: 2012-11-12
 *		Author: mdfeist
 */

#ifndef OPENGL_H_
#define OPENGL_H_

#ifdef __cplusplus
extern "C" {
#endif
	#include <GL/glew.h>

	#include <Cg/cg.h>
	#include <Cg/cgGL.h>

	#include <GL/glut.h>

	#include <stdint.h>

	void errcheck();

	GLuint newTexture(int w, int h, uint16_t* buffer);

	GLuint setupFBO(int w, int h);
	void bindFBO(GLuint fbo_handle, GLuint fbo_texture);
	void unbindFBO();

	int setupCg(CGcontext *context, CGprogram *fProgram,
			CGprofile *fragmentProfile, char *file);

#ifdef __cplusplus
}
#endif

#endif /* OPENGL_H_ */
