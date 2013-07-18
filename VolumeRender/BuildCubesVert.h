#pragma once
#include "Thread.h"

#include <GL/glew.h>

#include <vector>

class VolumeCube;
struct VertexPositionColor;

class BuildCubesVert : public Thread
{
public:
	BuildCubesVert(void);
	~BuildCubesVert(void);

	void setup(HANDLE mutex,
		std::vector<VolumeCube>* cubes, 
		std::vector<VertexPositionColor>* vertices,
		std::vector<GLuint>* indices);

	void buildCubes(int i);

	bool lock();
	void unlock();
protected:
	DWORD runThread();

private:
	std::vector<VolumeCube>* mCubes;
	std::vector<VertexPositionColor>* mVertices;
	std::vector<GLuint>* mIndices;

	int cubeIndex;

	HANDLE g_hMutex;
};

