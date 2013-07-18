#pragma once
#include "BuildCubesVert.h"

#define MAX_THREADS 8

class BuildCubesManager
{
public:
	BuildCubesManager(void);
	~BuildCubesManager(void);

	void setup(
		std::vector<VolumeCube>* cubes, 
		std::vector<VertexPositionColor>* vertices,
		std::vector<GLuint>* indices);

	void buildCubes();
private:
	BuildCubesVert threads[MAX_THREADS];

	std::vector<VolumeCube>* mCubes;

	HANDLE		g_hMutex;

	treeType kdTree;
};


