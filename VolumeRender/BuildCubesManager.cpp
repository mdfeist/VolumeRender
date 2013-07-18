#include "StdAfx.h"
#include "BuildCubesManager.h"

#include "VolumeCube.h"
#include "VolumeStructs.h"

BuildCubesManager::BuildCubesManager(void) {
	g_hMutex = CreateMutex(
		NULL,
		//(LPSECURITY_ATTRIBUTES)SYNCHRONIZE, 
		FALSE, 
		NULL);
}

BuildCubesManager::~BuildCubesManager(void) {
	// Free the mutex
	CloseHandle(g_hMutex);
}

void BuildCubesManager::setup(
	std::vector<VolumeCube>* cubes, 
	std::vector<VertexPositionColor>* vertices,
	std::vector<GLuint>* indices)
{
	mCubes = cubes;

	for (int i = 0; i < MAX_THREADS; i++)
		threads[i].setup(g_hMutex, cubes, vertices, indices);
}

void BuildCubesManager::buildCubes() {
	for (unsigned int i = 0; i < MAX_THREADS; i++) {
		threads[i].buildCubes(i);
	}

	for (unsigned int i = MAX_THREADS; i < mCubes->size(); i++) {
		bool noThreadFree = true;
		while (noThreadFree) {
			for (int threadIndex = 0; threadIndex < MAX_THREADS; threadIndex++) {
				if (!threads[threadIndex].isAlive()) {
					noThreadFree = false;
					threads[threadIndex].buildCubes(i);
				}
			}
		}
	}

	bool threadIsAlive = true;
	while (threadIsAlive) {
		threadIsAlive = false;

		for (unsigned int i = 0; i < MAX_THREADS; i++) {
			if (threads[i].isAlive())
				threadIsAlive = true;
		}
	}
}

