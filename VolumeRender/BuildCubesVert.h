#pragma once
#include "Thread.h"

#include <GL/glew.h>

#include "kdtree++/kdtree.hpp"
#include <algorithm>
#include <vector>

struct kdtreeNode
{
	typedef double value_type;

	double xyz[3];
	size_t index;

	value_type operator[](size_t n) const
	{
		return xyz[n];
	}

	double distance( const kdtreeNode &node);
};

typedef KDTree::KDTree<3, kdtreeNode> treeType;

class VolumeCube;
struct VertexPositionColor;

class BuildCubesVert : public Thread
{
public:
	BuildCubesVert(void);
	~BuildCubesVert(void);

	void setup(HANDLE mutex,
		treeType* kdTree,
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

	treeType* kdCubes;
	
	int cubeIndex;

	HANDLE g_hMutex;
};

