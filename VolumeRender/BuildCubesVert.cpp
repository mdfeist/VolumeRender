#include "StdAfx.h"
#include "BuildCubesVert.h"

#include "VolumeCube.h"
#include "VolumeStructs.h"

double kdtreeNode::distance( const kdtreeNode &node)
{
	double x = xyz[0] - node.xyz[0];
	double y = xyz[1] - node.xyz[1];
	double z = xyz[2] - node.xyz[2];

	// this is not correct   return sqrt( x*x+y*y+z*z);

	// this is what kdtree checks with find_within_range()
	// the "manhattan distance" from the search point.
	// effectively, distance is the maximum distance in any one dimension.
	return max( fabs(x), max( fabs(y), fabs(z) ) );
}

const GLushort VolumeIndicesFront[] = { 4, 5, 1, 1, 0, 4, };
const GLushort VolumeIndicesTop[] = { 0, 1, 2, 2, 3, 0, };
const GLushort VolumeIndicesBack[] = { 3, 2, 6, 6, 7, 3, };
const GLushort VolumeIndicesBottom[] = { 7, 6, 5, 5, 4, 7, };
const GLushort VolumeIndicesLeft[] = { 4, 0, 3, 3, 7, 4, };
const GLushort VolumeIndicesRight[] = { 1, 5, 6, 6, 2, 1, };

BuildCubesVert::BuildCubesVert(void){
	threadHandle = NULL;
	cubeIndex = 0;
}


BuildCubesVert::~BuildCubesVert(void)
{
}

// Lock
bool BuildCubesVert::lock() {
	// Request ownership of mutex
	DWORD  dwWaitResult;
	while(true) {
		// Wait for Mutex to be free
		dwWaitResult = WaitForSingleObject(g_hMutex, INFINITE);
		switch (dwWaitResult) {
			// The thread got ownership of the mutex
		case WAIT_OBJECT_0: 
			return true;
			break; 

			// The thread got ownership of an abandoned mutex
			// The database is in an indeterminate state
		case WAIT_ABANDONED: 
			return false; 
			break;
		}
	}

	return false;
}

void BuildCubesVert::unlock() {
	ReleaseMutex(g_hMutex);
}

void BuildCubesVert::setup(
		HANDLE mutex,
		treeType* kdTree,
		std::vector<VolumeCube>* cubes, 
		std::vector<VertexPositionColor>* vertices,
		std::vector<GLuint>* indices) {
	g_hMutex = mutex;

	kdCubes = kdTree;

	mCubes = cubes;
	mVertices = vertices;
	mIndices = indices;
}

void BuildCubesVert::buildCubes(int i) {
	cubeIndex = i;
	start();
}

DWORD BuildCubesVert::runThread() {
	int i = cubeIndex;
	Eigen::Vector3f center = Eigen::Vector3f(
		(*mCubes)[i].X + (*mCubes)[i].Width/2.f, 
		(*mCubes)[i].Y + (*mCubes)[i].Height/2.f, 
		(*mCubes)[i].Z + (*mCubes)[i].Depth/2.f);

	int inside = 0;
	bool addFace[6];
	memset(addFace, true, 6);

	Eigen::Vector3f points[6];

	points[0] = center + Eigen::Vector3f((*mCubes)[i].Width, 0.f, 0.f);
	points[1] = center - Eigen::Vector3f((*mCubes)[i].Width, 0.f, 0.f);
	points[2] = center + Eigen::Vector3f(0.f, (*mCubes)[i].Height, 0.f);
	points[3] = center - Eigen::Vector3f(0.f, (*mCubes)[i].Height, 0.f);
	points[4] = center + Eigen::Vector3f(0.f, 0.f, (*mCubes)[i].Depth);
	points[5] = center - Eigen::Vector3f(0.f, 0.f, (*mCubes)[i].Depth);
	
	for (unsigned int p = 0; p < 6; p++) {
		kdtreeNode target;
		target.xyz[0] = points[p].x();
		target.xyz[1] = points[p].y();
		target.xyz[2] = points[p].z();

		std::pair<treeType::const_iterator,double> found = kdCubes->find_nearest(target);
		int k = found.first->index;

		//add the min/max vertex to the list
		Eigen::Vector3f t_min = Eigen::Vector3f(
			(*mCubes)[k].X, 
			(*mCubes)[k].Y,
			(*mCubes)[k].Z);
		Eigen::Vector3f t_max = Eigen::Vector3f(
			(*mCubes)[k].X + (*mCubes)[k].Width, 
			(*mCubes)[k].Y + (*mCubes)[k].Height, 
			(*mCubes)[k].Z + (*mCubes)[k].Depth);

		for (unsigned int p = 0; p < 6; p++) {
			if (points[p].x() > t_min.x() &&
				points[p].y() > t_min.y() &&
				points[p].z() > t_min.z() &&
				points[p].x() < t_max.x() &&
				points[p].y() < t_max.y() &&
				points[p].z() < t_max.z()) {
					addFace[p] = false;
					inside++;
			}
		}
	}

	/*
	for (unsigned int k = 0; k < (*mCubes).size(); k++) {
		//add the min/max vertex to the list
		Eigen::Vector3f t_min = Eigen::Vector3f(
			(*mCubes)[k].X, 
			(*mCubes)[k].Y,
			(*mCubes)[k].Z);
		Eigen::Vector3f t_max = Eigen::Vector3f(
			(*mCubes)[k].X + (*mCubes)[k].Width, 
			(*mCubes)[k].Y + (*mCubes)[k].Height, 
			(*mCubes)[k].Z + (*mCubes)[k].Depth);

		for (unsigned int p = 0; p < 6; p++) {
			if (points[p].x() > t_min.x() &&
				points[p].y() > t_min.y() &&
				points[p].z() > t_min.z() &&
				points[p].x() < t_max.x() &&
				points[p].y() < t_max.y() &&
				points[p].z() < t_max.z()) {
					addFace[p] = false;
					inside++;
			}
		}

		if (inside >= 6)
			break;
	}
	*/
	if (inside < 6) {
		lock();

		//add the min/max vertex to the list
		Eigen::Vector3f min = Eigen::Vector3f(
			(*mCubes)[i].X, 
			(*mCubes)[i].Y,
			(*mCubes)[i].Z);
		Eigen::Vector3f max = Eigen::Vector3f(
			(*mCubes)[i].X + (*mCubes)[i].Width, 
			(*mCubes)[i].Y + (*mCubes)[i].Height, 
			(*mCubes)[i].Z + (*mCubes)[i].Depth);

		// Create Vertices
		float3 verts[8] = {
			float3(min.x(), min.y(), max.z()),
			float3(max.x(), min.y(), max.z()),
			float3(max.x(), max.y(), max.z()),
			float3(min.x(), max.y(), max.z()),
			float3(min.x(), min.y(), min.z()),
			float3(max.x(), min.y(), min.z()),
			float3(max.x(), max.y(), min.z()),
			float3(min.x(), max.y(), min.z()),
		};

		// Add Indices
		unsigned int startIndex = mVertices->size();

		// Right
		if (addFace[0]) {
			for (unsigned int index = 0; index < 6; index++) {
				mIndices->push_back(VolumeIndicesRight[index] + startIndex);
			}
		}

		// Left
		if (addFace[1]) {
			for (unsigned int index = 0; index < 6; index++) {
				mIndices->push_back(VolumeIndicesLeft[index] + startIndex);
			}
		}

		// Back
		if (addFace[2]) {
			for (unsigned int index = 0; index < 6; index++) {
				mIndices->push_back(VolumeIndicesBack[index] + startIndex);
			}
		}

		// Front
		if (addFace[3]) {
			for (unsigned int index = 0; index < 6; index++) {
				mIndices->push_back(VolumeIndicesFront[index] + startIndex);
			}
		}

		// Top
		if (addFace[4]) {
			for (unsigned int index = 0; index < 6; index++) {
				mIndices->push_back(VolumeIndicesTop[index] + startIndex);
			}
		}

		// Bottom
		if (addFace[5]) {
			for (unsigned int index = 0; index < 6; index++) {
				mIndices->push_back(VolumeIndicesBottom[index] + startIndex);
			}
		}

		// Add Vertices
		for (unsigned int index = 0; index < 8; index++) {
			mVertices->push_back(VertexPositionColor(verts[index], verts[index]));
		}

		unlock();
	}

	return 0;
}
