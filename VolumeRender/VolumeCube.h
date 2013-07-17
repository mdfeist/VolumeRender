#pragma once

#include "Eigen\Core.h"

class VolumeCube {
public:
	float X, Y, Z;
    float Width, Height, Depth;

    float Top() { return X; }
    float Left() { return Y; }
    float Front() { return Z; }

	VolumeCube(Eigen::Vector3f& pos, Eigen::Vector3f& dims)
	{
		X = pos.x();
		Y = pos.y();
		Z = pos.z();

		Width = dims.x();
		Height = dims.y();
		Depth = dims.z();
	}

	VolumeCube(float x, float y, float z, float w, float h, float d)
	{
		X = x;
		Y = y;
		Z = z;
		
		Width = w;
		Height = h;
		Depth = d;
	}
};
