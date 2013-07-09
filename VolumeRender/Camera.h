#pragma once

#include "Eigen\Core.h"
#include "Eigen\src\Geometry\RotationBase.h"
#include "Eigen\src\Geometry\Quaternion.h"

class Camera
{
public:
	Camera(void);
	~Camera(void);

	void setFOV(float value);
	float getFOV();

	void setWidth(int value);
	int getWidth();

	void setHeight(int value);
	int getHeight();

	void setNearClipping(float value);
	float getNearClipping();

	void setFarClipping(float value);
	float getFarClipping();

private:
	float fov;								// Field of View

	int width;								// Screen Width
	int height;								// Screen Height

	float nearClipping;						// Near clipping perspective
	float farClipping;						// Far clipping perspective

	Eigen::Vector3f position;				// Camera Position
	Eigen::Quaternionf rotation;			// Camera Rotation
};

