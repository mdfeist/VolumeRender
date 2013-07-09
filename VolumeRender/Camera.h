#pragma once

#include "Eigen\Core.h"
#include "Eigen\Geometry.h"

class Camera
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

	void setPosition(float x, float y, float z);
	void setPosition(Eigen::Vector3f &value);

	void increasePosition(float x, float y, float z);
	void increasePosition(Eigen::Vector3f &value);

	Eigen::Vector3f getPosition();

	void setRotation(float w, float x, float y, float z);
	void setRotation(Eigen::Quaternionf &value);

	Eigen::Quaternionf getRotation();

	Eigen::Vector3f getForwardVector();
	Eigen::Vector3f getUpVector();
	Eigen::Vector3f getRightVector();

	Eigen::Quaternionf getLookAt(float x, float y, float z);
	Eigen::Quaternionf getLookAt(Eigen::Vector3f &lookAt);

	void lookAt(float x, float y, float z);
	void lookAt(Eigen::Vector3f &lookAt);

	void follow(Eigen::Vector3f goTo, float distance);

private:
	float fov;								// Field of View

	int width;								// Screen Width
	int height;								// Screen Height

	float nearClipping;						// Near clipping perspective
	float farClipping;						// Far clipping perspective

	Eigen::Vector3f position;				// Camera Position
	Eigen::Quaternionf rotation;			// Camera Rotation
};

