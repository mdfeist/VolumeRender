#pragma once

#include "Camera.h"
#include "Eigen\Geometry.h"

class Actor {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Actor() {}
    virtual ~Actor() {}
	virtual void init() = 0;
	virtual bool needsInit() = 0;
    virtual void render(Camera*) = 0;

	void setScale(float x, float y, float z) { 
		setScale(Eigen::Vector3f( x, y, z ));
	}

	void setScale(Eigen::Vector3f &value) {
		scale = value;
	}

	void increaseScale(float x, float y, float z) {
		increaseScale(Eigen::Vector3f( x, y, z ));
	}

	void increaseScale(Eigen::Vector3f &value) {
		scale += value;
	}

	Eigen::Vector3f getScale() {
		return scale;
	}

	void setPosition(float x, float y, float z) { 
		setPosition(Eigen::Vector3f( x, y, z ));
	}

	void setPosition(Eigen::Vector3f &value) {
		position = value;
	}

	void increasePosition(float x, float y, float z) {
		increasePosition(Eigen::Vector3f( x, y, z ));
	}

	void increasePosition(Eigen::Vector3f &value) {
		position += value;
	}

	Eigen::Vector3f getPosition() {
		return position;
	}

	void setRotation(float w, float x, float y, float z) {
		setRotation(Eigen::Quaternionf( w, x, y, z ));
	}

	void setRotation(Eigen::Quaternionf &value) {
		rotation = value;
	}

	Eigen::Quaternionf getRotation() {
		return rotation;
	}

protected:
	Eigen::Vector3f scale;					// Actor scale
	Eigen::Vector3f position;				// Actor Position
	Eigen::Quaternionf rotation;			// Actor Rotation
};
