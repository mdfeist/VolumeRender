#include "StdAfx.h"
#include "Camera.h"


Camera::Camera(void)
{
}


Camera::~Camera(void)
{
}

void Camera::setFOV(float value) {
	fov = value;
}

float Camera::getFOV() {
	return fov;
}

void Camera::setWidth(int value) {
	width = value;
}

int Camera::getWidth() {
	return width;
}

void Camera::setHeight(int value) {
	height = value;
}

int Camera::getHeight() {
	return height;
}

void Camera::setNearClipping(float value) {
	nearClipping = value;
}

float Camera::getNearClipping() {
	return nearClipping;
}

void Camera::setFarClipping(float value) {
	farClipping = value;
}

float Camera::getFarClipping() {
	return farClipping;
}

void Camera::setPosition(float x, float y, float z) {
	position = Eigen::Vector3f(x, y, z);
}

void Camera::setPosition(Eigen::Vector3f &value) {
	position = value;
}

void Camera::increasePosition(float x, float y, float z) {
	position += Eigen::Vector3f(x, y, z);
}

void Camera::increasePosition(Eigen::Vector3f &value) {
	position += value;
}

Eigen::Vector3f Camera::getPosition() {
	return position;
}

void Camera::setRotation(float w, float x, float y, float z) {
	rotation = Eigen::Quaternionf(w, x, y, z);
}

void Camera::setRotation(Eigen::Quaternionf &value) {
	rotation = value;
}

Eigen::Quaternionf Camera::getRotation() {
	return rotation;
}

Eigen::Vector3f Camera::getForwardVector() {
	Eigen::Vector3f v(0.f, 0.f, -1.f);
    Eigen::Vector3f out = rotation * v;
	out += position;
    
    return out;
}

Eigen::Vector3f Camera::getUpVector() {
	Eigen::Vector3f v(0.f, 1.f, 0.f);
	Eigen::Vector3f out = rotation * v;
	out += position;
    
    return out;
}

Eigen::Vector3f Camera::getRightVector() {
	Eigen::Vector3f v(1.f, 0.f, 0.f);
	Eigen::Vector3f out = rotation * v;
	out += position;
    
    return out;
}

Eigen::Quaternionf Camera::getLookAt(Eigen::Vector3f &lookAt) {
	Eigen::Vector3f diff = lookAt - position;
	float length = diff.norm();
	
	float angleX = -diff.y()/length;
    float angleY = atan2f(diff.z(), diff.x()) + (float)M_PI/2.0f;
    float angleZ = 0.0f;

	Eigen::Quaternionf roation(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));

	return roation;
}