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