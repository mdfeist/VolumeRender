#pragma once

#include "Camera.h"

class Actor {
public:
    virtual ~Actor() {}
	virtual void init() = 0;
	virtual bool needsInit() = 0;
    virtual void render(Camera*) = 0;
};
