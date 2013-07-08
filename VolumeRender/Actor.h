#pragma once

class Actor {
public:
    virtual ~Actor() {}
	virtual void init() = 0;
	virtual bool needsInit() = 0;
	virtual void reset() = 0;
    virtual void render(int w, int h) = 0;
};
