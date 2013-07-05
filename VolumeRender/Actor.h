#pragma once

class Actor {
public:
    virtual ~Actor() {}
    virtual void render() = 0;
};
