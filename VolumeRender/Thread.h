#pragma once
#include "Runnable.h"

class Thread : public Runnable {
public:
    void start() {
        CreateThread(NULL, 0, &Runnable::createThread, this, 0, NULL);
    }
protected:
    virtual DWORD runThread() = 0;
};
