#pragma once
#include <Windows.h>

class Runnable {
public:
    virtual ~Runnable() {}
    static DWORD WINAPI createThread(LPVOID args) {
        Runnable *prunnable = static_cast<Runnable*>(args);
        return prunnable->runThread();
    }
 protected:
    virtual DWORD runThread() = 0;
};
