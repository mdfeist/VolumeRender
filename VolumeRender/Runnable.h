#pragma once
#include <Windows.h>

class Runnable {
public:
    virtual ~Runnable() {}
    static DWORD WINAPI createThread(LPVOID args) {
        Runnable *prunnable = static_cast<Runnable*>(args);
		DWORD ret = prunnable->runThread();
        return ret;
    }
 protected:
    virtual DWORD runThread() = 0;
};
