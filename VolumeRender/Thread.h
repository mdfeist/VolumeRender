#pragma once
#include "Runnable.h"

class Thread : public Runnable {
public:
    void start() {
		//_beginthreadex( NULL, 0, &Runnable::createThread, this, 0, NULL);
        threadHandle = CreateThread(NULL, 0, &Runnable::createThread, this, 0, NULL);
    }

	bool isAlive() {
		DWORD result = WaitForSingleObject( threadHandle, 0);

		if (result != WAIT_OBJECT_0)
			return true;

		return false;
	}

	HANDLE getHandle() { return threadHandle; }

protected:
    virtual DWORD runThread() = 0;
	HANDLE threadHandle;
};
