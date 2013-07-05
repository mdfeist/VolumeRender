// VolumeRender.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>

#include "Renderer.h"
#include "Volume.h"

int main(int argc, char* argv[])
{
	Volume* volume = new Volume();

	Renderer* render1 = new Renderer();
	render1->setClearColor(0.0f, 0.0f, 0.0f, 0.5f);
	render1->addActor(volume);

	while(true) {
		Sleep(1000);
	}

	return 0;
}

