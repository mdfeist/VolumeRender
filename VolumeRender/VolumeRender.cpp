// VolumeRender.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>

#include "Renderer.h"
#include "Volume.h"

#include "Eigen\Geometry.h"

int main(int argc, char* argv[])
{
	Volume* volume = new Volume();
	//volume->setPosition(2.0, 0.0, 0.0);
	
	char volumeFile[] = "C:/Users/mdfeist/Desktop/COU IV";
	volume->loadVolume(volumeFile);

	Renderer* render1 = new Renderer();
	render1->setClearColor(0.0f, 0.0f, 0.0f, 0.5f);
	render1->addActor(volume);

	float angle = 0.0f;

	//for(int i = 0; i < 10; i++) {
	while(true) {	
		//Eigen::Quaternionf rotation(Eigen::AngleAxisf(angle, Eigen::Vector3f(1.0f, 0.0, 0.0)));
		//volume->setRotation(rotation);

		//angle += 0.1f;
		Sleep(1000);
	}

	return 0;
}

