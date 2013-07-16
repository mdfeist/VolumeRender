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
	Eigen::Quaternionf rotation(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f(1.0, 0.0, 0.0)));
	volume->setRotation(rotation);
	
	//char volumeFile[] = "C:/Users/mdfeist/Desktop/COU IV";
	//char volumeFile[] = "C:/Users/mdfeist/Desktop/MANIX/MANIX/MANIX/CER-CT/ANGIO CT";
	char volumeFile[] = "C:/Users/mdfeist/Desktop/ARTIFIX/ARTIFIX/Thorax 1CTA_THORACIC_AORTA_GATED (Adult)/A Aorta w-c  1.5  B20f  60%";
	//char volumeFile[] = "C:/Users/mdfeist/Desktop/Echo_V1.bin";
	//char volumeFile[] = "C:/Users/mdfeist/Desktop/T01";
	volume->loadVolume(volumeFile);
	//volume->loadRaw(volumeFile);

	Renderer* render1 = new Renderer();
	render1->getActiveCamera()->setFOV(45.f);
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

