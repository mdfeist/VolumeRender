#pragma once
#include <GL/glew.h>

class VolumeLoader
{
public:
	VolumeLoader(void);
	~VolumeLoader(void);

	int loadRaw(char *directory);
	int loadVolume(char *directory);

	int getPixelCount();
	GLubyte* getData();
	
	float getSpacingX();
	float getSpacingY();
	float getSpacingZ();

	int getVolumeWidth();
	int getVolumeHeight();
	int getVolumeDepth();

private:
	int pixelCount;
	GLubyte *data;
	float spacingX, spacingY, spacingZ;
	int volumeWidth, volumeHeight, volumeDepth;
};

