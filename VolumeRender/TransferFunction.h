#pragma once
#include <GL/glew.h>

#include <vector>

class TransferControlPoint;

class TransferFunction
{
public:
	TransferFunction(void);
	~TransferFunction(void);
	
	void buildDefault();
	void addColorKnot(float r, float g, float b, int isoValue);
	void addAlphaKnot(float a, int isoValue);

	void computeTransferFunction();

	GLubyte* getTransferData();
private:
	GLubyte* transfer;

	std::vector<TransferControlPoint*> colorKnots;
	std::vector<TransferControlPoint*> alphaKnots;
};

