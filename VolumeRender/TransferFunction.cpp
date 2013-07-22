#include "StdAfx.h"
#include "TransferFunction.h"

#include "Cubic.h"
#include "TransferControlPoint.h"

inline int clamp(int x, int a, int b) {				// Clamp so x is between a and b inclusively
	if (x < a)
		x = a;
	if (x > b)
		x = b;

    return x;
}

TransferFunction::TransferFunction(void) {
	transfer = NULL;
}


TransferFunction::~TransferFunction(void) {
	if (transfer)
		delete[] transfer;
}

GLubyte* TransferFunction::getTransferData() {
	return transfer;
}

void TransferFunction::buildDefault() {
	colorKnots.push_back( new TransferControlPoint(0.f, 0.f, 0.f, 0) );			// Air
	colorKnots.push_back( new TransferControlPoint(0.f, 0.f, 0.f, 63) );		// Air
	colorKnots.push_back( new TransferControlPoint(0.98f, 0.78f, 0.89f, 64) );	// Lung
	colorKnots.push_back( new TransferControlPoint(0.98f, 0.78f, 0.89f, 70) );	// Lung
	colorKnots.push_back( new TransferControlPoint(0.98f, 0.78f, 0.89f, 112) );	// Fat
	colorKnots.push_back( new TransferControlPoint(0.98f, 0.78f, 0.89f, 128) );	// Fat
	colorKnots.push_back( new TransferControlPoint(1.0f, 0.25f, 0.25f, 129) );	// Blood/Muscle
	colorKnots.push_back( new TransferControlPoint(1.0f, 0.25f, 0.25f, 132) );	// Blood/Muscle
	colorKnots.push_back( new TransferControlPoint(0.5f, 0.25f, 0.25f, 133) );	// Liver
	colorKnots.push_back( new TransferControlPoint(0.5f, 0.25f, 0.25f, 136) );	// Liver
	colorKnots.push_back( new TransferControlPoint(0.75f, 0.25f, 0.25f, 140) );	// Soft Tissue
	colorKnots.push_back( new TransferControlPoint(0.75f, 0.25f, 0.25f, 166) ); // Soft Tissue
	colorKnots.push_back( new TransferControlPoint(1.0f, 1.0f, .85f, 167) );	// Bone
	colorKnots.push_back( new TransferControlPoint(1.0f, 1.0f, .85f, 256) );	// Bone

	alphaKnots.push_back( new TransferControlPoint(0.0f, 0) );		// Air
	alphaKnots.push_back( new TransferControlPoint(0.0f, 63) );		// Air
	alphaKnots.push_back( new TransferControlPoint(0.05f, 64) );	// Lung
	alphaKnots.push_back( new TransferControlPoint(0.02f, 70) );	// Lung
	alphaKnots.push_back( new TransferControlPoint(1.0f, 112) );	// Fat
	alphaKnots.push_back( new TransferControlPoint(0.95f, 128) );	// Fat
	alphaKnots.push_back( new TransferControlPoint(0.2f, 129) );	// Blood/Muscle
	alphaKnots.push_back( new TransferControlPoint(0.05f, 132) );	// Blood/Muscle
	alphaKnots.push_back( new TransferControlPoint(0.3f, 133) );	// Liver
	alphaKnots.push_back( new TransferControlPoint(0.1f, 136) );	// Liver
	alphaKnots.push_back( new TransferControlPoint(0.7f, 140) );	// Soft Tissue
	alphaKnots.push_back( new TransferControlPoint(0.35f, 166) );	// Soft Tissue
	alphaKnots.push_back( new TransferControlPoint(0.95f, 210) );	// Bone
	alphaKnots.push_back( new TransferControlPoint(1.f, 256) );		// Bone
}

void TransferFunction::addColorKnot(float r, float g, float b, int isoValue) {
	GLubyte value = (GLubyte)clamp((int)(255.f*(float)(isoValue + 1000)/2000.f), 0, 255.f);
	colorKnots.push_back( new TransferControlPoint(r, g, b, value) );
}

void TransferFunction::addAlphaKnot(float a, int isoValue) {
	GLubyte value = (GLubyte)clamp((int)(255.f*(float)(isoValue + 1000)/2000.f), 0, 255.f);
	alphaKnots.push_back( new TransferControlPoint(a, value) );
}

void TransferFunction::computeTransferFunction() {
	std::sort(colorKnots.begin(), colorKnots.end(), [](TransferControlPoint* a, TransferControlPoint* b){return a->IsoValue < b->IsoValue;});
	std::sort(alphaKnots.begin(), alphaKnots.end(), [](TransferControlPoint* a, TransferControlPoint* b){return a->IsoValue < b->IsoValue;});

	colorKnots.erase( 
		std::unique( colorKnots.begin(), colorKnots.end(), [](TransferControlPoint* a, TransferControlPoint* b){return a->IsoValue == b->IsoValue;} ), 
		colorKnots.end() );

	alphaKnots.erase( 
		std::unique( alphaKnots.begin(), alphaKnots.end(), [](TransferControlPoint* a, TransferControlPoint* b){return a->IsoValue == b->IsoValue;} ), 
		alphaKnots.end() );

	// Check if valid transfer function data
	// If not clear old data and build default transfer function
	if (colorKnots.size() < 2 || alphaKnots.size() < 2) {
		colorKnots.clear();
		alphaKnots.clear();
		buildDefault();
	} else {
		if (colorKnots[0]->IsoValue != 0) {
			std::vector<TransferControlPoint*>::iterator it;
			it = colorKnots.begin();
			it = colorKnots.insert ( it , 
				new TransferControlPoint(
					colorKnots[0]->Color.x(), 
					colorKnots[0]->Color.y(), 
					colorKnots[0]->Color.z(), 
					0 ));
		}

		if (colorKnots[colorKnots.size() - 1]->IsoValue != 256) {
			colorKnots.push_back ( new TransferControlPoint(
				colorKnots[colorKnots.size() - 1]->Color.x(), 
				colorKnots[colorKnots.size() - 1]->Color.y(), 
				colorKnots[colorKnots.size() - 1]->Color.z(), 
				256 ));
		}

		if (alphaKnots[0]->IsoValue != 0) {
			std::vector<TransferControlPoint*>::iterator it;
			it = alphaKnots.begin();
			it = alphaKnots.insert ( it , 
				new TransferControlPoint(
					alphaKnots[0]->Color.w(),
					0 ));
		}

		if (alphaKnots[alphaKnots.size() - 1]->IsoValue != 256) {
			alphaKnots.push_back ( new TransferControlPoint(
				alphaKnots[alphaKnots.size() - 1]->Color.w(),
				256 ));
		}
	}

	//initialize the cubic spline for the transfer function
	Eigen::Vector4f* transferFunc = new Eigen::Vector4f[256];

	//temporary transfer function copy the color/alpha from the transfer control points
	std::vector<TransferControlPoint*> tempColorKnots = std::vector<TransferControlPoint*>(colorKnots);
	std::vector<TransferControlPoint*> tempAlphaKnots = std::vector<TransferControlPoint*>(alphaKnots);

	int colorN = colorKnots.size() - 1;
	int alphaN = tempAlphaKnots.size() - 1;

	float* red = new float[colorN + 1];
	float* green = new float[colorN + 1];
	float* blue = new float[colorN + 1];
	float* alpha = new float[alphaN + 1];

	for (int i = 0; i < colorN + 1; i++)
	{
		red[i] = tempColorKnots[i]->Color.x();
		green[i] = tempColorKnots[i]->Color.y();
		blue[i] = tempColorKnots[i]->Color.z();
	}

	for (int i = 0; i < alphaN + 1; i++)
	{
		alpha[i] = tempAlphaKnots[i]->Color.w();
	}


	Cubic* redCubic = Cubic::calcNaturalCubic(colorN, red);
	Cubic* greenCubic = Cubic::calcNaturalCubic(colorN, green);
	Cubic* blueCubic = Cubic::calcNaturalCubic(colorN, blue);
	Cubic* alphaCubic = Cubic::calcNaturalCubic(alphaN, alpha);

	int numTF = 0;
	for (unsigned int i = 0; i < colorKnots.size() - 1; i++)
	{
		int steps = colorKnots[i + 1]->IsoValue - colorKnots[i]->IsoValue;

		for (int j = 0; j < steps; j++)
		{
			float k = (float)j / (float)(steps - 1);

			transferFunc[numTF].x() = redCubic[i].GetPointOnSpline(k);
			transferFunc[numTF].y() = greenCubic[i].GetPointOnSpline(k);
			transferFunc[numTF].z() = blueCubic[i].GetPointOnSpline(k);
			numTF++;
		}
	}

	numTF = 0;
	for (unsigned int i = 0; i < alphaKnots.size() - 1; i++)
	{
		int steps = alphaKnots[i + 1]->IsoValue - alphaKnots[i]->IsoValue;

		for (int j = 0; j < steps; j++)
		{
			float k = (float)j / (float)(steps - 1);

			transferFunc[numTF++].w() = alphaCubic[i].GetPointOnSpline(k);
		}
	}

	transfer = (GLubyte*)malloc(4 * 256 * sizeof(GLubyte));
	for (int i = 0; i < 256; i++)
	{
		//store rgba
		transfer[4*i + 0] = (GLubyte)clamp((int)(255.f*transferFunc[i].x()), 0, 255);
		transfer[4*i + 1] = (GLubyte)clamp((int)(255.f*transferFunc[i].y()), 0, 255);
		transfer[4*i + 2] = (GLubyte)clamp((int)(255.f*transferFunc[i].z()), 0, 255);
		transfer[4*i + 3] = (GLubyte)clamp((int)(255.f*transferFunc[i].w()), 0, 255);
	}
}
