#include "StdAfx.h"
#include "Volume.h"

#include <Windows.h>
#include <math.h>
#include <stddef.h>

#include "itkImage.h"
#include "itkImageSeriesReader.h"
#include "itkGDCMImageIO.h"
#include "itkRGBAPixel.h"
#include "itkGDCMSeriesFileNames.h"

#include "TransferControlPoint.h"

// All textures for the buffer are TEXTURE_SIZExTEXTURE_SIZE in dimensions
#define TEXTURE_SIZE 1024
// Used to find the offset of variables in a structure (found when binding the VBO)
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))
// The size of the volume test data
#define VOLUME_TEX_SIZE 128

struct float3 {										// A three dimensional float
	float x, y, z;									// Variables for each dimension
	
	float3() {										// Generic constructor
		x = 0.f;
		y = 0.f;
		z = 0.f;
	}

	float3(float _x, float _y, float _z) {			// Constructor with given values
		x = _x;
		y = _y;
		z = _z;
	}

	float3 operator + (const float3 &b) const {		// Addition operator
		return float3(x + b.x, y + b.y, z + b.z);
	}

	float3 operator - (const float3 &b) const {		// Subtraction operator
		return float3(x - b.x, y - b.y, z - b.z);
	}
};

struct Vertex										// Vertex used for creating VBO
{
    float3 m_Pos;									// Position of vertex
    float3 m_Color;									// Color of vertex
	float3 m_Normal;								// Normal of vertex

	Vertex() {}										// Generic constructor

	Vertex(float3 pos, float3 norm) {				// Constructor with given values
		m_Pos = pos;								// Set the position
		m_Color = pos;								// Set color to be the same as position
		m_Normal = norm;							// Set normal
	}
};

float length(float3 p) {							// Get the length of a float3
	return sqrtf(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline int clamp(int x, int a, int b) {
    return x < a ? a : (x > b ? b : x);
}

/// <summary>
/// Cubic class that calculates the cubic spline from a set of control points/knots
/// and performs cubic interpolation.
/// 
/// Based on the natural cubic spline code from: http://www.cse.unsw.edu.au/~lambert/splines/natcubic.html
/// </summary>
class Cubic
{
private:
	Eigen::Vector4f a, b, c, d; // a + b*s + c*s^2 +d*s^3 

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Cubic() {}
	Cubic(Eigen::Vector4f& a, Eigen::Vector4f& b, Eigen::Vector4f& c, Eigen::Vector4f& d)
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}

	//evaluate the point using a cubic equation
	Eigen::Vector4f GetPointOnSpline(float s)
	{
		return (((d * s) + c) * s + b) * s + a;
	}

	static std::vector<Cubic*> CalculateCubicSpline(int n, std::vector<TransferControlPoint*> v)
	{
		Eigen::Vector4f* gamma = new Eigen::Vector4f[n + 1];
		Eigen::Vector4f* delta = new Eigen::Vector4f[n + 1];
		Eigen::Vector4f* D = new Eigen::Vector4f[n + 1];
		int i;
		/* We need to solve the equation
		* taken from: http://mathworld.wolfram.com/CubicSpline.html
		[2 1       ] [D[0]]   [3(v[1] - v[0])  ]
		|1 4 1     | |D[1]|   |3(v[2] - v[0])  |
		|  1 4 1   | | .  | = |      .         |
		|    ..... | | .  |   |      .         |
		|     1 4 1| | .  |   |3(v[n] - v[n-2])|
		[       1 2] [D[n]]   [3(v[n] - v[n-1])]

		by converting the matrix to upper triangular.
		The D[i] are the derivatives at the control points.
		*/

		//this builds the coefficients of the left matrix
		gamma[0] = Eigen::Vector4f::Zero();
		gamma[0].x() = 1.0f / 2.0f;
		gamma[0].y() = 1.0f / 2.0f;
		gamma[0].z() = 1.0f / 2.0f;
		gamma[0].w() = 1.0f / 2.0f;
		for (i = 1; i < n; i++)
		{
			Eigen::Vector4f v = ((4.f * Eigen::Vector4f::Ones()) - gamma[i - 1]);
			gamma[i].x() = 1.0f / v.x();
			gamma[i].y() = 1.0f / v.y();
			gamma[i].z() = 1.0f / v.z();
			gamma[i].w() = 1.0f / v.w();
		}

		{
			Eigen::Vector4f v = ((2.f * Eigen::Vector4f::Ones()) - gamma[n - 1]);
			gamma[n].x() = 1.0f / v.x();
			gamma[n].y() = 1.0f / v.y();
			gamma[n].z() = 1.0f / v.z();
			gamma[n].w() = 1.0f / v.w();
		}

		delta[0] = 3.f * (v[1]->Color - v[0]->Color);
		
		delta[0].x() *= gamma[0].x();
		delta[0].y() *= gamma[0].y();
		delta[0].z() *= gamma[0].z();
		delta[0].w() *= gamma[0].w();

		for (i = 1; i < n; i++)
		{
			delta[i] = (3.f * (v[i + 1]->Color - v[i - 1]->Color) - delta[i - 1]);

			delta[i].x() *= gamma[i].x();
			delta[i].y() *= gamma[i].y();
			delta[i].z() *= gamma[i].z();
			delta[i].w() *= gamma[i].w();
		}

		delta[n] = (3.f * (v[n]->Color - v[n - 1]->Color) - delta[n - 1]);
		
		delta[n].x() *= gamma[n].x();
		delta[n].y() *= gamma[n].y();
		delta[n].z() *= gamma[n].z();
		delta[n].w() *= gamma[n].w();

		D[n] = delta[n];
		for (i = n - 1; i >= 0; i--)
		{
			D[i] = delta[i] - gamma[i];

			D[i].x() *= D[i + 1].x();
			D[i].y() *= D[i + 1].y();
			D[i].z() *= D[i + 1].z();
			D[i].w() *= D[i + 1].w();
		}

		// now compute the coefficients of the cubics 
		std::vector<Cubic*> C(n);
		for (i = 0; i < n; i++)
		{
			Eigen::Vector4f a = v[i]->Color;
			Eigen::Vector4f b = D[i];
			Eigen::Vector4f c = 3.f * (v[i + 1]->Color - v[i]->Color) - 2.f * D[i] - D[i + 1];
			Eigen::Vector4f d = 2.f * (v[i]->Color - v[i + 1]->Color) + D[i] + D[i + 1];
			C[i] = new Cubic(a, b, c, d);
		}
		return C;
	}
};


// create a test volume texture, here you could load your own volume
GLuint create_volumetexture()
{
	int size = VOLUME_TEX_SIZE*VOLUME_TEX_SIZE*VOLUME_TEX_SIZE* 4;
	GLubyte *data = new GLubyte[size];

	for(int x = 0; x < VOLUME_TEX_SIZE; x++)
	{for(int y = 0; y < VOLUME_TEX_SIZE; y++)
	{for(int z = 0; z < VOLUME_TEX_SIZE; z++)
	{
		data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = z%250;
		data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%250;
		data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 230;
	  	
		float3 p =	float3(x,y,z)- float3(VOLUME_TEX_SIZE-20,VOLUME_TEX_SIZE-30,VOLUME_TEX_SIZE-30);
		bool test = (length(p) < 42);
		if(test)
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 0;

		p =	float3(x,y,z)- float3(VOLUME_TEX_SIZE/2,VOLUME_TEX_SIZE/2,VOLUME_TEX_SIZE/2);
		test = (length(p) < 24);
		if(test)
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 0;

		
		if(x > 20 && x < 40 && y > 0 && y < VOLUME_TEX_SIZE && z > 10 &&  z < 50)
		{
			
			data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 100;
		    data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%100;
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		}

		if(x > 50 && x < 70 && y > 0 && y < VOLUME_TEX_SIZE && z > 10 &&  z < 50)
		{
			
			data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%100;
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		}

		if(x > 80 && x < 100 && y > 0 && y < VOLUME_TEX_SIZE && z > 10 &&  z < 50)
		{
			
			data[(x*4)   + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		    data[(x*4)+1 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 70;
		    data[(x*4)+2 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = y%100;
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 250;
		}

		p =	float3(x,y,z)- float3(24,24,24);
		test = (length(p) < 40);
		if(test)
			data[(x*4)+3 + (y * VOLUME_TEX_SIZE * 4) + (z * VOLUME_TEX_SIZE * VOLUME_TEX_SIZE * 4)] = 0;

			
	}}}

	GLuint volume_texture;
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glGenTextures(1, &volume_texture);
	glBindTexture(GL_TEXTURE_3D, volume_texture);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, VOLUME_TEX_SIZE, VOLUME_TEX_SIZE, VOLUME_TEX_SIZE, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

	delete []data;
	printf("volume texture created\n");

	return volume_texture;
}

//Check for any GL Errors
void errcheck() {
	static GLenum errCode;
	const GLubyte *errString;

	if ((errCode = glGetError()) != GL_NO_ERROR) {
		errString = gluErrorString(errCode);
		fprintf(stderr, "OpenGL Error: %s\n", errString);
	}
}

//Check for any Cg Errors
void CheckCgError(void) {
	CGerror err = cgGetError();
	if (err != CG_NO_ERROR) {
		fprintf(stderr, "CG error: %s\n", cgGetErrorString(err));
	}
}

// Create a new 2D texture
GLuint newTexture(int width, int height) {
	GLuint texture;
	glGenTextures(1, &texture);												// Generate texture
	glBindTexture(GL_TEXTURE_2D, texture);									// Bind texture
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);				// Use the texture color when rendering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);		// Set the MAG_FILTER to interpolate linearly between the closest pixels
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);		// Set the MIN_FILTER to interpolate linearly between the closest pixels
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);	// Clamp the X value to boarder when doing texture look ups
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);	// Clamp the Y value to boarder when doing texture look ups
	glTexImage2D(															// Create texture
		GL_TEXTURE_2D,														// Texture is 2D
		0,																	// Use base level of detail
		GL_RGBA16F_ARB,														// Use internal format GL_RGBA16F_ARB
		width,																// Set the texture width
		height,																// Set the texture height
		0,																	// Set the boarder to zero (Needs to be 0)
		GL_RGBA,															// Use format GL_RGBA
		GL_FLOAT,															// Use type GL_FLOAT
		NULL);																// Set data to NULL
	return texture;															// Return texture
}

Volume::Volume(void) : Actor()												// Constructor
{
	cubeVerticesVBO = 0;													// Set cubeVerticesVBO to NULL
	initialized = false;													// Set initialized to false

	position = Eigen::Vector3f(0, 0, 0);
	rotation = Eigen::Quaternionf::Identity();
}


Volume::~Volume(void)														// Destructor
{
	if ( cubeVerticesVBO != 0 )												// Check if cubeVerticesVBO is not NULL
	{
		glDeleteBuffersARB( 1, &cubeVerticesVBO );							// Delete cubeVerticesVBO from GPU buffer
		cubeVerticesVBO = 0;												// Set cubeVerticesVBO to NULL
	}
}

void Volume::init() {
	FBO = setupFBO();
	printf("FBO created\n");

	front_facing = newTexture(TEXTURE_SIZE, TEXTURE_SIZE);
	back_facing = newTexture(TEXTURE_SIZE, TEXTURE_SIZE);
	printf("Textures created\n");

	char shaderFile[] = "shader/raycastDiffuse.cg";
	if (setupCg(&context, &fProgram, &fragmentProfile, shaderFile)) {
		fprintf(stderr, "Error: %s\n", "Initializing Cg");
		CheckCgError();
	}

	cgFrontTexData = cgGetNamedParameter(fProgram, "frontTexData");
	cgBackTexData = cgGetNamedParameter(fProgram, "backTexData");
	cgVolumeTexData = cgGetNamedParameter(fProgram, "VolumeS");
	cgTransferTexData = cgGetNamedParameter(fProgram, "TransferS");
	cgStepSize = cgGetNamedParameter(fProgram, "stepSize");

	cgGLSetParameter1f(cgStepSize, 1.0f/100.0f);				// Set the incremental step size of the ray cast

	createCube(1.0f, 1.0f, 1.0f);
	printf("Cube created\n");

	volume_texture = createVolume();
	printf("volume texture created\n");
	//volume_texture = create_volumetexture();

	GLuint transferTexture;
	glGenTextures(1, &transferTexture);
	glBindTexture(GL_TEXTURE_1D, transferTexture);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, transfer);

	initialized = true;
}

bool Volume::needsInit() {
	return !initialized;
}

int Volume::loadVolume(char *directory) {
	typedef signed short    PixelType;
	const unsigned int      Dimension = 3;

	typedef itk::Image< PixelType, Dimension >         ImageType;

	typedef itk::ImageSeriesReader<ImageType> ReaderType; 
	ReaderType::Pointer reader = ReaderType::New();
	
	typedef itk::GDCMImageIO       ImageIOType;
	ImageIOType::Pointer dicomIO = ImageIOType::New();

	reader->SetImageIO( dicomIO );

	typedef itk::GDCMSeriesFileNames NamesGeneratorType;
	NamesGeneratorType::Pointer nameGenerator = NamesGeneratorType::New();


	nameGenerator->SetUseSeriesDetails( true );
	nameGenerator->AddSeriesRestriction("0008|0021" );

	nameGenerator->SetDirectory( directory );
	
	try
	{
		std::cout << std::endl << "The directory: " << directory << std::endl;
		std::cout << "Contains the following DICOM Series: ";
		std::cout << std::endl << std::endl;

		typedef std::vector< std::string >    SeriesIdContainer;
		const SeriesIdContainer & seriesUID = nameGenerator->GetSeriesUIDs();
		SeriesIdContainer::const_iterator seriesItr = seriesUID.begin();
		SeriesIdContainer::const_iterator seriesEnd = seriesUID.end();
		while( seriesItr != seriesEnd )
		{
			std::cout << seriesItr->c_str() << std::endl;
			++seriesItr;
		}


		std::string seriesIdentifier;

		//single series in folder
		seriesIdentifier = seriesUID.begin()->c_str();

		std::cout << std::endl << std::endl;
		std::cout << "Now reading series: ";
		std::cout << seriesIdentifier;
		std::cout << std::endl << std::endl;


		typedef std::vector< std::string >   FileNamesContainer;
		FileNamesContainer fileNames;
		fileNames = nameGenerator->GetFileNames( seriesIdentifier );

		// File names to Read
		reader->SetFileNames( fileNames );
		std::cout << "- Reading files ... ";

		try
		{
			reader->UpdateLargestPossibleRegion();
			std::cout << "Successfully read " << fileNames.size() << " file(s)." << std::endl;
		}
		catch (itk::ExceptionObject &ex)
		{
			std::cout << "Failed."<< std::endl;
			std::cout <<"*********************************************************************" <<std::endl;
			std::cout << ex << std::endl;
			std::cout << "*********************************************************************" <<std::endl;
			return EXIT_FAILURE;
		}

		//Test Function: Get Dimention values
		typedef itk::Image< PixelType, 3 >   ImageType;
		ImageType::Pointer image = reader->GetOutput();

		ImageType::RegionType region = image->GetLargestPossibleRegion();
		ImageType::SizeType size = region.GetSize();

		// Pointer to the start of the Image
		signed short * bufferPointer = reader->GetOutput()->GetBufferPointer();

		// Get number of pixels in image
		pixelCount = size[0] * size[1] *size[2];

		volumeWidth = size[0];
		volumeHeight = size[1];
		volumeDepth = size[2];

		std::cout << "- Volume Size: " 
			<< "[" 
			<< volumeWidth << ", "
			<< volumeHeight << ", "
			<< volumeDepth
			<< "]" << std::endl;


		// Create array for iso values
		data = new GLubyte[pixelCount * 4];
		mSamples = new float[pixelCount];
		memset(data, 0, pixelCount * 4);

		std::cout << "- Normalizing data" << std::endl; 

		float min = -500.f;
		float max = 700.f;

		// Normalize values
		for (int i = 0; i < pixelCount; i = i++)
		{
			float normal = ((float)bufferPointer[i] - min)/(max - min);

			if (normal < 0.f)
				normal = 0.f;
			else if (normal > 1.f)
				normal = 1.f;

			mSamples[i] = normal;
		}
	}
	catch (itk::ExceptionObject &ex)
    {
		std::cout << ex << std::endl;
		return EXIT_FAILURE;
    }

	std::cout << "- Computing Transfer Function" << std::endl; 
	computeTransferFunction();
	
	std::cout << "- Computing Gradients" << std::endl; 
	//generate normals from gradients
	std::cout << "- Size: " << pixelCount << std::endl; 
	mGradients = (float*)malloc(3 * pixelCount * sizeof(float));

	if (mGradients) {
		std::cout << "- Generate Gradients" << std::endl; 
		generateGradients(0);

		std::cout << "- Computing Filter" << std::endl; 
		//filter the gradients with an NxNxN box filter
		filterNxNxN(3);

		for (int i = 0; i < pixelCount; i++)
		{
			data[4*i + 0] = mGradients[3*i + 0];
			data[4*i + 1] = mGradients[3*i + 1];
			data[4*i + 2] = mGradients[3*i + 2];
			data[4*i + 3] = 255*mSamples[i];
		}

		free(mGradients);
	} else {
		for (int i = 0; i < pixelCount; i++)
		{
			data[4*i + 0] = 255;
			data[4*i + 1] = 255;
			data[4*i + 2] = 255;
			data[4*i + 3] = 255*mSamples[i];
		}
	}

	delete mSamples;
	
	return EXIT_SUCCESS;

}

GLuint Volume::createVolume() {
	GLuint volume_texture;
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glGenTextures(1, &volume_texture);
	glBindTexture(GL_TEXTURE_3D, volume_texture);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, 
		volumeWidth, volumeHeight, volumeDepth, 
		0, GL_RGBA, GL_UNSIGNED_BYTE, data);

	delete []data;

	return volume_texture;
}

void Volume::computeTransferFunction() {
	// Test spline
	colorKnots.push_back( new TransferControlPoint(.91f, .7f, .61f, 0) );
	colorKnots.push_back( new TransferControlPoint(.91f, .7f, .61f, 80) );
	colorKnots.push_back( new TransferControlPoint(1.0f, 1.0f, .85f, 82) );
	colorKnots.push_back( new TransferControlPoint(1.0f, 1.0f, .85f, 256) );

	alphaKnots.push_back( new TransferControlPoint(0.0f, 0) );
	alphaKnots.push_back( new TransferControlPoint(0.0f, 40) );
	alphaKnots.push_back( new TransferControlPoint(0.2f, 60) );
	alphaKnots.push_back( new TransferControlPoint(0.05f, 63) );
	alphaKnots.push_back( new TransferControlPoint(0.0f, 80) );
	alphaKnots.push_back( new TransferControlPoint(0.9f, 82) );
	alphaKnots.push_back( new TransferControlPoint(1.f, 256) );

	//initialize the cubic spline for the transfer function
	Eigen::Vector4f* transferFunc = new Eigen::Vector4f[256];

	//temporary transfer function copy the color/alpha from the transfer control points
	std::vector<TransferControlPoint*> tempColorKnots = std::vector<TransferControlPoint*>(colorKnots);
	std::vector<TransferControlPoint*> tempAlphaKnots = std::vector<TransferControlPoint*>(alphaKnots);

	std::vector<Cubic*> colorCubic = Cubic::CalculateCubicSpline(colorKnots.size() - 1, tempColorKnots);
	std::vector<Cubic*> alphaCubic = Cubic::CalculateCubicSpline(alphaKnots.size() - 1, tempAlphaKnots);

	int numTF = 0;
	for (int i = 0; i < colorKnots.size() - 1; i++)
	{
		int steps = colorKnots[i + 1]->IsoValue - colorKnots[i]->IsoValue;

		for (int j = 0; j < steps; j++)
		{
			float k = (float)j / (float)(steps - 1);

			transferFunc[numTF++] = colorCubic[i]->GetPointOnSpline(k);
		}
	}

	numTF = 0;
	for (int i = 0; i < alphaKnots.size() - 1; i++)
	{
		int steps = alphaKnots[i + 1]->IsoValue - alphaKnots[i]->IsoValue;

		for (int j = 0; j < steps; j++)
		{
			float k = (float)j / (float)(steps - 1);

			transferFunc[numTF++].w() = alphaCubic[i]->GetPointOnSpline(k).w();
		}
	}

	transfer = new GLubyte[4 * 256];
	for (int i = 0; i < 256; i++)
	{
		Eigen::Vector4f color = transferFunc[i];
		//store rgba
		transfer[4*i + 0] = 255*color.x();
		transfer[4*i + 1] = 255*color.y();
		transfer[4*i + 2] = 255*color.z();
		transfer[4*i + 3] = 255*color.w();
	}
}

/// <summary>
/// Generates gradients using a central differences scheme.
/// </summary>
/// <param name="sampleSize">The size/radius of the sample to take.</param>
void Volume::generateGradients(int sampleSize)
{
	int n = sampleSize;

	int index = 0;
	for (int z = 0; z < volumeDepth; z++)
	{
		for (int y = 0; y < volumeHeight; y++)
		{
			for (int x = 0; x < volumeWidth; x++)
			{
				float xDiff = sampleVolume(x + n, y, z) - sampleVolume(x - n, y, z);
				float yDiff = sampleVolume(x, y + n, z) - sampleVolume(x, y - n, z);
				float zDiff = sampleVolume(x, y, z + n) - sampleVolume(x, y, z - n);

				float size_squared = xDiff*xDiff + yDiff*yDiff + zDiff*zDiff;
				
				if (size_squared == 0.f) {
					mGradients[3*index + 0] = 0.f;
					mGradients[3*index + 1] = 0.f;
					mGradients[3*index + 2] = 0.f;
				} else {
					float size = sqrtf(size_squared);
					xDiff /= size;
					yDiff /= size;
					zDiff /= size;
					mGradients[3*index + 0] = xDiff;
					mGradients[3*index + 1] = yDiff;
					mGradients[3*index + 2] = zDiff;
				}
				
				index++;
			}
		}
	}
}

/// <summary>
/// Applies an NxNxN filter to the gradients. 
/// Should be an odd number of samples. 3 used by default.
/// </summary>
/// <param name="n"></param>
void Volume::filterNxNxN(int n)
{
	int index = 0;
	for (int z = 0; z < volumeDepth; z++)
	{
		for (int y = 0; y < volumeHeight; y++)
		{
			for (int x = 0; x < volumeWidth; x++)
			{
				Eigen::Vector3f sample = sampleNxNxN(x, y, z, n);
				
				mGradients[3*index + 0] = sample.x();
				mGradients[3*index + 1] = sample.y();
				mGradients[3*index + 2] = sample.z();

				index++;
			}
		}
	}
}

/// <summary>
/// Samples the sub-volume graident volume and returns the average.
/// Should be an odd number of samples.
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="z"></param>
/// <param name="n"></param>
/// <returns></returns>
Eigen::Vector3f Volume::sampleNxNxN(int x, int y, int z, int n)
{
	n = (n - 1) / 2;

	Eigen::Vector3f average = Eigen::Vector3f::Zero();
	int num = 0;

	for (int k = z - n; k <= z + n; k++)
	{
		for (int j = y - n; j <= y + n; j++)
		{
			for (int i = x - n; i <= x + n; i++)
			{
				if (isInBounds(i, j, k))
				{
					average += sampleGradients(i, j, k);
					num++;
				}
			}
		}
	}

	average /= (float)num;
	if (average.x() != 0.0f && average.y() != 0.0f && average.z() != 0.0f)
		average.normalize();

	return average;
}

/// <summary>
/// Samples the scalar volume
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="z"></param>
/// <returns></returns>
float Volume::sampleVolume(int x, int y, int z)
{
	//x = clamp(x, 0, volumeWidth - 1);
	//y = clamp(y, 0, volumeHeight - 1);
	//z = clamp(z, 0, volumeDepth - 1);

	int index = x + (y * volumeWidth) + (z * volumeWidth * volumeHeight);
	return 0.f;//(float)mSamples[index];
}

/// <summary>
/// Samples the gradient volume
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="z"></param>
/// <returns></returns>
Eigen::Vector3f Volume::sampleGradients(int x, int y, int z)
{
	int index = x + (y * volumeWidth) + (z * volumeWidth * volumeHeight);
	Eigen::Vector3f sample = Eigen::Vector3f(
		mGradients[3*index + 0],
		mGradients[3*index + 1],
		mGradients[3*index + 2]);

	return sample;
}

/// <summary>
/// Checks whether the input is in the bounds of the volume data array
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="z"></param>
/// <returns></returns>
bool Volume::isInBounds(int x, int y, int z)
{
	return ((x >= 0 && x < volumeWidth) &&
		(y >= 0 && y < volumeHeight) &&
		(z >= 0 && z < volumeDepth));
}

void Volume::createCube(float x, float y, float z) {
	// Define the 24 vertices of a unit cube
	Vertex cube_Vertices[24] = {
		// Back side
		Vertex( float3( 0.0, 0.0, 0.0), float3(0.0, 0.0, -1.0)),
		Vertex( float3( 0.0, y, 0.0), float3(0.0, 0.0, -1.0)),
		Vertex( float3( x, y, 0.0), float3(0.0, 0.0, -1.0)),
		Vertex( float3( x, 0.0, 0.0), float3(0.0, 0.0, -1.0)),
		// Front side
		Vertex( float3( 0.0, 0.0, z), float3(0.0, 0.0, 1.0)),
		Vertex( float3( x, 0.0, z), float3(0.0, 0.0, 1.0)),
		Vertex( float3( x, y, z), float3(0.0, 0.0, 1.0)),
		Vertex( float3( 0.0, y, z), float3(0.0, 0.0, 1.0)),
		// Top side
		Vertex( float3( 0.0, y, 0.0), float3(0.0, 1.0, 0.0)),
		Vertex( float3( 0.0, y, z), float3(0.0, 1.0, 0.0)),
		Vertex( float3( x, y, z), float3(0.0, 1.0, 0.0)),
		Vertex( float3( x, y, 0.0), float3(0.0, 1.0, 0.0)),
		// Bottom side
		Vertex( float3( 0.0, 0.0, 0.0), float3(0.0, -1.0, 0.0)),
		Vertex( float3( x, 0.0, 0.0), float3(0.0, -1.0, 0.0)),
		Vertex( float3( x, 0.0, z), float3(0.0, -1.0, 0.0)),
		Vertex( float3( 0.0, 0.0, z), float3(0.0, -1.0, 0.0)),
		// Left Side
		Vertex( float3( 0.0, 0.0, 0.0), float3(-1.0, 0.0, 0.0)),
		Vertex( float3( 0.0, 0.0, z), float3(-1.0, 0.0, 0.0)),
		Vertex( float3( 0.0, y, z), float3(-1.0, 0.0, 0.0)),
		Vertex( float3( 0.0, y, 0.0), float3(-1.0, 0.0, 0.0)),
		// Right Side
		Vertex( float3( x, 0.0, 0.0), float3(1.0, 0.0, 0.0)),
		Vertex( float3( x, y, 0.0), float3(1.0, 0.0, 0.0)),
		Vertex( float3( x, y, z), float3(1.0, 0.0, 0.0)),
		Vertex( float3( x, 0.0, z), float3(1.0, 0.0, 0.0))
	};

	glGenBuffersARB( 1, &cubeVerticesVBO );						// Create VBO

	glBindBufferARB( GL_ARRAY_BUFFER_ARB, cubeVerticesVBO );	// Bind VBO buffer
	glBufferDataARB( GL_ARRAY_BUFFER_ARB,						// Copy the vertex data to the VBO
		sizeof(cube_Vertices),									// Get the size of cube_Vertices
		cube_Vertices,											// Give the data for the vertices
		GL_STATIC_DRAW_ARB );									// Tell the buffer that the data is not going to change
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );					// Unbind VBO buffer
}

void Volume::unbindFBO() {
	// 'unbind' the FBO. things will now be drawn to screen as usual
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

GLuint Volume::setupFBO() {													// Create a new frame buffer for off screen rendering
	GLuint fbo_handle;

	glGenFramebuffersEXT(1, &fbo_handle);									// Create buffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);					// Bind buffer

	// The depth buffer
	GLuint depthrenderbuffer;
	glGenRenderbuffers(1, &depthrenderbuffer);								// Create depth buffer
	glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);					// Bind buffer
	glRenderbufferStorage(GL_RENDERBUFFER,									// Create storage
		GL_DEPTH_COMPONENT,													// Specify that the internal format is the depth component
		TEXTURE_SIZE, TEXTURE_SIZE);										// Set storage width and height
	glFramebufferRenderbuffer(GL_FRAMEBUFFER,								// Attach depth buffer to frame buffer
		GL_DEPTH_ATTACHMENT,												// Specify that the internal format is the depth component
		GL_RENDERBUFFER, depthrenderbuffer);								// Attach depth render buffer texture to the frame buffer

	errcheck();																// Check for errors

	unbindFBO();															// Unbind frame buffer object

	return fbo_handle;														// Return new frame buffer
}

bool Volume::bindFBO(GLuint fbo_handle, GLuint fbo_texture) {				// Bind frame buffer and attach texture for rendering
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);					// Bind frame buffer

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,							// Set fbo_texture as our color attachment #0
		GL_COLOR_ATTACHMENT0_EXT,
		GL_TEXTURE_2D, fbo_texture, 0);

	//errcheck();

	GLenum dbuffers[2] = { GL_COLOR_ATTACHMENT0_EXT };						// Set the list of draw buffers.
	glDrawBuffers(1, dbuffers);												// Set Draw buffers
	
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) // Always check that our frame buffer is ok
		return false;

	return true;
}

int Volume::setupCg(CGcontext *context, CGprogram *fProgram, 
			CGprofile *fragmentProfile, char *file) {
	cgSetErrorCallback(CheckCgError);

	*context = cgCreateContext();

	if (!cgIsContext(*context)) {
		fprintf(stderr, "Error: %s\n", "Failed To Create Cg Context");
		return 1;
	}

	*fragmentProfile = cgGLGetLatestProfile(CG_GL_FRAGMENT);

	printf("fragment profile: %s\n",
	       cgGetProfileString(cgGLGetLatestProfile(CG_GL_FRAGMENT)));

	if (*fragmentProfile == CG_PROFILE_UNKNOWN) {
		fprintf(stderr, "Error: %s\n", "Invalid profile type");
		return 1;
	}

	cgGLSetOptimalOptions(*fragmentProfile);

	*fProgram = cgCreateProgramFromFile(*context, CG_SOURCE, file,
			*fragmentProfile, "main", NULL);

	if (!*fProgram) {
		printf("Couldn't create fragment program.\n");
		return 1;
	}

	if (!cgIsProgramCompiled(*fProgram)) {
		cgCompileProgram(*fProgram);
	}

	if (!cgIsProgramCompiled(*fProgram)) {
		printf("Couldn't compile fragment program.\n");
	}
	
	cgGLEnableProfile(*fragmentProfile);

	if (*fProgram) {
		cgGLLoadProgram(*fProgram);
	} else {
		printf("Couldn't load fragment program.\n");
		CheckCgError();
		return 1;
	}

	cgGLDisableProfile(*fragmentProfile);

	return 0;
}

void Volume::render(Camera* camera) {
	int width = camera->getWidth();
	int height = camera->getHeight();

	glViewport(0, 0, TEXTURE_SIZE, TEXTURE_SIZE);			// Set Viewport
	glMatrixMode(GL_PROJECTION);							// Update projection
	glPushMatrix();
	glLoadIdentity();										// Load Identity
	gluPerspective(camera->getFOV(),						// Set Field of View
		(GLfloat)width/(GLfloat)height,						// Set aspect ratio
		camera->getNearClipping(),							// Set near clipping
		camera->getFarClipping());							// Set far clipping
	glMatrixMode(GL_MODELVIEW);								// Change back to model view mode
	glPushMatrix();											// set where to start the current object

	glScalef(
		1.0f, 
		(float)volumeHeight/volumeWidth, 
		(float)volumeDepth/volumeWidth);

	glScalef(3.f, 3.f, 3.f);

	glTranslatef(position.x(), position.y(), position.z());	// set position of the texture cube

	// Convert rotation quaternion into axis angle
	float rotationScale = sqrt(								// Get length of quaternion
		rotation.x() * rotation.x() + 
		rotation.y() * rotation.y()  + 
		rotation.z() * rotation.z());

	float x = rotation.x() / rotationScale;					// Get axis x value
	float y = rotation.y() / rotationScale;					// Get axis y value
	float z = rotation.z() / rotationScale;					// Get axis z value
	float angle = (acos(rotation.w())*2.0f)*(180.f/M_PI);	// Get rotation angle

	glRotatef(angle, x, y, z);								// set rotation of the texture cube

	glTranslatef(-0.5, -0.5, -0.5);							// center the texture cube

	// We need to enable the client stats for the vertex attributes we want 
	// to render even if we are not using client-side vertex arrays.
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	// Bind the vertices's VBO
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, cubeVerticesVBO );
	glVertexPointer( 3, GL_FLOAT, sizeof(Vertex), MEMBER_OFFSET(Vertex,m_Pos) );
	glColorPointer( 3, GL_FLOAT, sizeof(Vertex), MEMBER_OFFSET(Vertex,m_Color) );
	glNormalPointer( GL_FLOAT, sizeof(Vertex), MEMBER_OFFSET(Vertex,m_Normal) );
	
	glEnable(GL_TEXTURE_2D);							// Enable 2D textures
	
	bindFBO(FBO, front_facing);							// Render to our frame buffer using the front_facing texture
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_BACK);								// Remove back facing face's
	glDrawArrays( GL_QUADS, 0, 24 );					// Render Front Facing
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's

	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer
	
	bindFBO(FBO, back_facing);							// Render to our frame buffer using the back_facing texture
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_FRONT);								// Remove front facing face's
	glDrawArrays( GL_QUADS, 0, 24 );					// Render Back Facing
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer

	// Unbind buffers so client-side vertex arrays still work.
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

	// Disable the client side arrays again.
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glPopMatrix();										// Restore state

	// Render Volume
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, TEXTURE_SIZE, TEXTURE_SIZE, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	cgGLEnableProfile(fragmentProfile);
	cgGLBindProgram(fProgram);
	CheckCgError();

	// enable Cg shader and texture (a 'compute' fragment program)
	cgGLSetTextureParameter(cgFrontTexData, front_facing);		// Bind front facing render to cgFrontTexData
	cgGLSetTextureParameter(cgBackTexData, back_facing);		// Bind back facing render to cgBackTexData
	cgGLSetTextureParameter(cgVolumeTexData, volume_texture);	// Bind the voulume_texture to cgVolumeTexData
	cgGLSetTextureParameter(cgTransferTexData, transferTexture);// Bind the transferTexture to cgTransferTexData
	cgGLEnableTextureParameter(cgFrontTexData);					// Enable cgFrontTexData
	cgGLEnableTextureParameter(cgBackTexData);					// Enable cgBackTexData
	cgGLEnableTextureParameter(cgVolumeTexData);				// Enable cgVolumeTexData
	cgGLEnableTextureParameter(cgTransferTexData);				// Enable cgTransferTexData
	
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	
	glBegin(GL_TRIANGLE_FAN);
	glTexCoord2f(0, 1);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1, 1);
	glVertex3f(TEXTURE_SIZE, 0, 0);
	glTexCoord2f(1, 0);
	glVertex3f(TEXTURE_SIZE, TEXTURE_SIZE, 0);
	glTexCoord2f(0, 0);
	glVertex3f(0, TEXTURE_SIZE, 0);
	glEnd();
	
	// disable shader
	cgGLDisableProfile(fragmentProfile);
	CheckCgError();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix(); //end the current object transformations
}
