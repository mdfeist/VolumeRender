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
#include "Cubic.h"
#include "VolumeCube.h"

// All textures for the buffer are TEXTURE_SIZExTEXTURE_SIZE in dimensions
#define TEXTURE_SIZE 512
// Used to find the offset of variables in a structure (found when binding the VBO)
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))

const GLushort  NUM_BOX_INDICES = 36;			// Number of indices for a single box
const GLushort  NUM_BOX_VERTICES = 8;			// Number of vertices for a single box
const GLushort  NUM_BOX_TRIS = 12;				// Number of triangles for a single box
GLushort  VolumeIndices[] =					// Array of indices for a single box
{
	// front
    0, 1, 2,
    2, 3, 0,
    // top
    3, 2, 6,
    6, 7, 3,
    // back
    7, 6, 5,
    5, 4, 7,
    // bottom
    4, 5, 1,
    1, 0, 4,
    // left
    4, 0, 3,
    3, 7, 4,
    // right
    1, 5, 6,
    6, 2, 1,
};

struct float3 {										// A three dimensional float
													// Variables for each dimension
	float x;										// x dimension
	float y;										// y dimension
	float z;										// z dimension								
	
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

struct Vertex										// Vertex used for creating VBO (Includes Normals)
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

struct VertexPositionColor							// Vertex used for creating VBO (No Normals)
{
	float3 m_Pos;									// Position of vertex
    float3 m_Color;									// Color of vertex

	VertexPositionColor() {}							// Generic constructor

	VertexPositionColor(float3 pos, float3 color) {	// Constructor with given values
		m_Pos = pos;								// Set the position
		m_Color = color;							// Set color to be the same as position
	}
};

float length(float3 p) {							// Get the length of a float3
	return sqrtf(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline int clamp(int x, int a, int b) {
	if (x < a)
		x = a;
	if (x > b)
		x = b;

    return x;
}

inline void loadingBar(int x, int n, int w) {
	// Calculuate the ratio of complete-to-incomplete.
	float ratio = x/(float)n;
	int c = ratio * w;

	std::cout << std::setw(3) << (int)(ratio*100) << "% [";
	for (int x=0; x<c; x++) std::cout << "=";
	for (int x=c; x<w; x++) std::cout << " ";
	std::cout << "]\r" << std::flush;
}

GLuint createNoise() {
	srand(time(NULL));
	GLfloat* pbyData = new GLfloat[TEXTURE_SIZE*TEXTURE_SIZE];

	// Creating Noise data 
    for(int i = 0; i<TEXTURE_SIZE; i++)
	{
        for(int j = 0; j<TEXTURE_SIZE; j++)
        {
            int offset = (i*TEXTURE_SIZE+j);
			pbyData[offset] = (GLfloat)((int)rand() % 512)/512.f;
		}
	}

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
		GL_RED,																// Use internal format GL_RED
		TEXTURE_SIZE,														// Set the texture width
		TEXTURE_SIZE,														// Set the texture height
		0,																	// Set the boarder to zero (Needs to be 0)
		GL_RED,																// Use format GL_RED
		GL_FLOAT,															// Use type GL_FLOAT
		pbyData);															// Set data to NULL

	delete[] pbyData;

	return texture;															// Return texture
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
		Sleep(1000);
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
	data = NULL;
	transfer = NULL;

	cubeVerticesVBO = 0;													// Set cubeVerticesVBO to NULL
	initialized = false;													// Set initialized to false

	position = Eigen::Vector3f(0, 0, 0);
	rotation = Eigen::Quaternionf::Identity();

	isoValue = 0.0f;

	spacingX = 1.f;
	spacingY = 1.f;
	spacingZ = 1.f;
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
	printf("- FBO created\n");

	front_facing = newTexture(TEXTURE_SIZE, TEXTURE_SIZE);
	back_facing = newTexture(TEXTURE_SIZE, TEXTURE_SIZE);

	colorTexture = newTexture(TEXTURE_SIZE, TEXTURE_SIZE);

	noiseTexture = createNoise();
	printf("- Textures created\n");

	char shaderFirstPassFile[] = "shader/raycastDiffuse.cg";
	if (setupCg(&context, &fProgramFirstPass, &fragmentProfile, shaderFirstPassFile)) {
		fprintf(stderr, "Error: %s\n", "Initializing Cg");
		CheckCgError();
	}

	cgFrontTexData = cgGetNamedParameter(fProgramFirstPass, "frontTexData");
	cgBackTexData = cgGetNamedParameter(fProgramFirstPass, "backTexData");
	cgNoiseTexData = cgGetNamedParameter(fProgramFirstPass, "noiseTexData");
	cgVolumeTexData = cgGetNamedParameter(fProgramFirstPass, "VolumeS");
	cgTransferTexData = cgGetNamedParameter(fProgramFirstPass, "TransferS");
	cgStepSize = cgGetNamedParameter(fProgramFirstPass, "stepSize");
	cgisoValue = cgGetNamedParameter(fProgramFirstPass, "isoValue");

	cgGLSetParameter1f(cgStepSize, 1.0f/780.0f);				// Set the incremental step size of the ray cast

	char shaderSecondPassFile[] = "shader/bicubic.cg";
	if (setupCg(&context, &fProgramSecondPass, &fragmentProfile, shaderSecondPassFile)) {
		fprintf(stderr, "Error: %s\n", "Initializing Cg");
		CheckCgError();
	}

	cgColorTexData = cgGetNamedParameter(fProgramSecondPass, "texData");

	//createCube(1.0f, 1.0f, 1.0f);
	
	buildVertBuffer();
	printf("- VBO created\n");

	volume_texture = createVolume();
	printf("- Volume texture created\n");

	/*
	for (int i = 0; i < 256; i++) {
		printf("%3d	\t%3d %3d %3d %3d\n", 
			i, 
			transfer[4*i + 0],
			transfer[4*i + 1],
			transfer[4*i + 2],
			transfer[4*i + 3]);
	}
	*/
	glGenTextures(1, &transferTexture);
	glBindTexture(GL_TEXTURE_1D, transferTexture);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, transfer);

	errcheck();

	free(transfer);

	initialized = true;
}

bool Volume::needsInit() {
	return !initialized;
}

void Volume::setup() {
	std::cout << "- Computing Transfer Function" << std::endl; 
	computeTransferFunction();

	std::cout << "- Recursively Building Cubes for Empty Space Leaping" << std::endl; 
	VolumeCube C(0.f, 0.f, 0.f, 1.f, 1.f, 1.f);
	recursiveVolumeBuild(C);
	std::cout << "- Cubes Created" << std::endl; 
}

int Volume::loadRaw(char *directory) {
	// reopen file, and read the data
	FILE* dataFile = NULL;
	
	fopen_s(&dataFile, directory, "rb");

	if (dataFile) {
		if (data != NULL) {
			delete[] data;
			data = NULL;
		}

		std::cout << "- Saved File Found" << std::endl; 
		fread(&spacingX, sizeof(float), 1, dataFile);
		fread(&spacingY, sizeof(float), 1, dataFile); 
		fread(&spacingZ, sizeof(float), 1, dataFile);
		fread(&volumeWidth, sizeof(int), 1, dataFile); 
		fread(&volumeHeight, sizeof(int), 1, dataFile);
		fread(&volumeDepth, sizeof(int), 1, dataFile); 

		
		std::cout << "- Volume Spacing: " 
			<< "[" 
			<< spacingX << ", "
			<< spacingY << ", "
			<< spacingZ
			<< "]" << std::endl;

		std::cout << "- Volume Dimension: " 
			<< "[" 
			<< volumeWidth << ", "
			<< volumeHeight << ", "
			<< volumeDepth
			<< "]" << std::endl;

		pixelCount = volumeWidth * volumeHeight * volumeDepth;
		data = new GLubyte[pixelCount];
		fread(data, sizeof(GLubyte), pixelCount, dataFile);

		fclose(dataFile);

		return EXIT_SUCCESS;
	}

	return EXIT_FAILURE;
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
		ImageType::SpacingType spacing = image->GetSpacing();

		// Pointer to the start of the Image
		signed short * bufferPointer = reader->GetOutput()->GetBufferPointer();

		// Get number of pixels in image
		pixelCount = size[0] * size[1] *size[2];

		volumeWidth = size[0];
		volumeHeight = size[1];
		volumeDepth = size[2];

		spacingX = spacing[0];
		spacingY = spacing[1];
		spacingZ = spacing[2];

		std::cout << "- Volume Size: " 
			<< "[" 
			<< volumeWidth << ", "
			<< volumeHeight << ", "
			<< volumeDepth
			<< "]" << std::endl;

		if (data != NULL) {
			delete[] data;
			data = NULL;
		}

		// Create array for iso values
		data = new GLubyte[pixelCount];
		memset(data, 0, pixelCount);

		std::cout << "- Normalizing data" << std::endl; 

		float min = -1000.f;
		float max = 1000.f;

		// Normalize values
		for (unsigned int i = 0; i < pixelCount; i++)
		{
			float normal = ((float)bufferPointer[i] - min)/(max - min);

			if (normal < 0.f)
				normal = 0.f;
			else if (normal > 1.f)
				normal = 1.f;

			data[i] = (GLubyte)(255.f*normal);
		}
	}
	catch (itk::ExceptionObject &ex)
    {
		std::cout << ex << std::endl;
		return EXIT_FAILURE;
    }
	
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
	glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, 
		volumeWidth, volumeHeight, volumeDepth, 
		0, GL_RED, GL_UNSIGNED_BYTE, data);

	delete []data;

	return volume_texture;
}

void Volume::computeTransferFunction() {
	// Test spline CT_BONE
	/*
	 colorFun->AddRGBPoint( -3024, 0, 0, 0, 0.5, 0.0 );
      colorFun->AddRGBPoint( -16, 0.73, 0.25, 0.30, 0.49, .61 );
      colorFun->AddRGBPoint( 641, .90, .82, .56, .5, 0.0 );
      colorFun->AddRGBPoint( 3071, 1, 1, 1, .5, 0.0 );
      
      opacityFun->AddPoint(-3024, 0, 0.5, 0.0 );
      opacityFun->AddPoint(-16, 0, .49, .61 );
      opacityFun->AddPoint(641, .72, .5, 0.0 );
      opacityFun->AddPoint(3071, .71, 0.5, 0.0);
	  */
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
	alphaKnots.push_back( new TransferControlPoint(0.8f, 112) );	// Fat
	alphaKnots.push_back( new TransferControlPoint(0.7f, 128) );	// Fat
	alphaKnots.push_back( new TransferControlPoint(0.2f, 129) );	// Blood/Muscle
	alphaKnots.push_back( new TransferControlPoint(0.05f, 132) );	// Blood/Muscle
	alphaKnots.push_back( new TransferControlPoint(0.3f, 133) );	// Liver
	alphaKnots.push_back( new TransferControlPoint(0.1f, 136) );	// Liver
	alphaKnots.push_back( new TransferControlPoint(0.7f, 140) );	// Soft Tissue
	alphaKnots.push_back( new TransferControlPoint(0.35f, 166) );	// Soft Tissue
	alphaKnots.push_back( new TransferControlPoint(0.95f, 210) );	// Bone
	alphaKnots.push_back( new TransferControlPoint(1.f, 256) );		// Bone

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
	for (int i = 0; i < colorKnots.size() - 1; i++)
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
	for (int i = 0; i < alphaKnots.size() - 1; i++)
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

int Volume::sampleVolume(int x, int y, int z)
{
	x = (int)clamp(x, 0, volumeWidth - 1);
	y = (int)clamp(y, 0, volumeHeight - 1);
	z = (int)clamp(z, 0, volumeDepth - 1);

	return (int)data[x + (y * volumeWidth) + (z * volumeWidth * volumeHeight)];
}

float Volume::sampleVolume3DWithTransfer(Eigen::Vector3f& min, Eigen::Vector3f& max)
{
	float result = 0.0f;

	for (int x = (int)floorf(min.x()); x <= (int)ceilf(max.x()); x++)
	{
		for (int y = (int)floorf(min.y()); y <= (int)ceilf(max.y()); y++)
		{
			for (int z = (int)floorf(min.z()); z <= (int)ceilf(max.z()); z++)
			{
				//sample the volume to get the iso value
				int isovalue = sampleVolume(x, y, z);

				//accumulate the opacity from the transfer function
				result += (float)transfer[4*isovalue + 3];
			}
		}
	}

	return result;
}

void Volume::recursiveVolumeBuild(VolumeCube C)
{
	//stop when the current cube is 1/10 of the original volume
	if (C.Width <= 0.1f)
	{
		//add the min/max vertex to the list
		Eigen::Vector3f min = Eigen::Vector3f(C.X, C.Y, C.Z);
		Eigen::Vector3f max = Eigen::Vector3f(C.X + C.Width, C.Y + C.Height, C.Z + C.Depth);
		Eigen::Vector3f min_pi = Eigen::Vector3f(
			C.X * volumeWidth,
			C.Y * volumeHeight,
			C.Z * volumeDepth);
		Eigen::Vector3f max_pi = Eigen::Vector3f(
			(C.X + C.Width) * volumeWidth,
			(C.Y + C.Height) * volumeHeight,
			(C.Z + C.Depth) * volumeDepth);

		//additively sample the transfer function and check if there are any
		//samples that are greater than zero
		float opacity = sampleVolume3DWithTransfer(min_pi, max_pi);

		if (opacity > 0.0f)
		{

			float3 pos1(min.x(), min.y(), max.z());
			float3 pos2(max.x(), min.y(), max.z());
			float3 pos3(max.x(), max.y(), max.z());
			float3 pos4(min.x(), max.y(), max.z());
			float3 pos5(min.x(), min.y(), min.z());
			float3 pos6(max.x(), min.y(), min.z());
			float3 pos7(max.x(), max.y(), min.z());
			float3 pos8(min.x(), max.y(), min.z());

			VertexPositionColor v1(pos1, pos1);
			VertexPositionColor v2(pos2, pos2);
			VertexPositionColor v3(pos3, pos3);
			VertexPositionColor v4(pos4, pos4);
			VertexPositionColor v5(pos5, pos5);
			VertexPositionColor v6(pos6, pos6);
			VertexPositionColor v7(pos7, pos7);
			VertexPositionColor v8(pos8, pos8);

			mVertices.push_back(v1);
			mVertices.push_back(v2);
			mVertices.push_back(v3);
			mVertices.push_back(v4);
			mVertices.push_back(v5);
			mVertices.push_back(v6);
			mVertices.push_back(v7);
			mVertices.push_back(v8);
		}
		return;
	}

	float newWidth = C.Width / 2.f;
	float newHeight = C.Height / 2.f;
	float newDepth = C.Depth / 2.f;

	///  SubGrid        r | c | d
	///  Front:
	///  Top-Left    :  0   0   0
	///  Top-Right   :  0   1   0
	///  Bottom-Left :  1   0   0
	///  Bottom-Right:  1   1   0
	///  Back:
	///  Top-Left    :  0   0   1
	///  Top-Right   :  0   1   1
	///  Bottom-Left :  1   0   1
	///  Bottom-Right:  1   1   1
	for (float r = 0; r < 2; r++)
	{
		for (float c = 0; c < 2; c++)
		{
			for (float d = 0; d < 2; d++)
			{
				VolumeCube cube = VolumeCube(C.Left() + c * (newWidth),
					C.Top() + r * (newHeight),
					C.Front() + d * (newDepth),
					newWidth,
					newHeight,
					newDepth);

				recursiveVolumeBuild(cube);
			}
		}
	}
}

void Volume::buildVertBuffer()
{
	VertexPositionColor* data = new VertexPositionColor[mVertices.size()];
	std::copy(mVertices.begin(), mVertices.end(), data);
	unsigned int mNumBoxes = (mVertices.size() / 8);
	unsigned int mNumVertices = mVertices.size();
	unsigned int mNumTris = mNumBoxes * NUM_BOX_TRIS;

	mNumIndices = mNumBoxes * NUM_BOX_INDICES;

	//we are handling the adding vertex condition
	GLuint* indices = new GLuint [mNumIndices];
	for (int copyIndex = 0; copyIndex < mNumBoxes; copyIndex++)
	{
		for (int index = 0; index < NUM_BOX_INDICES; index++)
		{
			indices[copyIndex * NUM_BOX_INDICES + index] = (GLuint)(VolumeIndices[index] + (copyIndex) * NUM_BOX_VERTICES);
		}
	}

	// Create VBO for vertices
	glGenBuffersARB( 1, &cubeVerticesVBO );							// Create VBO

	glBindBufferARB( GL_ARRAY_BUFFER_ARB, cubeVerticesVBO );		// Bind VBO buffer
	glBufferDataARB( GL_ARRAY_BUFFER_ARB,							// Copy the vertex data to the VBO
		mNumVertices * sizeof(VertexPositionColor),					// Get the size of data
		&data[0],													// Give the data for the vertices
		GL_STATIC_DRAW_ARB );										// Tell the buffer that the data is not going to change
	
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );						// Unbind VBO buffer
	
	// Create VBO for indices
	glGenBuffersARB( 1, &cubeIndicesVBO );							// Create buffer

	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, cubeIndicesVBO );	// Bind buffer
	glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,					// Copy the indices data to the buffer
		mNumIndices * sizeof(GLuint),								// Get the size of indices
		&indices[0],												// Give the data for the indices
		GL_STATIC_DRAW_ARB );										// Tell the buffer that the data is not going to change

	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, 0 );				// Unbind buffer

	delete[] indices;
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
	glGenRenderbuffers(1, &depthrenderbuffer);								// Create depth buffer
	glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);					// Bind buffer

	// No need to force GL_DEPTH_COMPONENT24, drivers usually give you the max precision if available
	//glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, TEXTURE_SIZE, TEXTURE_SIZE, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	//glBindTexture(GL_TEXTURE_2D, 0);

	// attach the texture to FBO depth attachment point
	//glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, depthrenderbuffer, 0);
	
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

bool Volume::bindFBO(GLuint fbo_handle, GLuint *fbo_texture, GLsizei size) {	// Bind frame buffer and attach textures for rendering
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_handle);						// Bind frame buffer

	if (size > GL_MAX_COLOR_ATTACHMENTS)										// Limit number of color attachments to the GL_MAX_COLOR_ATTACHMENTS
		size = GL_MAX_COLOR_ATTACHMENTS;

	for (int i = 0; i < size; i++)
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,							// Set fbo_texture as our color attachment #0
			GL_COLOR_ATTACHMENT0_EXT + i,
			GL_TEXTURE_2D, fbo_texture[i], 0);

	//errcheck();

	GLenum dbuffers[GL_MAX_COLOR_ATTACHMENTS];						 			// Set the list of draw buffers.

	for (int i = 0; i < size; i++)
		dbuffers[i] = GL_COLOR_ATTACHMENT0_EXT + i;

	glDrawBuffers(size, dbuffers);												// Set Draw buffers
	
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)		// Always check that our frame buffer is ok
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

void Volume::setIsoValue(float value) {
	isoValue = value;

	if (isoValue > 1.0f)
		isoValue = 1.0f;
	else if (isoValue < 0.0f)
		isoValue = 0.0f;
}


void Volume::increaseIsoValue(float value) {
	isoValue += value;

	if (isoValue > 1.0f)
		isoValue = 1.0f;
	else if (isoValue < 0.0f)
		isoValue = 0.0f;
}

void Volume::render(Camera* camera) {
	int width = camera->getWidth();							// Get Camera Width
	int height = camera->getHeight();						// Get Camera Height
	
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
	
	glScalef(												// Scale Volume to proper proportions
		1.0f,												// Scale width (Always 1.0f)
		(spacingY/spacingX)*								// Scale the voxel spacing height relative to spacing width
			((float)volumeHeight/volumeWidth),				// Scale the height dimension relative to the width
		(spacingZ/spacingX)*								// Scale the voxel spacing depth relative to spacing width
			((float)volumeDepth/volumeWidth));				// Scale the depth dimension relative to the width
			
	//glScalef(2.f, 2.f, 2.f);
	glScalef(1.5f, 1.5f, 1.5f);								// Scale the volume

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

	// Bind the vertices's VBO
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, cubeVerticesVBO );
	glVertexPointer( 3, GL_FLOAT, sizeof(VertexPositionColor), MEMBER_OFFSET(VertexPositionColor,m_Pos) );
	glColorPointer( 3, GL_FLOAT, sizeof(VertexPositionColor), MEMBER_OFFSET(VertexPositionColor,m_Color) );

	// We need to enable the client stats for the vertex attributes we want 
	// to render even if we are not using client-side vertex arrays.
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_INDEX_ARRAY);

	// Bind the indices VBO
	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, cubeIndicesVBO );

	glEnable( GL_TEXTURE_2D );							// Enable 2D textures
	
	bindFBO(FBO, &back_facing,  1);						// Render to our frame buffer using the back_facing texture

	glClearDepth(0.0f);									// Depth Buffer Setup
	glDepthFunc(GL_GREATER);							// The Type Of Depth Testing To Do

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_FRONT);								// Remove front facing face's
	glDrawElements(	GL_TRIANGLES,						// Render Back Facing
		mNumIndices, GL_UNSIGNED_INT, 0);
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer

	bindFBO(FBO, &front_facing, 1);						// Render to our frame buffer using the front_facing texture
	
	glClearDepth(1.0f);									// Depth Buffer Setup
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// Clear color and depth buffer's

	glEnable(GL_CULL_FACE);								// Enable the ability to remove face's
	glCullFace(GL_BACK);								// Remove back facing face's
	glDrawElements(	GL_TRIANGLES,						// Render Front Facing
		mNumIndices, GL_UNSIGNED_INT, 0);				
	glDisable(GL_CULL_FACE);							// Disable the ability to remove face's
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);				// Unbind frame buffer

	// Unbind buffers so client-side vertex arrays still work.
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );
	glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, 0 );

	// Disable the client side arrays again.
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);;
	glDisableClientState(GL_INDEX_ARRAY);

	glPopMatrix();												// Restore state

	// First Pass Render Volume
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, TEXTURE_SIZE, TEXTURE_SIZE, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	bindFBO(FBO, &colorTexture, 1);								// Render to our frame buffer using the front_facing texture

	cgGLEnableProfile(fragmentProfile);
	cgGLBindProgram(fProgramFirstPass);
	CheckCgError();

	// enable Cg shader and texture (a 'compute' fragment program)
	cgGLSetTextureParameter(cgFrontTexData, front_facing);		// Bind front facing render to cgFrontTexData
	cgGLSetTextureParameter(cgBackTexData, back_facing);		// Bind back facing render to cgBackTexData
	cgGLSetTextureParameter(cgNoiseTexData, noiseTexture);		// Bind noiseTexture to cgNoiseTexData
	cgGLSetTextureParameter(cgVolumeTexData, volume_texture);	// Bind the voulume_texture to cgVolumeTexData
	cgGLSetTextureParameter(cgTransferTexData, transferTexture);// Bind the transferTexture to cgTransferTexData
	cgGLEnableTextureParameter(cgFrontTexData);					// Enable cgFrontTexData
	cgGLEnableTextureParameter(cgBackTexData);					// Enable cgBackTexData
	cgGLEnableTextureParameter(cgNoiseTexData);					// Enable cgNoiseTexData
	cgGLEnableTextureParameter(cgVolumeTexData);				// Enable cgVolumeTexData
	cgGLEnableTextureParameter(cgTransferTexData);				// Enable cgTransferTexData

	cgGLSetParameter1f(cgisoValue, isoValue);

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

	cgGLDisableTextureParameter(cgFrontTexData);				// Disable cgFrontTexData
	cgGLDisableTextureParameter(cgBackTexData);					// Disable cgBackTexData
	cgGLDisableTextureParameter(cgNoiseTexData);				// Disable cgNoiseTexData
	cgGLDisableTextureParameter(cgVolumeTexData);				// Disable cgVolumeTexData
	cgGLDisableTextureParameter(cgTransferTexData);				// Disable cgTransferTexData
	
	// disable shader
	cgGLDisableProfile(fragmentProfile);
	CheckCgError();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);						// Unbind frame buffer
	
	glPopMatrix();												// end the current object transformations
	
	// Second Pass Render Volume
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, TEXTURE_SIZE, TEXTURE_SIZE, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	cgGLEnableProfile(fragmentProfile);
	cgGLBindProgram(fProgramSecondPass);
	CheckCgError();

	// enable Cg shader and texture (a 'compute' fragment program)
	//cgGLSetTextureParameter(cgColorTexData, front_facing);
	//cgGLSetTextureParameter(cgColorTexData, back_facing);
	cgGLSetTextureParameter(cgColorTexData, colorTexture);		// Bind color render to cgColorTexData
	cgGLEnableTextureParameter(cgColorTexData);					// Enable cgColorTexData

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
	
	cgGLDisableTextureParameter(cgColorTexData);				// Disable cgColorTexData
	
	// disable shader
	cgGLDisableProfile(fragmentProfile);
	CheckCgError();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();												//end the current object transformations
	
}
