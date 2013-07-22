#include "StdAfx.h"
#include "VolumeLoader.h"

#include "itkImage.h"
#include "itkImageSeriesReader.h"
#include "itkGDCMImageIO.h"
#include "itkRGBAPixel.h"
#include "itkGDCMSeriesFileNames.h"

VolumeLoader::VolumeLoader(void) {
	data = NULL;															// Set the volume data to NULL

	volumeWidth = 1;														// Set the default width to one
	volumeHeight = 1;														// Set the default height to one
	volumeDepth = 1;														// Set the default depth to one

	spacingX = 1.f;															// By default there is no spacing
	spacingY = 1.f;															// And all the dimensions of the
	spacingZ = 1.f;															// voxels are the same
}


VolumeLoader::~VolumeLoader(void) {
	if (data)
		delete [] data;
}

int VolumeLoader::loadRaw(char *directory) {								// Loads raw volume data from a file
	FILE* dataFile = NULL;
	fopen_s(&dataFile, directory, "rb");									// Opens file

	if (dataFile) {															// Checks if file was able to open
		if (data != NULL) {													// Checks if volume data already exists
			delete[] data;													// If so delete it from memory
			data = NULL;													// Set pointer to NULL
		}

		std::cout << "- Saved File Found" << std::endl; 
		fread(&spacingX, sizeof(float), 1, dataFile);						// Read X spacing size
		fread(&spacingY, sizeof(float), 1, dataFile);						// Read Y spacing size 
		fread(&spacingZ, sizeof(float), 1, dataFile);						// Read Z spacing size
		fread(&volumeWidth, sizeof(int), 1, dataFile);						// Read volume width
		fread(&volumeHeight, sizeof(int), 1, dataFile);						// Read volume height
		fread(&volumeDepth, sizeof(int), 1, dataFile);						// Read volume depth

		
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

		pixelCount = volumeWidth * volumeHeight * volumeDepth;				// Calculate the number of voxels
		data = new GLubyte[pixelCount];										// Allocate memory for the volume data
		fread(data, sizeof(GLubyte), pixelCount, dataFile);					// Read volume data from file

		fclose(dataFile);													// Close file

		return EXIT_SUCCESS;
	}

	return EXIT_FAILURE;
}

int VolumeLoader::loadVolume(char *directory) {									// Loads DICOM files
	typedef signed short    PixelType;										// How each pixel is stored
	const unsigned int      Dimension = 3;									// The number of dimensions of the data

	typedef itk::Image< PixelType, Dimension >         ImageType;			// The image type

	typedef itk::ImageSeriesReader<ImageType> ReaderType;					// Image reader type
	ReaderType::Pointer reader = ReaderType::New();							// Create the image reader
	
	typedef itk::GDCMImageIO       ImageIOType;								// IO class for reading DICOM images
	ImageIOType::Pointer dicomIO = ImageIOType::New();						// Create Image IO

	reader->SetImageIO( dicomIO );											// Set the image IO in the reader to read DICOM images

	typedef itk::GDCMSeriesFileNames NamesGeneratorType;					// Generate a sequence of filenames from a DICOM series
	NamesGeneratorType::Pointer nameGenerator = NamesGeneratorType::New();	// Creates new GDCMSeriesFileNames

	nameGenerator->SetUseSeriesDetails( true );								// If multiple volumes are being grouped as a single series for your DICOM objects
	nameGenerator->AddSeriesRestriction("0008|0021" );						// Add more restriction on the selection of a Series

	nameGenerator->SetDirectory( directory );								// Give the directory to read from
	
	try
	{
		std::cout << "- The directory: " << directory << std::endl;
		std::cout << "- Contains the following DICOM Series: ";

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

		std::cout << "- Now reading series: ";
		std::cout << seriesIdentifier;
		std::cout << std::endl;


		typedef std::vector< std::string >   FileNamesContainer;
		FileNamesContainer fileNames;
		fileNames = nameGenerator->GetFileNames( seriesIdentifier );

		// File names to Read
		reader->SetFileNames( fileNames );
		std::cout << "- Reading files ... " << std::endl;

		try
		{
			reader->UpdateLargestPossibleRegion();
			std::cout << "- Successfully read " << fileNames.size() << " file(s)." << std::endl;
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

int VolumeLoader::getPixelCount() {
	return pixelCount;
}

GLubyte* VolumeLoader::getData() {
	return data;
}

float VolumeLoader::getSpacingX() {
	return spacingX;
}

float VolumeLoader::getSpacingY() {
	return spacingY;
}

float VolumeLoader::getSpacingZ() {
	return spacingZ;
}

int VolumeLoader::getVolumeWidth() {
	return volumeWidth;
}

int VolumeLoader::getVolumeHeight() {
	return volumeHeight;
}

int VolumeLoader::getVolumeDepth() {
	return volumeDepth;
}
