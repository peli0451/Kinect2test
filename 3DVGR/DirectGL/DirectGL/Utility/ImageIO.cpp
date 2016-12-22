#include "stdafx.h"
#include "ImageIO.h"
#include "String.h"
#include "Logging.h"
#include <FreeImage.h>

//DirectGL::Utility::ImageIO::GDIPlusImageLoader DirectGL::Utility::ImageIO::m_loader;
DirectGL::Utility::ImageIO::FreeImageImageLoader DirectGL::Utility::ImageIO::m_freeLoader;

//DirectGL::Utility::ImageIO::GDIPlusImageLoader::GDIPlusImageLoader()
//{
//	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
//
//	Gdiplus::GdiplusStartup(&m_gdiplusToken, &gdiplusStartupInput, nullptr);
//}
//
//DirectGL::Utility::ImageIO::GDIPlusImageLoader::~GDIPlusImageLoader()
//{
//	Gdiplus::GdiplusShutdown(m_gdiplusToken);
//}

DirectGL::Utility::ImageIO::FreeImageImageLoader::FreeImageImageLoader()
{
	FreeImage_Initialise();
}

DirectGL::Utility::ImageIO::FreeImageImageLoader::~FreeImageImageLoader()
{
	FreeImage_DeInitialise();
}

DirectGL::Texturing::Texture2DPtr DirectGL::Utility::ImageIO::loadImage(const std::string &path)
{
	FIBITMAP *bitmap = FreeImage_Load(FreeImage_GetFileType(path.c_str()), path.c_str());
	
	if (!bitmap)
	{
		LOGERROR("Failed to load image <" + path + ">!");
		return nullptr;
	}
	
	FIBITMAP *conv = FreeImage_ConvertTo24Bits(bitmap);
	FreeImage_Unload(bitmap);

	unsigned width = FreeImage_GetWidth(conv),
			height = FreeImage_GetHeight(conv);
	std::vector<GLubyte> imageData(width * height * 3);
	GLubyte *pixels = FreeImage_GetBits(conv);
	for (unsigned int y = 0, pos = 0; y < height; ++y)
	for (unsigned int x = 0; x < width; ++x)
	{
		int mempos = 3 * (x + y * width);
		imageData[pos++] = pixels[mempos + 2]; // FreeImage seems to use a BGR color order
		imageData[pos++] = pixels[mempos + 1];
		imageData[pos++] = pixels[mempos];
	}

	Texturing::Texture2DPtr retVal(new Texturing::Texture2D(true));
	retVal->setImage(width, height, imageData);
	FreeImage_Unload(conv);
	return retVal;
	//Gdiplus::Bitmap *bitmap = Gdiplus::Bitmap::FromFile(string_cast<std::wstring>(path).c_str());
	//
	//if (!bitmap)
	//{
	//	LOGERROR("Failed to load image <" + path + ">!");
	//	return;
	//}
	//
	//std::vector<GLubyte> imageData(bitmap->GetHeight() * bitmap->GetWidth() * 3);
	//for (unsigned int y = 0, pos = 0; y < bitmap->GetHeight(); ++y)
	//for (unsigned int x = 0; x < bitmap->GetWidth(); ++x)
	//{
	//	Gdiplus::Color color;
	//	bitmap->GetPixel(x, bitmap->GetHeight() - y - 1, &color);
	//	imageData[pos++] = color.GetR();
	//	imageData[pos++] = color.GetG();
	//	imageData[pos++] = color.GetB();
	//}
	//
	//image.setImage(bitmap->GetWidth(), bitmap->GetHeight(), imageData);
	//delete bitmap;
}

void DirectGL::Utility::ImageIO::saveImage(const Texturing::Texture2DPtr &texture, const std::string &path)
{
	auto width(texture->getWidth()), height(texture->getHeight());
	FIBITMAP *bitmap = FreeImage_Allocate(width, height, 32);
	GLubyte *pixels = FreeImage_GetBits(bitmap);
	//std::memcpy(pixels, texture->getData().data(), sizeof(GLubyte)* texture->getData().size());
	auto &texData(texture->getData());
	
	for (GLsizei y = 0, pos = 0; y < height; ++y)
	for (GLsizei x = 0; x < width; ++x)
	{
		int mempos = 4 * (x + y * width);
		pixels[mempos + 3] = 255;
		pixels[mempos + 2] = texData[pos++]; // FreeImage seems to use a BGR color order
		pixels[mempos + 1] = texData[pos++];
		pixels[mempos] = texData[pos++];
	}
	FreeImage_Save(FIF_PNG, bitmap, path.c_str());
	FreeImage_Unload(bitmap);
}