#pragma once

//#define WIN32_LEAN_AND_MEAN   
//
//#include <windows.h>
//#include <objidl.h>
//#include <gdiplus.h>

#include <vector>
#include "DirectGL\Texturing\Texture2D.h"

namespace DirectGL
{
	namespace Utility
	{
		class ImageIO
		{
		private:
			ImageIO() {}
			ImageIO(const ImageIO &other) {}
			//class GDIPlusImageLoader
			//{
			//public:
			//	GDIPlusImageLoader();
			//	~GDIPlusImageLoader();
			//
			//private:
			//	ULONG_PTR m_gdiplusToken;
			//};

			class FreeImageImageLoader
			{
			public:
				FreeImageImageLoader();
				~FreeImageImageLoader();
			};

		public:
			static Texturing::Texture2DPtr loadImage(const std::string &path);
			static void saveImage(const Texturing::Texture2DPtr &texture, const std::string &path);

		private:

			//static GDIPlusImageLoader m_loader;
			static FreeImageImageLoader m_freeLoader;
		};
	}
}