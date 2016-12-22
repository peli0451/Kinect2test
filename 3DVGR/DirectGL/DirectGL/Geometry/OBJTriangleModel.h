#pragma once

#include <DirectGL/Geometry/MultiMaterialModel.h>

namespace DirectGL
{
	namespace Geometry
	{
		class OBJTriangleModel : public MultiMaterialModel
		{
		public:
			OBJTriangleModel() {}
			OBJTriangleModel(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction) = defaultProgress) { fromFile(filename, progressCallback); }
			OBJTriangleModel(std::ifstream &stream) : MultiMaterialModel(stream) {}
			void fromFile(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction) = defaultProgress);

		private:
			static void defaultProgress(float progress, const std::string &currentAction) {}
		};
		typedef boost::shared_ptr<OBJTriangleModel> OBJTriangleModelPtr;
	}
}