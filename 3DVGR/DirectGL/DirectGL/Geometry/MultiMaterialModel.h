#pragma once

#include <DirectGL/Geometry/MaterializedModel.h>

#include <boost/smart_ptr.hpp>
#include <vector>

namespace DirectGL
{
	namespace Geometry
	{
		class MultiMaterialModel : public Renderable
		{
		public:
			using MaterialGroup = MaterializedModel;
			using Ptr = boost::shared_ptr<MultiMaterialModel>;
			MultiMaterialModel() {}
			MultiMaterialModel(std::ifstream &stream) { fromFileStream(stream); }

			~MultiMaterialModel() { clear(); }

			void draw(Shaders::Program &program);
			void draw();
			void drawMaterialGroup(std::size_t index, Shaders::Program &program) { m_groups[index].draw(program); }
			void drawMaterialGroup(std::size_t index) { m_groups[index].draw(); }

			void toFileStream(std::ofstream &stream) const;
			void fromFileStream(std::ifstream &stream);

			void addMaterialGroup(const MaterialGroup &model); // TODO removing?

			const MaterialGroup &getMaterialGroup(std::size_t index) const { return m_groups[index]; } // TODO errors
			size_t getGroupCount() const { return m_groups.size(); }
			bool isEmtpy() const { return m_groups.empty(); }
			
			void clear();
			float raycut(const Position3D &start, const Position3D &ray, Direction3D &normal) const;

			void transform(const Cameras::Transformation &transform);

			bool isAreaLight() const { return m_areaLight; }
			void setAreaLight(bool enable) { m_areaLight = enable; }


		private:
			typedef std::vector<MaterialGroup> MaterialGroups;
			MaterialGroups m_groups;

			bool m_areaLight = false;
		};
		typedef boost::shared_ptr<MultiMaterialModel> MultiMaterialModelPtr;
	}
}