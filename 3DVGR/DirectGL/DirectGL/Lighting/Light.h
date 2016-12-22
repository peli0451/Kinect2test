/** @file Light.h ===========================================================
*
*	Declares the Light class, a Proxy class to encapsulate different Light
*	properties and convenience methods to configure shaders and calculate
*	additional information like shadow maps.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <epoxy/gl.h>

#include <DirectGL/Renderable.h>
#include <DirectGL/Types.h>
#include <DirectGL/Texturing/Color.h>
#include <DirectGL/Geometry/PointModel.h>
#include <DirectGL/Shaders/Program.h>
#include <DirectGL/Texturing/Texture2D.h>
#include <DirectGL/Cameras/QuaternionCamera.h>
#include <DirectGL/Buffers/UniformBuffer.h>

#include <boost/smart_ptr.hpp>

#include <algorithm>

namespace DirectGL
{
	namespace Lighting
	{
		class Light : public Renderable
		{
		public:
		// TYPEDEFS ===================================================================
			using LightUniformBuffer = Buffers::UniformBuffer;


		// ENUMS ======================================================================
			enum Type
			{
				Point = 0,
				Directional = 1
			};

		// CONSTRUCTORS ===============================================================
			Light(const Position3D &position = Position3D::Zero(),
				  const Texturing::Color3f &ambient = Texturing::Color3f::Zero(),
				  const Texturing::Color3f &diffuse = Texturing::Color3f::Ones(),
				  const Texturing::Color3f &specular = Texturing::Color3f::Zero(),
				  GLfloat intensity = 1.f, GLfloat cutoffCosine = -1.f, GLfloat falloffExponent = 8.f, const Type &type = Point) :
				  m_ambient(ambient), m_diffuse(diffuse), m_specular(specular), m_intensity(intensity), m_cosineCutoff(cutoffCosine), m_falloffExponent(falloffExponent), m_type(type)
			{
				m_transformation.setTranslation(Eigen::Translation3f(position));
				m_aabb.bound(Position3D::Ones() * -0.01f, Position3D::Ones() * 0.01f);
			}
			inline Light(std::ifstream &stream){ fromFileStream(stream); }
			inline ~Light() { clear(); }

		// SERIALIZERS ================================================================
			void toFileStream(std::ofstream &stream) const;
			void fromFileStream(std::ifstream &stream);

		// RENDERABLE OVERRIDES =======================================================
			void draw() override;
			void draw(Shaders::Program &program) override { draw(); }
			void setTransformation(const Cameras::Transformation &transformation); 
			void clear();

		// METHODS ====================================================================
			virtual void drawIcon();
			void bindToProgram(Shaders::Program &program);
			void renderShadowMap(const RenderableList &scene);

		// GETTERS ====================================================================
			inline const Texturing::Color3f &getAmbient() const { return m_ambient; }
			inline const Texturing::Color3f &getDiffuse() const { return m_diffuse; }
			inline const Texturing::Color3f &getSpecular() const { return m_specular; }
			inline GLfloat getIntensity() const{ return m_intensity; }
			GLfloat getFalloffCorrectedIntensity() const;
			inline GLfloat getCutoffCosine() const { return m_cosineCutoff; }
			inline GLfloat getFalloffExponent() const { return m_falloffExponent; }
			inline Direction3D getDirection() const { return m_transformation.getRotation()._transformVector(Direction3D(0.f, 0.f, -1.f)); }
			inline const Texturing::Texture2D::Ptr &getShadowMap() const { return m_shadowMap; }
			inline const DirectGL::Cameras::QuaternionCamera &getShadowMapCamera() const { return m_lastShadowCamera; }
			inline bool isShadowMapped() const { return m_shadowMap.get() != nullptr; }
			inline const Type &getType() const { return m_type; }

		// SETTERS ====================================================================
			inline void setAmbient(const Texturing::Color3f &ambient) { m_ambient = ambient; }
			inline void setDiffuse(const Texturing::Color3f &diffuse) { m_diffuse = diffuse; m_uniformBuffer.setUniform("Light.diffuse", sizeof(diffuse), diffuse.data()); }
			inline void setSpecular(const Texturing::Color3f &specular) { m_specular = specular; m_uniformBuffer.setUniform("Light.specular", sizeof(specular), specular.data()); }
			inline void setIntensity(GLfloat intensity) { m_intensity = intensity; m_uniformBuffer.setUniform("Light.intensity", sizeof(intensity), &intensity); }
			inline void setFalloff(const Direction3D &dir, GLfloat falloffExponent, GLfloat cutoffCosine);
			void setCutoffCosine(GLfloat cosine);
			inline void setFalloffExponent(GLfloat exponent) { m_falloffExponent = exponent; m_uniformBuffer.setUniform("Light.falloff", sizeof(exponent), &exponent); }
			void setDirection(const Direction3D &direction);
			void setShadowMapResolution(GLsizei width = 512, GLsizei height = 512);
			void setType(const Type &type) { m_type = type; m_uniformBuffer.setUniform("Light.type", sizeof(m_type), &m_type); }

		private:
		// INTERNAL STATE =============================================================
			Texturing::Color3f m_ambient,
							   m_diffuse,
							   m_specular;

			GLfloat m_intensity,
					m_cosineCutoff,
					m_falloffExponent;

			Texturing::Texture2D::Ptr m_shadowMap;
			Buffers::Renderbuffer::Ptr m_depthBuffer;
			Shaders::Program m_shadowMapProgram;
			Cameras::QuaternionCamera m_lastShadowCamera;
			LightUniformBuffer m_uniformBuffer;
			Type m_type;

		// GLOBAL INTERNALS ===========================================================
			static Geometry::PointModel& getSpritePoint();
		};
		typedef boost::shared_ptr<Light> LightPtr;
	}
}