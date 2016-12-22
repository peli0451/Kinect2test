#pragma once

#include <DirectGL/Texturing/Color.h>
#include <DirectGL/Texturing/Texture2D.h>
#include <DirectGL/Utility/OBJFile.h>
#include <DirectGL/Shaders/Program.h>
#include <DirectGL/Buffers/UniformBuffer.h>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <algorithm>

namespace DirectGL
{
	namespace Texturing
	{
		class Material
		{
		public:
			Material(const std::string &name = std::string(""), const Color3f &ambient = Color3f::Zero(), const Color3f &diffuse = Color3f::Ones(), 
					 const Color3f &specular = Color3f::Zero(), const Color3f &emissive = Color3f::Zero(), 
					 float shininess = 1.f, float transparency = 1.f, float refraction = 1.f) : m_name(name),
					 m_ambient(ambient), m_diffuse(diffuse), m_specular(specular), m_emissive(emissive),
					 m_shininess(std::max(0.f, shininess)), m_transparency(std::max(0.f, std::min(1.f, transparency))), m_refraction(refraction), m_mirror(false), m_flipTexture(false) {} // TODO remove texture functions
			Material(const Utility::MTLFile::Material &material) { fromMTLMaterial(material); }

			void fromFileStream(std::ifstream &stream);
			void fromMTLMaterial(const Utility::MTLFile::Material &material);

			void toFileStream(std::ofstream &stream);

#define SET_IF_VALID(name, size, value) if(m_uniformBuffer.isValid()) m_uniformBuffer.setUniform(name, size, value)
#define SET_IF_VALID_TEX(name) const bool use(texture != nullptr); if(m_uniformBuffer.isValid()) m_uniformBuffer.setUniform(name, sizeof(use), &use)
			
			inline void setAmbient(const Color3f &ambient) { m_ambient = ambient; SET_IF_VALID("Material.ambient", sizeof(ambient), ambient.data()); }
			inline void setDiffuse(const Color3f &diffuse) { m_diffuse = diffuse; SET_IF_VALID("Material.diffuse", sizeof(diffuse), diffuse.data()); }
			inline void setSpecular(const Color3f &specular) { m_specular = specular; SET_IF_VALID("Material.specular", sizeof(specular), specular.data()); }
			inline void setEmissive(const Color3f &emissive) { m_emissive = emissive; SET_IF_VALID("Material.emissive", sizeof(emissive), emissive.data()); }
			inline void setShininess(float shininess){ m_shininess = std::max(0.f, shininess); SET_IF_VALID("Material.shininess", sizeof(m_shininess), &m_shininess); }
			inline void setTransparency(float transparency) { m_transparency = std::max(0.f, std::min(1.f, transparency)); SET_IF_VALID("Material.transparency", sizeof(m_transparency), &m_transparency); }
			inline void setRefraction(float refraction) { m_refraction = refraction; } // TODO what bounds to use
			inline void setName(const std::string &name) { m_name = name; }
			inline void setAmbientTexture(const Texture2DPtr &texture) { m_ambientTex = texture;  SET_IF_VALID_TEX("Material.useAmbientTexture"); }
			inline void setDiffuseTexture(const Texture2DPtr &texture) { m_diffuseTex = texture; SET_IF_VALID_TEX("Material.useDiffuseTexture"); }
			inline void setSpecularTexture(const Texture2DPtr &texture) { m_specularTex = texture; SET_IF_VALID_TEX("Material.useSpecularTexture"); }
			inline void setTransparencyTexture(const Texture2DPtr &texture) { m_transparencyTex = texture; SET_IF_VALID_TEX("Material.useTransparencyTexture"); }
			inline void setBumpTexture(const Texture2DPtr &texture) { m_bumpTex = texture; SET_IF_VALID_TEX("Material.useBumpTexture"); }
			inline void setMirrorMaterial(bool isMirror) { m_mirror = isMirror; }
			inline void setTextureFlipped(bool flip) { m_flipTexture = flip; SET_IF_VALID("Material.flipTexture", sizeof(flip), &flip); }

#undef SET_IF_VALID_TEX
#undef SET_IF_VALID

			inline const Color3f &getAmbient() const { return m_ambient; }
			inline const Color3f &getDiffuse() const { return m_diffuse; }
			inline const Color3f &getSpecular() const { return m_specular; }
			inline const Color3f &getEmissive() const { return m_emissive; }
			inline float getShininess() const { return m_shininess; }
			inline float getTransparency() const { return m_transparency; }
			inline float getRefraction() const { return m_refraction; }
			inline const std::string &getName() const { return m_name; }
			inline const Texture2DPtr getAmbientTexture() const { return m_ambientTex; }
			inline const Texture2DPtr getDiffuseTexture() const { return m_diffuseTex; }
			inline const Texture2DPtr getSpecularTexture() const { return m_specularTex; }
			inline const Texture2DPtr getTransparencyTexture() const { return m_transparencyTex; }
			inline const Texture2DPtr getBumpTexture() const { return m_bumpTex; }
			inline bool isMirrorMaterial() const { return m_mirror; }
			inline bool isTextureFlipped() const { return m_flipTexture; }

			void bindToProgram(Shaders::Program &program);
		
		private:
			Color3f m_ambient,
					m_diffuse,
					m_specular,
					m_emissive;

			GLfloat m_shininess,
					m_transparency,
					m_refraction;

			std::string m_name;

			Texture2DPtr m_ambientTex,
						 m_diffuseTex,
						 m_specularTex,
						 m_transparencyTex,
						 m_bumpTex;

			bool m_mirror;
			bool m_flipTexture;

			Buffers::UniformBuffer m_uniformBuffer;
		};
		typedef boost::shared_ptr<Material> MaterialPtr;
	}
}