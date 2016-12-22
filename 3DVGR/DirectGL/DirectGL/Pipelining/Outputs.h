#pragma once

#include "Node.h"
#include <DirectGL/Buffers/GBuffer.h>

namespace DirectGL
{
	namespace Pipelining
	{
		class GBufferOutput : public Node::Output
		{
		public:
			using Ptr = boost::shared_ptr<GBufferOutput>;
			using GBufferType = DirectGL::Buffers::GBuffer<DirectGL::Texturing::Texture2D, GL_RGBA, GL_RGB32F, GL_RGB32F, GL_RGBA>;
			GBufferType m_gBuffer;
		};
	}
}