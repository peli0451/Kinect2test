#pragma once

#include "Node.h"

#include <vector>

namespace DirectGL
{
	namespace Pipelining
	{
		class Pipeline
		{
		public:
			using NodeList = std::vector<Node::Ptr>;
			enum PlacementHint
			{
				Front,
				Back
			};
			
			void addNode(const Node::Ptr &node, const PlacementHint &hint);

			void execute();
			void clear() { m_nodeList.clear(); }

			const NodeList &getNodeList() const { return m_nodeList; }

		private:
			NodeList m_nodeList;

		};
	}
}