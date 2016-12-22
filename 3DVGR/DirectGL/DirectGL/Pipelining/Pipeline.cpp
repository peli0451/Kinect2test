#include "stdafx.h"
#include "Pipeline.h"

void DirectGL::Pipelining::Pipeline::addNode(const Node::Ptr &node, const PlacementHint &hint)
{
	switch (hint)
	{
	case PlacementHint::Front:
	{
		m_nodeList.insert(m_nodeList.begin(), node);
		auto it = m_nodeList.begin() + 1;
		while (it != m_nodeList.end() && !(*it)->isPrevNode(node, false))
			std::swap((*(it++)), *(it - 1));
		break;
	}
	case PlacementHint::Back:
	{
		m_nodeList.push_back(node);
		auto it = m_nodeList.rbegin() + 1;
		while (it != m_nodeList.rend() && !(*it)->isPostNode(node, false))
			std::swap(*(it++), *(it - 1));
		break;
	}
	}
}

void DirectGL::Pipelining::Pipeline::execute()
{
	for (auto i : m_nodeList)
		i->execute();
}