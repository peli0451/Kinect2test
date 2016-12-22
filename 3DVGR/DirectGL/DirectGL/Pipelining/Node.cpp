#include "stdafx.h"
#include "Node.h"

#include <DirectGL/DirectGL.h>
#include <boost/lexical_cast.hpp>

void DirectGL::Pipelining::Node::connectOutput(unsigned int output, const Ptr &other, unsigned int otherInput)
{
	if (output >= getNumOutputs())
		LOGEXCEPTION(("<DirectGL::Pipelining::Node::connectOutput> Unknown output of id <" + boost::lexical_cast<std::string>(output)+">!").c_str());
	
	if (m_prevNodes.find(other.get()) != m_prevNodes.end())
		LOGEXCEPTION("<DirectGL::Pipelining::Node::connectOutput> Circular connection detected!");
	
	// if this output is already connected to the specified target, we'll abort
	auto ex = m_postNodes[other.get()].find(output);
	if (ex != m_postNodes[other.get()].end())
		return;

	m_outputs[output][other.get()] = otherInput;
	m_postNodes[other.get()][output] = otherInput;

	other->connectInput(otherInput, Ptr(this, ptrNonDelete), output);

	if (checkCircularSelfReference())
	{
		disconnectOutput(other, output);
		LOGEXCEPTION("<DirectGL::Pipelining::Node::connectOutput> Circular connection detected!");
	}
}

void DirectGL::Pipelining::Node::connectInput(unsigned int input, const Ptr &other, unsigned int otherOutput)
{
	if (input >= getNumInputs())
		LOGEXCEPTION(("<DirectGL::Pipelining::Node::connectInput> Unknown input of id <" + boost::lexical_cast<std::string>(input) + ">!").c_str());
	
	if (m_postNodes.find(other.get()) != m_postNodes.end())
		LOGEXCEPTION("<DirectGL::Pipelining::Node::connectInput> Circular connection detected!");

	// if this input is already connected to the specified target, we'll abort
	auto ex = m_prevNodes[other.get()].find(input);
	if (ex != m_prevNodes[other.get()].end())
		return;

	m_inputs[input] = std::make_pair(other.get(), otherOutput);
	m_prevNodes[other.get()][input] = otherOutput;
	
	other->connectOutput(otherOutput, Ptr(this, ptrNonDelete), input);

	if (checkCircularSelfReference())
	{
		disconnectInput(input);
		LOGEXCEPTION("<DirectGL::Pipelining::Node::connectInput> Circular connection detected!");
	}

}

void DirectGL::Pipelining::Node::disconnect(Node *other)
{
	auto it = m_prevNodes.find(other);

	if (it != m_prevNodes.end())
	{
		auto size(it->second.size());
		for (auto i = 0; i < size; ++i)
			disconnectInput(it->second.begin()->first);
	}
	else
	{
		it = m_postNodes.find(other);
		if (it != m_postNodes.end())
		{
			auto size(it->second.size());
			for (auto i = 0; i < size; ++i)
				disconnectOutput(Ptr(other, ptrNonDelete), it->second.begin()->first);
		}
	}
}

void DirectGL::Pipelining::Node::disconnect(const Ptr &other)
{
	disconnect(other.get());
}

void DirectGL::Pipelining::Node::disconnectInput(unsigned int input)
{
	if (input >= getNumInputs())
		LOGEXCEPTION(("<DirectGL::Pipelining::Node::disconnectInput> Unknown input of id <" + boost::lexical_cast<std::string>(input)+">!").c_str());

	auto it = m_inputs.find(input);
	if (it != m_inputs.end())
	{
		auto curr(it->second);

		m_prevNodes[curr.first].erase(input);
		m_inputs.erase(it);

		if (m_prevNodes[curr.first].empty())
			m_prevNodes.erase(curr.first);

		curr.first->disconnectOutput(Ptr(this, ptrNonDelete), curr.second);
	}
}

void DirectGL::Pipelining::Node::disconnectOutput(const Ptr &other, unsigned int output)
{
	if (output >= getNumOutputs())
		LOGEXCEPTION(("<DirectGL::Pipelining::Node::disconnectOutput> Unknown output of id <" + boost::lexical_cast<std::string>(output)+">!").c_str());

	auto it = m_outputs.find(output);
	if (it != m_outputs.end())
	{
		auto &curr(it->second);
		auto f = curr.find(other.get());

		if (f != curr.end())
		{
			auto otherInput = m_postNodes[other.get()][output];
			curr.erase(other.get());
			m_postNodes[other.get()].erase(output);
			if (curr.empty())
				m_outputs.erase(it);
			if (m_postNodes[other.get()].empty())
				m_postNodes.erase(other.get());

			other->disconnectInput(otherInput);
		}
	}
}

void DirectGL::Pipelining::Node::execute()
{

}

void DirectGL::Pipelining::Node::eject()
{
	while (m_prevNodes.size())
		disconnect(m_prevNodes.begin()->first);
	while (m_postNodes.size())
		disconnect(m_postNodes.begin()->first);
}

bool DirectGL::Pipelining::Node::isPrevNode(Node *other, bool recursive) const
{
	bool retVal = false;
	for (auto i : m_prevNodes)
	{
		if (i.first == other) return true;
		else if(recursive) retVal |= i.first->isPrevNode(other);
	}

	return retVal;
}

bool DirectGL::Pipelining::Node::isPrevNode(const Ptr &other, bool recursive) const
{
	return isPrevNode(other.get());
}

bool DirectGL::Pipelining::Node::isPostNode(const Ptr &other, bool recursive) const
{
	bool retVal = false;
	for (auto i : m_postNodes)
	{
		if (i.first == other.get()) return true;
		else if(recursive) retVal |= i.first->isPostNode(other);
	}
	
	return retVal;
}

bool DirectGL::Pipelining::Node::checkCircularSelfReference()
{
	return isPrevNode(this);
}

void DirectGL::Pipelining::Node::ptrNonDelete(Ptr::element_type *ptr)
{
}