#pragma once

#include <map>
#include <boost/smart_ptr.hpp>
#include <DirectGL/Utility/Logging.h>

#define InputEnum(...) struct Inputs { enum {__VA_ARGS__ MAX}; }; \
						std::size_t getNumInputs() const { return Inputs::MAX; }
#define OutputEnum(...) struct Outputs { enum {__VA_ARGS__ MAX}; }; \
						std::size_t getNumOutputs() const { return Outputs::MAX; }

namespace DirectGL
{
	namespace Pipelining
	{
		class Node
		{
		public:
			class Output
			{
			public:
				using Ptr = boost::shared_ptr<Output>;
			private:
				virtual void dummy() {}
			};

			using Ptr = boost::shared_ptr<Node>;

			virtual ~Node() {}

			virtual void connectOutput(unsigned int output, const Ptr &other, unsigned int otherInput);
			virtual void connectInput(unsigned int input, const Ptr &other, unsigned int otherOutput);
			virtual void disconnect(const Ptr &other);
			virtual void disconnectInput(unsigned int input);
			virtual void disconnectOutput(const Ptr &other, unsigned int output);

			virtual void execute();
			virtual void eject();

			virtual Output::Ptr getOutput(unsigned int index) const { if (index >= getNumOutputs()) LOGEXCEPTION("<DirectGL::Pipelining::Node::getOutput> Output index out of bounds!"); return boost::shared_ptr<Output>(nullptr); }
			virtual std::size_t getNumInputs() const = 0;
			virtual std::size_t getNumOutputs() const = 0;

			bool isPrevNode(const Ptr &other, bool recursive = true) const;
			bool isPostNode(const Ptr &other, bool recursive = true) const;

		private:
			void disconnect(Node *other);
			bool isPrevNode(Node *other, bool recursive = true) const;
			static void ptrNonDelete(Ptr::element_type *ptr);

			bool checkCircularSelfReference();
			using NodeMap = std::map<Node*, std::map<unsigned int, unsigned int>>;
			using InputMap = std::map<unsigned int, std::pair<Node*, unsigned int>>;
			using OutputMap = std::map<unsigned int, std::map<Node*, unsigned int>>;
			NodeMap m_prevNodes, m_postNodes;
			InputMap m_inputs;
			OutputMap m_outputs;
		};
	}
}