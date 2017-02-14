// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>
#include <vector>

#include "GraphNode.h"
#include "GraphEdge.h"
#include "ParameterIndex.h"
#include "VariableLengthVector.h"
#include "SparseBlockMatrix.h"

namespace MiniSlamGraph
{
	class SlamGraph
	{
	public:
		typedef std::map<int, GraphNode *> NodeIndex;
		typedef std::vector<GraphEdge *> EdgeList;

		static NodeIndex cloneNodeIndex(const NodeIndex & src);
		static void clearNodeIndex(NodeIndex & src);

		virtual ~SlamGraph(void);

		void addNode(GraphNode *node);
		void addEdge(GraphEdge *edge);

		const NodeIndex & getNodeIndex(void) const { return mNodes; }
		void setNodeIndex(const NodeIndex & src);

		/** Before any calls to evaluateF() or related functions, the
			evaluations have to be initialized with prepareEvaluations().
			This will internally assign the parameters of all nodes to places
			in the gradient vector and hessian matrix.
		*/
		void prepareEvaluations(void);
		const ParameterIndex & getParameters(void) const
		{
			return mParameterIndex;
		}

		virtual double evaluateF(const NodeIndex *override_nodes = NULL) const;
		virtual void evaluateGradientAndHessian(VariableLengthVector* & g, SparseBlockMatrix* & H, const NodeIndex *override_nodes = NULL) const;

	protected:
		/** This function is internally called by evaluateGradientAndHessian()
			and is supposed to allocate the structures for the gradient vector
			and, most crucially, the sparse Hessian matrix.
		*/
		virtual void allocateGradientAndHessian(VariableLengthVector* & g, SparseBlockMatrix* & H) const = 0;

	private:
		NodeIndex mNodes;
		EdgeList mEdges;

		ParameterIndex mParameterIndex;
	};
}

