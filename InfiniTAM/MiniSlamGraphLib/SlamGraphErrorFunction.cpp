// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "SlamGraphErrorFunction.h"

//#define USELIB_CSPARSE

#ifdef COMPILE_WITH_CSPARSE
#include "Matrix_CSparse.h"
#endif

//#define DEBUG_DERIVATIVES
#ifdef DEBUG_DERIVATIVES
#include <TooN/functions/derivatives.h>
class ComposeWrapperErrorFunction {
public:
	ComposeWrapperErrorFunction(const K_OPTIM::ErrorFunction & function, const K_OPTIM::OptimizationParameter & pos)
		: basefun(function), basepos(pos)
	{}

	double operator()(const TooN::Vector<> & pos) const
	{
		K_OPTIM::OptimizationParameter *newpos = basepos.clone();
		basefun.applyDelta(basepos, pos.get_data_ptr(), *newpos);

		K_OPTIM::ErrorFunction::EvaluationPoint *tmpPt = basefun.evaluateAt(newpos);
		double ret = tmpPt->f();
		delete tmpPt;

		return ret;
	}
private:
	const K_OPTIM::ErrorFunction & basefun;
	const K_OPTIM::OptimizationParameter & basepos;
};

void debugDerivatives(const double *computed_grad, const K_OPTIM::ErrorFunction & function, const K_OPTIM::OptimizationParameter & pos)
{
	TooN::Vector<> tmppos(function.numParameters());
	tmppos = TooN::Zeros;
	TooN::Matrix<TooN::Dynamic, 2> mat = TooN::numerical_gradient_with_errors(ComposeWrapperErrorFunction(function, pos), tmppos);
	for (int i = 0; i < mat.num_rows(); ++i) {
		fprintf(stderr, "%i: %e (%f) <=> %e%s\n", i, mat(i, 0), mat(i, 1), computed_grad[i], (fabs(mat(i, 0) - computed_grad[i]) > 1e-4) ? " !!!" : "");
	}
}
#endif

using namespace MiniSlamGraph;

SlamGraphErrorFunction::Parameters::Parameters(const SlamGraph & graph)
{
	mNodes = SlamGraph::cloneNodeIndex(graph.getNodeIndex());
}

SlamGraphErrorFunction::Parameters::Parameters(const Parameters & src)
{
	mNodes = SlamGraph::cloneNodeIndex(src.mNodes);
}

SlamGraphErrorFunction::Parameters::~Parameters(void)
{
	clear();
}

void SlamGraphErrorFunction::Parameters::copyFrom(const /*Optimization*/Parameters & _src)
{
	const Parameters & src = (const Parameters &)_src;
	clear();
	mNodes = SlamGraph::cloneNodeIndex(src.mNodes);
}

//void copyValuesFrom(const OptimizationParameter & src);
void SlamGraphErrorFunction::Parameters::clear(void)
{
	SlamGraph::clearNodeIndex(mNodes);
}

SlamGraphErrorFunction::EvaluationPoint::EvaluationPoint(const SlamGraphErrorFunction *parent, Parameters *para)
{
	mParent = parent;
	mPara = para;
	cacheF = parent->getGraph()->evaluateF(&(mPara->getNodes()));
	cacheG = NULL;
	cacheH = NULL;
}

SlamGraphErrorFunction::EvaluationPoint::~EvaluationPoint(void)
{
	delete mPara;
	if (cacheG != NULL) delete cacheG;
	if (cacheH != NULL) delete cacheH;
}

double SlamGraphErrorFunction::EvaluationPoint::f(void)
{
	return cacheF;
}

const double* SlamGraphErrorFunction::EvaluationPoint::nabla_f(void)
{
	cacheGH();
	return cacheG->getData();
}

const Matrix* SlamGraphErrorFunction::EvaluationPoint::hessian_GN(void)
{
	cacheGH();
	return cacheH;
}

void SlamGraphErrorFunction::EvaluationPoint::cacheGH(void)
{
	if (cacheG != NULL) return;
	SparseBlockMatrix *H_tmp = NULL;
	mParent->getGraph()->evaluateGradientAndHessian(cacheG, H_tmp, &(mPara->getNodes()));

#ifdef COMPILE_WITH_CSPARSE
	cacheH = new Matrix_CSparse(*H_tmp, (Matrix_CSparse::Pattern*&)(const_cast<SlamGraphErrorFunction*>(mParent)->getHessianSparsityPattern()));
#else
	MatrixSymPosDef *H = new MatrixSymPosDef(cacheG->getOverallSize());
	cacheH = H;
	for (int i = 0; i < H->numRows()*H->numCols(); ++i) H->getMemory()[i] = 0.0f;
	H_tmp->densify(H->getMemory(), H->numCols());
#endif
	delete H_tmp;

#ifdef DEBUG_DERIVATIVES
	static int counter = 0;
	static bool inDebugDerivatives = false;
	if (inDebugDerivatives) return;
	if (counter++ >= 0) {
		inDebugDerivatives = true;
		int numPara = cacheH->numRows();
		fprintf(stderr, "debugDerivatives for %i parameters\n", numPara);//mParent->numParameters());
		debugDerivatives(cacheG->linearize(), *mParent, *mPara);
		fprintf(stderr, "debugDerivatives done\n");
		inDebugDerivatives = false;
	}
#endif
}

SlamGraphErrorFunction::SlamGraphErrorFunction(const SlamGraph & graph)
{
	mGraph = &graph;
	mSparsityPattern = NULL;
}

SlamGraphErrorFunction::~SlamGraphErrorFunction(void)
{
#ifdef COMPILE_WITH_CSPARSE
	if (mSparsityPattern) Matrix_CSparse::freePattern((Matrix_CSparse::Pattern*)mSparsityPattern);
#endif
}

int SlamGraphErrorFunction::numParameters(void) const
{
	return mGraph->getParameters().numTotalParameters();
}

SlamGraphErrorFunction::EvaluationPoint* SlamGraphErrorFunction::evaluateAt(/*K_OPTIM::Optimization*/Parameters *para) const
{
	return new EvaluationPoint(this, (Parameters*)para);
}

void SlamGraphErrorFunction::applyDelta(const /*K_OPTIM::Optimization*/Parameters & para_old, const double *delta, /*K_OPTIM::Optimization*/Parameters & para_new) const
{
	const SlamGraph::NodeIndex & list_old = ((const Parameters &)para_old).getNodes();
	SlamGraph::NodeIndex & list_new = ((Parameters &)para_new).getNodes();
	const ParameterIndex & paraIndex = mGraph->getParameters();

	SlamGraph::NodeIndex::const_iterator old_it = list_old.begin();
	SlamGraph::NodeIndex::iterator new_it = list_new.begin();
	for (; new_it != list_new.end(); ++new_it, ++old_it) {
		int idx = paraIndex.findIndex(new_it->first);
		if (idx < 0) continue;
		new_it->second->applyDelta(&(delta[idx]), old_it->second);
	}
}