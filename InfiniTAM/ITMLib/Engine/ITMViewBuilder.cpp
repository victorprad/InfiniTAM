// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder.h"

using namespace ITMLib::Engine;
using namespace ITMLib::Objects;
using namespace ORUtils;

ITMViewBuilder::ITMViewBuilder(const ITMRGBDCalib *calib)
{
	this->calib = calib;
	this->shortImage = NULL;

	if (calib->disparityCalib.params == Vector2f(0.0f, 0.0f)) inputImageType = InfiniTAM_SHORT_DEPTH_IMAGE;
	else inputImageType = InfiniTAM_DISPARITY_IMAGE;
}

ITMViewBuilder::~ITMViewBuilder()
{
	if (this->shortImage != NULL) delete this->shortImage;
}