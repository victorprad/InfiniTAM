// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../CPU/ITMLowLevelEngine_CPU.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMLowLevelEngine_Metal : public ITMLowLevelEngine_CPU
		{
		public:
			ITMLowLevelEngine_Metal(void);
			~ITMLowLevelEngine_Metal(void);
		};
	}
}

#endif