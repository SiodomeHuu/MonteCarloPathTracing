#pragma once
#include "oclbasic.h"
#include <memory>

namespace MCPT::RayGeneration {

	struct RayBase {
		virtual ~RayBase() = default;
	};

	struct RayCL : public RayBase {
		cl::Buffer rayBuffer;
	};
	
	RayBase* generateRay();

	void init();
}