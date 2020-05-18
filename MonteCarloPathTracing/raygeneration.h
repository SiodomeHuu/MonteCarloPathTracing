#pragma once
#include "oclbasic.h"
#include <memory>
#include "objdef.h"

namespace MCPT::RayGeneration {

	struct RayBase {
		virtual ~RayBase() = default;
	};

	struct RayCL : public RayBase {
		cl::Buffer rayBuffer;
	};
	
	RayBase* generateRay();
	void resetCamera(Camera cmr);

	void init();
}