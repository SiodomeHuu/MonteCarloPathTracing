#pragma once

#include "oclbasic.h"
#include "objdef.h"
#include "config.h"

namespace MCPT::BVH::TEST {

	void test();

	void testall();

	float SAH(const std::vector<BVHNode>& node);
	float EPO(const std::vector<BVHNode>& node, const std::vector <Triangle> & triangles);
	float LCV(const std::vector<BVHNode>& node, const Camera& camera);

	float SAH(const std::vector<QuadBVHNode>& node, int objCount);
	//float EPO(const std::vector<QuadBVHNode>& node, const std::vector <Triangle>& triangles);
}