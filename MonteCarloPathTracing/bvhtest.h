#pragma once

#include "oclbasic.h"
#include "objdef.h"
#include "config.h"

namespace MCPT::BVH::TEST {

	void testall();

	double SAH(const std::vector<BVHNode>& node);
	double SAH_Multi(const std::vector<MultiPrimBVHNode>& node);
	double SAH(cl::Buffer nodeB);
	double EPO(cl::Buffer bvh, cl::Buffer triangles);

	double EPO_Multi(const std::vector<MultiPrimBVHNode>& node, const std::vector<unsigned int>& indices, cl::Buffer triangles);

	double LCV(const std::vector<BVHNode>& node, const Camera& camera);
	


	double SAH(const std::vector<QuadBVHNode>& node, int objCount);
	
	double QuadEPO(cl::Buffer bvh, cl::Buffer triangles);

	void singleTest(cl::Buffer bvh, cl::Buffer triangles);
	void singleTest(const std::vector<MultiPrimBVHNode>& node, const std::vector<unsigned int>& indices, cl::Buffer triangles);

	[[deprecated]]
	double EPO(const std::vector<BVHNode>& node, const std::vector<Triangle>& triangles);
}