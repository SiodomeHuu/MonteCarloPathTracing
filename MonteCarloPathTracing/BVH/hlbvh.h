#pragma once

#include "bvh.h"

namespace MCPT::BVH {

	template<class T>
	class HLBVH {};

	template<>
	class HLBVH<CPU> : public CPUBVH {
	public:
		HLBVH(const std::vector<Triangle>& triangles);
		HLBVH(std::vector<Triangle>&& triangles);

		virtual const std::vector<BVHNode>& getBVH() override;

	private:
		void build(std::vector<Triangle>&& triangles);

		std::vector<Triangle> triangles;
		std::vector<BVHNode> finalTree;
	};


}

