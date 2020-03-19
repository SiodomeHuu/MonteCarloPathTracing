#pragma once

#include "bvh.h"

namespace MCPT::BVH {

	template<class T>
	class TreeletBVH {};

	template<>
	class TreeletBVH<CPU> : public CPUBVH {
	public:
		TreeletBVH(const std::vector<BVHNode>& bvh);
		TreeletBVH(std::vector<BVHNode>&& bvh);

		virtual const std::vector<BVHNode>& getBVH() override;

	private:
		std::vector< BVHNode > bvhnode;
	};

	

}