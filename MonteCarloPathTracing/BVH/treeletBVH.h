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
		virtual std::vector<BVHNode>&& releaseBVH() override;

	private:
		std::vector< BVHNode > bvhnode;
	};



	template<>
	class TreeletBVH<GPU> : public GPUBVH {
	public:
		TreeletBVH(std::pair<cl::Buffer, cl::Buffer> bvh);

		virtual std::pair< cl::Buffer, cl::Buffer > getBuffer() override;

	private:
		cl::Buffer bvhNode;
		cl::Buffer objectNode;
	};
	

}