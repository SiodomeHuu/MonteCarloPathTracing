#pragma once

#include "bvh.h"
#include <memory>

namespace MCPT::BVH {

	template<class T>
	class TreeletBVH {};

	template<class T>
	class TreeletBVH_ {};

	template<>
	class TreeletBVH<CPU> : public CPUBVH {
	public:
		TreeletBVH(std::vector<BVHNode> bvh);

		// need to be modified in the future
		// when new type of built-BVH is used as input 
		TreeletBVH(std::unique_ptr<CPUBVH> pBVH);

		virtual const std::vector<BVHNode>& getBVH() override;
		virtual std::vector<BVHNode>&& releaseBVH() override;

	private:
		std::vector< BVHNode > bvhnode;

		struct Impl;
		std::unique_ptr<Impl> pImpl;

		friend class TreeletBVH_<CPU>;
	};



	template<>
	class TreeletBVH<GPU> : public GPUBVH {
	public:
		TreeletBVH(std::pair<cl::Buffer, cl::Buffer> bvh);
		TreeletBVH(cl::Buffer node, cl::Buffer tri);

		virtual std::pair< cl::Buffer, cl::Buffer > getBuffer() override;

	private:
		cl::Buffer bvhNode;
		cl::Buffer objectNode;
	};

	
	template<>
	class TreeletBVH_<CPU> : public CPUBVH {
	public:
		TreeletBVH_(const std::vector<BVHNode>& bvh);
		TreeletBVH_(std::vector<BVHNode>&& bvh);

		virtual const std::vector<BVHNode>& getBVH() override { return bvhNode; }
		virtual std::vector<BVHNode>&& releaseBVH() override { return std::move(bvhNode); }

		virtual const std::vector<unsigned int>& getIndices() override { return indices; }
		virtual std::vector<unsigned int>&& releaseIndices() override { return std::move(indices); }

	private:
		std::vector< MultiPrimBVHNode > bvhNode;
		std::vector<unsigned int> indices;
	};
	

}