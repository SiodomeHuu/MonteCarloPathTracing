#pragma once

#include "bvh/bvh.h"
#include "auxiliary.h"
#include <any>

namespace MCPT::BVH {
	
	using namespace Auxiliary;

	
	class TreeletTemp : public CPUBVH {
	public:
		/*
			arg: int
			|- max prims in leaf; <=0 stands for no limitation
		*/
		TreeletTemp(
			std::vector<BVHNode> bvh,
			const std::vector<uint32_t>& indices_,
			/* not used */ const std::any& arg = 32);

		TreeletTemp(const TreeletTemp& ano);
		TreeletTemp(TreeletTemp&& ano) noexcept;
		~TreeletTemp();

		
		float sah();
		


		virtual const std::vector<BVHNode>& getBVH() override { return bvhNode; }
		virtual std::vector<BVHNode>&& releaseBVH() override { return std::move(bvhNode); }

		virtual const std::vector<unsigned int>& getIndices() override { return indices; }
		virtual std::vector<unsigned int>&& releaseIndices() override { return std::move(indices); }

		uint32_t primitiveCount() const { return indices.size(); }

		std::vector<uint32_t> getLeafIDs() const;

	private:
		struct Impl;
		std::unique_ptr<Impl> pImpl;

		std::vector<MultiPrimBVHNode> bvhNode;
		std::vector<uint32_t> indices;
	};


}