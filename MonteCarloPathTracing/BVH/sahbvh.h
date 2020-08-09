#pragma once

#include "bvh.h"

#include <any>

namespace MCPT::BVH {
	template<class T>
	class SAHBVH {};

	/*
	template<>
	class SAHBVH<CPU> : public CPUBVH {
	public:
		SAHBVH(const std::vector<Triangle>& triangles);
		SAHBVH(std::vector<Triangle>&& triangles);

		virtual const std::vector<BVHNode>& getBVH() override;
		virtual std::vector<BVHNode>&& releaseBVH() override;

	private:
		std::vector< BVHNode > bvhnode;
	};
	*/

	using BBox = BoundingBox;

	template<>
	class SAHBVH<CPU> : public CPUBVH {
	public:
		/*
			arg: pair<>
				bool: true -> three axis ; false -> one widest axis
				uint32_t: > 1 -> use binning, and indicates count of bins ; 0 or 1 -> not
				int: max primitives count in a leaf; <=0 -> no constraint
		*/
		SAHBVH(
			const std::vector<BBox>& bboxes, const std::vector<uint32_t>& indices_,
			const std::any& arg = std::tuple<bool, uint32_t, int>({ false, 0, 16 })
		);

		std::vector<uint32_t> getLeafIDs() const;

		virtual const std::vector<BVHNode>& getBVH() override { return nodes; }
		virtual std::vector<BVHNode>&& releaseBVH() override { return std::move(nodes); }

		virtual const std::vector<unsigned int>& getIndices() override { return indices; }
		virtual std::vector<unsigned int>&& releaseIndices() override { return std::move(indices); }


		uint32_t primitiveCount() const { return indices.size(); }

	private:
		MultiPrimBVHNodes nodes;
		Indices indices;
	};
	

}