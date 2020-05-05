#pragma once

#include <vector>
#include "objdef.h"
#include "bvh.h"

namespace MCPT::BVH {

	template<class T>
	class SimpleBVHQuad {};

	template<>
	class SimpleBVHQuad<CPU> : public CPUBVH {
	public:
		SimpleBVHQuad(const std::vector<BVHNode>& bvh);

		virtual const std::vector<BVHNode>& getBVH() override;
		virtual std::vector<BVHNode>&& releaseBVH() override;

		const std::vector< QuadBVHNode >& getQuadBVH();
		int getObjectCount();

	private:
		std::vector< QuadBVHNode > bvhnode;
		int objCount;
	};


	/*
	template<>
	class SimpleBVHQuad<GPU> : public GPUBVH {
	public:
		SimpleBVHQuad(std::pair<cl::Buffer, cl::Buffer> bvh);

		virtual std::pair< cl::Buffer, cl::Buffer > getBuffer() override;

	private:
		cl::Buffer bvhNode;
		cl::Buffer objectNode;
	};
	*/

}