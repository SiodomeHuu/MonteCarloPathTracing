#pragma once

#include "objdef.h"
#include <vector>

namespace MCPT::BVH {


	struct CPU {};
	struct GPU {};


	class BVH {
	public:
		virtual ~BVH();
		int primitiveCount = 0;
	};


	class CPUBVH : virtual public BVH {
	public:
		virtual const std::vector<BVHNode>& getBVH() = 0;
		virtual std::vector<BVHNode>&& releaseBVH() = 0;
		
		virtual const std::vector<unsigned int>& getIndices() { throw "Not Implemented"; }
		virtual std::vector<unsigned int>&& releaseIndices() { throw "Not Implemented"; }
	};
	

	class GPUBVH : virtual public BVH {
	public:
		virtual std::pair< cl::Buffer, cl::Buffer > getBuffer() = 0; // [ bvhnode,object node]
	};

	using Indices = std::vector<uint32_t>;
	
	using BVHNodes = std::vector< BVHNode >;
	using MultiPrimBVHNodes = std::vector< MultiPrimBVHNode >;

	using CRefBVHNodesPair = std::pair< const BVHNodes&, const Indices& >;
	using BVHNodesPair = std::pair< BVHNodes, Indices >;

	using CRefMultiPrimBVHNodesPair = std::pair< const MultiPrimBVHNodes&, const Indices& >;
	using MultiPrimBVHNodesPair = std::pair< MultiPrimBVHNodes, Indices >;

}