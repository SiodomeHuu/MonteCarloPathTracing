#pragma once

#include "objdef.h"
#include <vector>

namespace MCPT::BVH {


	struct CPU {};
	struct GPU {};


	class BVH {
	public:
		virtual ~BVH();
	};


	class CPUBVH : virtual public BVH {
	public:
		virtual const std::vector<BVHNode>& getBVH() = 0;
	};


	class GPUBVH : virtual public BVH {
	public:
		virtual std::pair< cl::Buffer, cl::Buffer > getBuffer() = 0; // [ bvhnode,object node]
	};

}