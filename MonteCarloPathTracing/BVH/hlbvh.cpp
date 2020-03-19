#include "hlbvh.h"

#include <deque>

using namespace MCPT;
using namespace MCPT::BVH;


namespace {


	inline uint32_t LeftShift3(uint32_t x) {
		if (x == (1 << 10)) --x;
		x = (x | (x << 16)) & 0b00000011000000000000000011111111;
		// x = ---- --98 ---- ---- ---- ---- 7654 3210
		x = (x | (x << 8)) & 0b00000011000000001111000000001111;
		// x = ---- --98 ---- ---- 7654 ---- ---- 3210
		x = (x | (x << 4)) & 0b00000011000011000011000011000011;
		// x = ---- --98 ---- 76-- --54 ---- 32-- --10
		x = (x | (x << 2)) & 0b00001001001001001001001001001001;
		// x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
		return x;
	}
	inline uint32_t EncodeMorton3(const cl_uint4& v) {
		return (LeftShift3(v.z) << 2) | (LeftShift3(v.y) << 1) | LeftShift3(v.x);
	}
	static void radixSort(std::vector<MortonPrimitive>* v) {
		std::vector<MortonPrimitive> tempVector(v->size());
		constexpr int bitsPerPass = 6;
		constexpr int nBits = 30;
		constexpr int nPasses = nBits / bitsPerPass;
		for (int pass = 0; pass < nPasses; ++pass) {
			// Perform one pass of radix sort, sorting _bitsPerPass_ bits
			int lowBit = pass * bitsPerPass;

			// Set in and out vector pointers for radix sort pass
			std::vector<MortonPrimitive>& in = (pass & 1) ? tempVector : *v;
			std::vector<MortonPrimitive>& out = (pass & 1) ? *v : tempVector;

			// Count number of zero bits in array for current radix sort bit
			constexpr int nBuckets = 1 << bitsPerPass;
			int bucketCount[nBuckets] = { 0 };
			constexpr int bitMask = (1 << bitsPerPass) - 1;
			for (const MortonPrimitive& mp : in) {
				int bucket = (mp.mortonCode >> lowBit)& bitMask;
				++bucketCount[bucket];
			}

			// Compute starting index in output array for each bucket
			int outIndex[nBuckets];
			outIndex[0] = 0;
			for (int i = 1; i < nBuckets; ++i)
				outIndex[i] = outIndex[i - 1] + bucketCount[i - 1];

			// Store sorted values in output array
			for (const MortonPrimitive& mp : in) {
				int bucket = (mp.mortonCode >> lowBit)& bitMask;
				out[outIndex[bucket]++] = mp;
			}
		}
		// Copy final result from _tempVector_, if needed
		if (nPasses & 1) std::swap(*v, tempVector);
	}
	BoundingBox recursiveBuildNode(std::vector<BVHNode>& finalTree, size_t id) {
		if (finalTree[id].left == finalTree[id].right) return BoundingBox({ finalTree[id].bbmin,finalTree[id].bbmax });
		else {
			auto leftBox = recursiveBuildNode(finalTree, finalTree[id].left);
			auto rightBox = recursiveBuildNode(finalTree, finalTree[id].right);
			BoundingBox ans;
			ans.bbmin = min(leftBox.bbmin, rightBox.bbmin);
			ans.bbmax = max(leftBox.bbmax, rightBox.bbmax);
			finalTree[id].bbmin = ans.bbmin;
			finalTree[id].bbmax = ans.bbmax;
			return ans;
		}
	}


}



HLBVH<CPU>::HLBVH<CPU>(const std::vector<Triangle>& triangles) {
	auto t = triangles;
	build(std::move(t));
}

HLBVH<CPU>::HLBVH<CPU>(std::vector<Triangle>&& triangles) {
	build(std::move(triangles));
}

void HLBVH<CPU>::build(std::vector<Triangle>&& triangles) {
	std::vector<BoundingBox> bbs;
	std::vector<float4> centroids;

	auto buildHLBVH = [&]() -> void {
		for (auto& tr : triangles) {
			bbs.push_back({
				min(min(tr.v[0],tr.v[1]),tr.v[2]),
				max(max(tr.v[0],tr.v[1]),tr.v[2])
				});
			auto& bbback = bbs.back();
			centroids.push_back(
				0.5f * (bbback.bbmin + bbback.bbmax)
			);
		}
		BoundingBox globalBox = {
			{FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX},
			{-FLT_MAX,-FLT_MAX,-FLT_MAX,-FLT_MAX}
		};
		for (auto& centroid : centroids) {
			globalBox.bbmin = min(globalBox.bbmin, centroid);
			globalBox.bbmax = max(globalBox.bbmax, centroid);
		}
		std::vector< MortonPrimitive > mortonPrims;
		size_t id = 0;

		cl_float4 globalBoxSize = globalBox.bbmax - globalBox.bbmin;

		for (auto& centroid : centroids) {
			cl_float4 data = centroid - globalBox.bbmin;
			for (int i = 0; i < 3; ++i) {
				data.s[i] /= globalBoxSize.s[i];
				data.s[i] *= 1024.0f;
			}
			cl_uint3 inputData;
			inputData.x = (cl_uint)round(data.x);
			inputData.y = (cl_uint)round(data.y);
			inputData.z = (cl_uint)round(data.z);

			mortonPrims.push_back({
				(int)id,
				(int)EncodeMorton3(inputData)
				});
			++id;
		}
		radixSort(&mortonPrims);
		auto CLZ = [](int a) -> size_t {
			size_t ans = 0;
			if (a == 0) return 32;
			while (a > 0) {
				a = a << 1;
				++ans;
			}
			return ans;
		};

		auto DELTA = [CLZ](std::vector<MortonPrimitive>& mortonPrims, size_t left, size_t right) -> size_t {
			return CLZ((int)(mortonPrims[left].mortonCode ^ mortonPrims[right].mortonCode));
		};

		auto findSplit = [DELTA](std::vector<MortonPrimitive>& mortonPrims, size_t left, size_t right)  -> size_t {
			int target = DELTA(mortonPrims, left, right);
			if (target == 32) return (right + left) >> 1;
			do {
				int mid = (right + left) >> 1;
				if (DELTA(mortonPrims, left, mid) > target) left = mid;
				else right = mid;
			} while (right > left + 1);
			return left;
		};


		finalTree.resize((mortonPrims.size() << 1) - 1);
		std::deque< cl_uint3 > ranges;
		ranges.push_back({ 0, (cl_uint)mortonPrims.size() - 1 });

		finalTree[0].parent = -1;
		while (!ranges.empty()) {
			auto range = ranges.front();
			ranges.pop_front();

			size_t splitPos = findSplit(mortonPrims, range.s[0], range.s[1]);

			size_t leftIDX = (splitPos != range.s[0]) ? (splitPos) : (splitPos + mortonPrims.size() - 1);
			size_t rightIDX = (splitPos + 1 != range.s[1]) ? (splitPos + 1) : (splitPos + mortonPrims.size());

			finalTree[range.s[2]].left = leftIDX;
			finalTree[leftIDX].parent = range.s[2];
			finalTree[range.s[2]].right = rightIDX;
			finalTree[rightIDX].parent = range.s[2];

			if (leftIDX == splitPos)
				ranges.push_back({ (cl_uint)range.s[0],(cl_uint)splitPos,(cl_uint)splitPos });
			if (rightIDX == splitPos + 1)
				ranges.push_back({ (cl_uint)splitPos + 1,(cl_uint)range.s[1],(cl_uint)splitPos + 1 });

		}
		for (size_t i = mortonPrims.size() - 1; i < finalTree.size(); ++i) {
			finalTree[i].left = finalTree[i].right = mortonPrims[i + 1 - mortonPrims.size()].id;
			finalTree[i].bbmin = bbs[finalTree[i].left].bbmin;
			finalTree[i].bbmax = bbs[finalTree[i].left].bbmax;
		}
		recursiveBuildNode(finalTree, 0);
	};
	buildHLBVH();

	this->triangles.swap(triangles);

}

const std::vector<BVHNode>& MCPT::BVH::HLBVH<CPU>::getBVH() {
	return finalTree;
}