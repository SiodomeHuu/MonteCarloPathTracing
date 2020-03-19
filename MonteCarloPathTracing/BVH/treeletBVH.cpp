#include "treeletBVH.h"
#include <algorithm>
#include <deque>
#include <functional>

using namespace MCPT;
using namespace MCPT::BVH;

constexpr int MAX_NODE = 7;
constexpr int TOTAL_BIT = 0x0000007F;

constexpr float Cinn = 1.2f;
constexpr float Cleaf = 0.0f;
constexpr float Ctri = 1.0f;


namespace {

	std::vector<float> SAHValue;


	inline BoundingBox unionBox(const BoundingBox& a, const BoundingBox& b) {
		BoundingBox ans;
		ans.bbmax = max(a.bbmax, b.bbmax);
		ans.bbmin = min(a.bbmin, b.bbmin);
		return ans;
	}
	inline float AREA(const BoundingBox& a) {
		auto arg = a.bbmax - a.bbmin;
		return 2.0f * (arg.x * arg.y + arg.x * arg.z + arg.y * arg.z);
	}
	inline float AREA(float4 bbmin, float4 bbmax) {
		auto arg = bbmax - bbmin;
		return 2.0f * (arg.x * arg.y + arg.x * arg.z + arg.y * arg.z);
	}



	void split(std::vector<BVHNode>& bvh,std::vector<int> nodeID,)


	void reconstructTreelet(std::vector<BVHNode>& bvh, size_t rootID) {

		struct QueueNode {
			int id;
			float value;
			bool operator<(const QueueNode& ano) const {
				if (value < ano.value) return true;
				else if (value == ano.value && id < ano.id) return true;
				return false;
			}
		};

		std::vector< QueueNode > pq;
		std::deque< int > freeBVHNode; // used in reconstruction part

		auto& node = bvh[rootID];
		pq.push_back({ (int)rootID,SAHValue[rootID] });


		while (pq.size() < MAX_NODE) {
			auto maxNode = pq.front();
			auto maxNodeID = maxNode.id;
			std::pop_heap(pq.begin(), pq.end());
			pq.pop_back();

			if (maxNode.value < 0.0f) {
				break;  // all nodes are already leaves
			}

			auto lid = bvh[maxNodeID].left;
			auto rid = bvh[maxNodeID].right;
			
			if (lid == rid) {
				pq.push_back({ maxNodeID,-1.0f });
				std::push_heap(pq.begin(), pq.end());
			} // if leaf and max, put to back
			else {
				pq.push_back({ lid,SAHValue[lid] });
				std::push_heap(pq.begin(), pq.end());
				pq.push_back({ rid,SAHValue[rid] });
				std::push_heap(pq.begin(), pq.end());

				freeBVHNode.push_back(maxNodeID);
			}
		} // then select MAX_NODE of the nodes need to be reconstructed
		
		std::vector< float > areaOfUnion;
		std::vector< BoundingBox > unionBoxes;
		areaOfUnion.resize((1 << MAX_NODE));
		unionBoxes.resize((1 << MAX_NODE));

		for (int i = 1; i < (1 << MAX_NODE); ++i) {
			auto x = [&]() {
				std::vector<int> ans;
				ans.resize(MAX_NODE,0);
				int temp = MAX_NODE - 1;
				int s = i;
				while (s > 0) {
					ans[temp] = s & 0x1;
					s >>= 1;
					--temp;
				}
				return ans;
			}();
			BoundingBox box = { {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX},
			{-FLT_MAX,-FLT_MAX,-FLT_MAX,-FLT_MAX} };

			for (int j = 0; j < MAX_NODE; ++j) {
				if (x[j]) {
					box = unionBox(box, { bvh[pq[j].id].bbmin,bvh[pq[j].id].bbmax });
				}
			}
			unionBoxes[i] = box;
			areaOfUnion[i] = AREA(box);
		} // calc SA for each subset

		std::vector< float > cost;
		cost.resize(1 << MAX_NODE);
		for (int i = 0; i < MAX_NODE; ++i) {
			cost[(1 << i)] = SAHValue[ pq[i].id ];
		} // initialize costs of individual leaves



		struct Node {
			int count;
			int value;
			bool operator<(const Node& ano) const {
				if (count < ano.count) return true;
				else if (count == ano.count && value < ano.value) return true;
				else return false;
			} // we need the smaller one in front, so overload operator<() as greaterthan?
		};

		auto bitsVector = [&]() {
			std::vector< Node > ans;
			auto CZ = [](int i) -> int {
				int ans = 0;
				while (i > 0) {
					ans += (i & 0x1);
					i >>= 1;
				}
				return ans;
			};

			for (int s = 1; s < (1 << MAX_NODE); ++s) {
				ans.push_back({ CZ(s),s });
			}
			sort(ans.begin(), ans.end());

			return ans;
		}();

		struct CsPsNode {
			float cost;
			int partition;
		};
		std::vector< CsPsNode > cp;
		cp.resize(1 << MAX_NODE);

		std::vector< int > partitionPos;
		partitionPos.resize(1 << MAX_NODE);

		int startFrom;
		for (startFrom = 0; startFrom < bitsVector.size(); ++startFrom) {
			if (bitsVector[startFrom].count == 2) break;
		}
		for (int k = 2; k <= MAX_NODE; ++k) {

			// below two variable store the split position & cost
			NEXT:

			float cs = FLT_MAX;
			float ps = 0;

			int s = bitsVector[startFrom].value;
			int delta = (s - 1) & s;
			int p = (-delta) & s;

			do {
				float c = cost[p] + cost[s ^ p];
				if (c < cs) {
					cs = c;
					ps = p;
				}
				p = (p - delta) & s;
			} while (p != 0);

			// because now one triangle one leaf node
			// so the algorithm of "Karras2013HPG_TreeletBVH"
			// codes of line 19~21 are not implemented below

			//float t = trianglesCount(L,s);
			//cost[s] = std::min( (),() );
			cost[s] = Cinn * areaOfUnion[s] + cs;
			partitionPos[s] = ps;

			++startFrom;
			if (startFrom >= bitsVector.size()) break;
			if (bitsVector[startFrom].count == k)
				goto NEXT;
		}

		// then reconstruct tree
		{


			std::function< void(std::vector<BVHNode>&, std::vector<int>&, int, int) > split;





			struct AuxTreeNode {
				int split;
				int parent;
				int left;
				int right;
			};

			std::vector< int > partitionTree;
			partitionTree.resize((MAX_NODE << 1) - 1);
			partitionTree[0] = partitionPos[partitionPos.size() - 1];

			for (int i = 0; i < (partitionTree.size() >> 1); ++i) {
				auto left = (i << 1) + 1;
				auto right = left + 1;
				partitionTree[left] = partitionPos[ partitionTree[i] ];
				partitionTree[right] = (~partitionTree[left]) & TOTAL_BIT;
			}

			int x = 0;

		}

	}



	void recurseGet(const std::vector<BVHNode>& bvh, int rootID, float rootArea) {
		if (SAHValue[rootID] == -1.0f) {
			if (rootID > (bvh.size() >> 1)) {
				SAHValue[rootID] = (Ctri + Cleaf) * AREA(bvh[rootID].bbmin, bvh[rootID].bbmax) / rootArea;
				return;
			}
			else {
				auto left = bvh[rootID].left;
				auto right = bvh[rootID].right;
				recurseGet(bvh, left, rootArea);
				recurseGet(bvh, right, rootArea);
				SAHValue[rootID] = SAHValue[left] + SAHValue[right] + 
					Cinn * (AREA(bvh[rootID].bbmin,bvh[rootID].bbmax) ) / rootArea;
			}
		}
		else {
			return;
		}
	}

	void getInformation(const std::vector<BVHNode>& bvh) {
		SAHValue.resize(bvh.size(), -1.0f);
		float rootArea = AREA(bvh[0].bbmin, bvh[0].bbmax);
		recurseGet(bvh, 0, rootArea);
	}

	
}



TreeletBVH<CPU>::TreeletBVH(const std::vector<BVHNode>& bvh)
	: bvhnode(bvh)
{
	getInformation(bvhnode);
	reconstructTreelet(bvhnode, 0);
}

TreeletBVH<CPU>::TreeletBVH(std::vector<BVHNode>&& bvh)
	: bvhnode(bvh)
{
	getInformation(bvhnode);
	reconstructTreelet(bvhnode, 0);
}

const std::vector<BVHNode>& MCPT::BVH::TreeletBVH<CPU>::getBVH() {
	return bvhnode;
}



