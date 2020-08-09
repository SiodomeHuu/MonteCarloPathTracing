#include "treeletTemp.h"
#include <algorithm>
#include <deque>
#include <functional>
#include <unordered_set>

#include <iostream>

namespace {

	using namespace MCPT;
	using namespace MCPT::BVH;
	using namespace MCPT::Auxiliary;

	void recurseGet(const std::vector<BVHNode>& bvh, int rootID, float rootArea, std::vector<float>& sahValue) {
		if (sahValue[rootID] == -1.0f) {
			if (rootID >= (bvh.size() >> 1)) {
				sahValue[rootID] = (Ctri + Cleaf) * AREA(bvh[rootID].bbmin, bvh[rootID].bbmax);
				return;
			}
			else {
				auto left = bvh[rootID].left;
				auto right = bvh[rootID].right;
				recurseGet(bvh, left, rootArea, sahValue);
				recurseGet(bvh, right, rootArea, sahValue);
				sahValue[rootID] = sahValue[left] + sahValue[right] +
					Cinn * (AREA(bvh[rootID].bbmin, bvh[rootID].bbmax));
			}
		}
		else {
			return;
		}
	}

	void getInformation(const std::vector<BVHNode>& bvh, std::vector<float>& sahValue) {
		sahValue.resize(bvh.size(), -1.0f);
		float rootArea = AREA(bvh[0].bbmin, bvh[0].bbmax);
		recurseGet(bvh, 0, rootArea, sahValue);
	}


	inline float area(const BoundingBox& c) { return AREA(c); }
	inline float area(const float4& a, const float4& b) { return AREA(a, b); }

}



namespace MCPT::BVH {
	using namespace Auxiliary;

	using BBox = BoundingBox;

	void reconstructTreelet(
		std::vector<BVHNode>& bvh, size_t rootID, // to perform optimazation
		std::vector<float>& sahValue, // sah information
		const float treeRootArea,
		const int maxNode, const int maxBit // have to be {7, 127}
	) {

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
		std::vector< int > freeBVHNode; // used in reconstruction part

		auto& node = bvh[rootID];
		pq.push_back({ (int)rootID, sahValue[rootID] });


		while (pq.size() < maxNode) {
			auto maxNode = pq.front();
			auto maxNodeID = maxNode.id;
			std::pop_heap(pq.begin(), pq.end());
			pq.pop_back();

			if (maxNode.value < 0.0f) {
				pq.push_back({ maxNodeID,-1.0f });
				break;  // all nodes are already leaves
			}

			auto lid = bvh[maxNodeID].left;
			auto rid = bvh[maxNodeID].right;

			if (lid == rid) {
				pq.push_back({ maxNodeID,maxNodeID * (-1.0f) });
				std::push_heap(pq.begin(), pq.end());
				continue;
			} // if leaf and max, put to back
			else {
				pq.push_back({ lid, sahValue[lid] });
				std::push_heap(pq.begin(), pq.end());
				pq.push_back({ rid, sahValue[rid] });
				std::push_heap(pq.begin(), pq.end());

				freeBVHNode.push_back(maxNodeID);
			}
		} // then select MAX_NODE of the nodes need to be reconstructed

		int nowNode;
		int nowTotalBit;

		if (pq.size() < 3) {
			return;
		}
		else if (pq.size() < maxNode) {
			nowNode = pq.size();
			nowTotalBit = (1 << nowNode) - 1;
		}
		else {
			nowNode = maxNode;
			nowTotalBit = maxBit;
		}

		std::vector< float > areaOfUnion;
		std::vector< BBox > unionBoxes;
		areaOfUnion.resize((1 << nowNode));
		unionBoxes.resize((1 << nowNode));

		for (int i = 1; i < (1 << nowNode); ++i) {
			auto x = [&]() {
				std::vector<int> ans;
				ans.resize(nowNode, 0);
				int temp = nowNode - 1;
				int s = i;
				while (s > 0) {
					ans[temp] = s & 0x1;
					s >>= 1;
					--temp;
				}
				return ans;
			}();
			BBox box;

			for (int j = 0; j < nowNode; ++j) {
				if (x[j]) {
					box = BBox::unionBBox(box, { bvh[pq[j].id].bbmin, bvh[pq[j].id].bbmax });
				}
			}
			unionBoxes[i] = box;
			areaOfUnion[i] = area(box);
		} // calc SA for each subset

		std::vector< float > cost;
		cost.resize(1 << nowNode);
		for (int i = 0; i < nowNode; ++i) {
			cost[(1 << i)] = sahValue[pq[i].id];
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

			for (int s = 1; s < (1 << nowNode); ++s) {
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
		cp.resize(1 << nowNode);

		std::vector< int > partitionPos;
		partitionPos.resize(1 << nowNode);

		int startFrom;
		for (startFrom = 0; startFrom < bitsVector.size(); ++startFrom) {
			if (bitsVector[startFrom].count == 2) break;
		}
		for (int k = 2; k <= nowNode; ++k) {

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

			cost[s] = Cinn * areaOfUnion[s] + cs;
			partitionPos[s] = ps;

			++startFrom;
			if (startFrom >= bitsVector.size()) break;
			if (bitsVector[startFrom].count == k)
				goto NEXT;
		}

		// then reconstruct tree
		{
			auto Count1Num = [](int x) -> int {
				int ans = 0;
				while (x > 0) {
					ans += (x & 0x1);
					x >>= 1;
				}
				return ans;
			};
			auto CLZ = [](int a) -> int {
				int ans = 0;
				if (a == 0) return 32;
				while (a > 0) {
					a = a << 1;
					++ans;
				}
				return ans;
			};

			struct SplitInnerNode {
				int parentCode;
				int selfCode;
				int parentID;
			};

			std::vector< SplitInnerNode > toSplit;
			std::vector< SplitInnerNode > answer;

			int freeNodeNow = 0;
			toSplit.push_back({ nowTotalBit , partitionPos[nowTotalBit], freeBVHNode[freeNodeNow] });
			++freeNodeNow;

			while (!toSplit.empty()) {
				for (auto i : toSplit) {
					auto leftCode = partitionPos[i.selfCode];
					auto rightCode = partitionPos[i.selfCode ^ i.parentCode];

					auto parentNodeID = i.parentID;

					if (Count1Num(i.selfCode) == 1) {
						int toRight = (8 * sizeof(int) - 1) - CLZ(i.selfCode);
						int node = pq[pq.size() - toRight - 1].id;

						bvh[parentNodeID].left = node;
						bvh[node].parent = parentNodeID;
					}
					else {
						auto freeNext = bvh[parentNodeID].left = freeBVHNode[freeNodeNow++];
						answer.push_back({ i.selfCode ,leftCode, freeNext });

						bvh[parentNodeID].left = freeNext;
						bvh[freeNext].parent = parentNodeID;
					}

					if (Count1Num(i.parentCode ^ i.selfCode) == 1) {
						int toRight = (8 * sizeof(int) - 1) - CLZ(i.parentCode ^ i.selfCode);
						int node = pq[pq.size() - toRight - 1].id;

						bvh[parentNodeID].right = node;
						bvh[node].parent = parentNodeID;
					}
					else {
						auto freeNext = bvh[parentNodeID].right = freeBVHNode[freeNodeNow++];
						answer.push_back({ i.selfCode ^ i.parentCode , rightCode, freeNext });

						bvh[parentNodeID].right = freeNext;
						bvh[freeNext].parent = parentNodeID;
					}
				}
				toSplit = std::move(answer);
			}

		}

		{
			for (int i = freeBVHNode.size() - 1; i >= 0; --i) {
				auto& node = bvh[freeBVHNode[i]];
				node.bbmax = max(bvh[node.left].bbmax, bvh[node.right].bbmax);
				node.bbmin = min(bvh[node.left].bbmin, bvh[node.right].bbmin);

				sahValue[freeBVHNode[i]] = sahValue[node.left] + sahValue[node.right] +
					Cinn * (area(bvh[freeBVHNode[i]].bbmin, bvh[freeBVHNode[i]].bbmax));
			}
		}

	}


	float TreeletTemp::sah() {

		std::deque< int > stack;
		stack.push_back(0);

		double sahAns = 0.0;
		double rootArea = AREA(bvhNode[0].bbmin, bvhNode[0].bbmax);
		while (!stack.empty()) {
			auto now = stack.back();
			stack.pop_back();

			auto& node = bvhNode[now];
			if (node.isLeaf) {
				sahAns += AREA(node.bbmin, node.bbmax) * (node.right - node.left) * Ctri;
			}
			else {
				sahAns += AREA(node.bbmin, node.bbmax) * Cinn;
				stack.push_back(node.left);
				stack.push_back(node.right);
			}
		}

		return sahAns / rootArea;
	}



	struct TreeletTemp::Impl {
		float rootArea;
		std::vector< float > sahValue;
	};

	TreeletTemp::~TreeletTemp() = default;
	TreeletTemp::TreeletTemp(const TreeletTemp & ano) {
		pImpl = std::make_unique<Impl>(*ano.pImpl);

		bvhNode = ano.bvhNode;
		indices = ano.indices;
	}
	TreeletTemp::TreeletTemp(TreeletTemp && ano) noexcept = default;

	TreeletTemp::TreeletTemp(
		std::vector<BVHNode> bvh,
		const std::vector<uint32_t> & indices_,
		const std::any & arg)
		: bvhNode(std::move(bvh)), pImpl(std::make_unique<Impl>()) {

		int tempPair;
		if (arg.type() != typeid(int)) {
			tempPair = 32;
		}
		else {
			tempPair = std::any_cast<int>(arg);
		}
		auto temp = 7;

		const int maxNode = temp;
		const int maxBit = (1 << temp) - 1;

		pImpl->rootArea = area(bvhNode[0].bbmin, bvhNode[0].bbmax);
		pImpl->sahValue.resize(bvhNode.size());

		getInformation(bvhNode, pImpl->sahValue);

		std::vector<int> flag;
		flag.resize(bvhNode.size() >> 1);

		for (int i = (bvhNode.size() >> 1); i < bvhNode.size(); ++i) {
			pImpl->sahValue[i] = (Ctri + Cleaf) * area(bvhNode[i].bbmin, bvhNode[i].bbmax);

			auto nowParent = bvhNode[i].parent;
			while (nowParent != -1) {
				if (!flag[nowParent]) { // another child not ready
					flag[nowParent] = 1;
					break;
				}
				else {
					reconstructTreelet(bvhNode, nowParent, pImpl->sahValue, pImpl->rootArea, maxNode, maxBit);
					nowParent = bvhNode[nowParent].parent;
				}
			}
		}

		// bottom-up check all nodes that need to take a collapse
		flag.clear();
		flag.resize(bvhNode.size());
		std::unordered_set<int> collapseIDs;
		for (int i = (bvhNode.size() >> 1); i < bvhNode.size(); ++i) {
			auto self = i;
			auto nowParent = bvhNode[i].parent;

			int toCollapse = -1;

			flag[i] = 1;
			while (nowParent != -1) {
				if (!flag[nowParent]) {
					flag[nowParent] = flag[self];
					break;
				}
				else if (flag[self] == 0) {
					break;
				}
				flag[nowParent] += flag[self];

				if (tempPair > 0 && flag[nowParent] > tempPair) {
					break;
				}
				else {
					auto collapseCost = Ctri * flag[nowParent] * area(bvhNode[nowParent].bbmin, bvhNode[nowParent].bbmax);
					if (collapseCost <= pImpl->sahValue[nowParent]) {
						toCollapse = nowParent;
						pImpl->sahValue[nowParent] = collapseCost;
					}
					self = nowParent;
					nowParent = bvhNode[nowParent].parent;
				}
			}
			if (toCollapse >= 0) {
				collapseIDs.insert(toCollapse);
			}
		}

		// top-down to take a collapse
		std::vector<uint32_t> sortedIndices;
		struct StackNode {
			unsigned int self;
		};
		std::deque<StackNode> stack;
		stack.push_back({ 0 });
		while (!stack.empty()) {
			auto backNode = stack.back();
			auto back = backNode.self;
			stack.pop_back();

			if (collapseIDs.find(back) != collapseIDs.end()) {
				// collapse
				auto getPrimitiveIDs = [](const std::vector<BVHNode>& tree, unsigned int root) {
					std::vector<unsigned int> ans;
					std::deque<unsigned int> stack;
					stack.push_back(root);
					while (!stack.empty()) {
						auto back = stack.back();
						stack.pop_back();
					AGAIN:
						auto left = tree[back].left;
						auto right = tree[back].right;
						if (left == right) {
							ans.push_back(left);
							continue;
						}
						stack.push_back(right);
						back = left;
						goto AGAIN;
					}
					return ans;
				};

				auto primitiveIDs = getPrimitiveIDs(bvhNode, back);
				auto l = sortedIndices.size();
				auto r = l + primitiveIDs.size();
				for (auto id : primitiveIDs) {
					sortedIndices.push_back(indices_[id]);
				}

				bvhNode[back].left = l;
				bvhNode[back].right = r;
				bvhNode[back].isLeaf = 1;
			}
			else if (bvhNode[back].left == bvhNode[back].right) {
				auto singlePrimitiveID = bvhNode[back].left;
				auto l = sortedIndices.size();
				sortedIndices.push_back(indices_[singlePrimitiveID]);

				bvhNode[back].left = l;
				bvhNode[back].right = l + 1;
				bvhNode[back].isLeaf = 1;
			}
			else {
				stack.push_back({ (uint32_t)bvhNode[back].left });
				stack.push_back({ (uint32_t)bvhNode[back].right });
			}
		}

		indices.swap(sortedIndices);
	}


	std::vector<uint32_t> TreeletTemp::getLeafIDs() const {
		std::vector<uint32_t> ans;
		for (int i = 0; i < bvhNode.size(); ++i) {
			if (bvhNode[i].isLeaf) ans.push_back(i);
		}
		return ans;
	}
}