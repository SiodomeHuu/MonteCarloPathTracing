#include "treeletBVH.h"
#include <algorithm>
#include <deque>
#include <functional>
#include <unordered_set>

#include "../auxiliary.h"

#include <iostream>

using namespace MCPT;
using namespace MCPT::BVH;
using namespace MCPT::Auxiliary;

constexpr int MAX_NODE = 7;
constexpr int TOTAL_BIT = (1 << MAX_NODE) - 1; //0x0000007F;




namespace {


	void reconstructTreelet(std::vector<BVHNode>& bvh, size_t rootID, std::vector<float>& sahValue, float rootArea) {

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
		pq.push_back({ (int)rootID,sahValue[rootID] });


		while (pq.size() < MAX_NODE) {
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
				pq.push_back({ maxNodeID,maxNodeID*(-1.0f) });
				std::push_heap(pq.begin(), pq.end());
				continue;
			} // if leaf and max, put to back
			else {
				pq.push_back({ lid,sahValue[lid] });
				std::push_heap(pq.begin(), pq.end());
				pq.push_back({ rid,sahValue[rid] });
				std::push_heap(pq.begin(), pq.end());

				freeBVHNode.push_back(maxNodeID);
			}
		} // then select MAX_NODE of the nodes need to be reconstructed

		int NOW_NODE;
		int NOW_TOTAL_BIT;

		if (pq.size() < 3) {
			return;
		}
		else if (pq.size() < MAX_NODE) { // now there're bugs when pq.size()<MAX_NODE, wait for debugging
			NOW_NODE = pq.size();
			NOW_TOTAL_BIT = (1 << NOW_NODE) - 1;
		}
		else {
			NOW_NODE = MAX_NODE;
			NOW_TOTAL_BIT = TOTAL_BIT;
		}
		
		std::vector< float > areaOfUnion;
		std::vector< BoundingBox > unionBoxes;
		areaOfUnion.resize((1 << NOW_NODE));
		unionBoxes.resize((1 << NOW_NODE));

		for (int i = 1; i < (1 << NOW_NODE); ++i) {
			auto x = [&]() {
				std::vector<int> ans;
				ans.resize(NOW_NODE,0);
				int temp = NOW_NODE - 1;
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

			for (int j = 0; j < NOW_NODE; ++j) {
				if (x[j]) {
					box = unionBox(box, { bvh[pq[j].id].bbmin,bvh[pq[j].id].bbmax });
				}
			}
			unionBoxes[i] = box;
			areaOfUnion[i] = AREA(box);
		} // calc SA for each subset

		std::vector< float > cost;
		cost.resize(1 << NOW_NODE);
		for (int i = 0; i < NOW_NODE; ++i) {
			cost[(1 << i)] = sahValue[ pq[i].id ];
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

			for (int s = 1; s < (1 << NOW_NODE); ++s) {
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
		cp.resize(1 << NOW_NODE);

		std::vector< int > partitionPos;
		partitionPos.resize(1 << NOW_NODE);

		int startFrom;
		for (startFrom = 0; startFrom < bitsVector.size(); ++startFrom) {
			if (bitsVector[startFrom].count == 2) break;
		}
		for (int k = 2; k <= NOW_NODE; ++k) {

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
			toSplit.push_back({ NOW_TOTAL_BIT , partitionPos[NOW_TOTAL_BIT], freeBVHNode[freeNodeNow] });
			++freeNodeNow;

			while (!toSplit.empty()) {
				for (auto i : toSplit) {
					auto leftCode = partitionPos[i.selfCode];
					auto rightCode = partitionPos[ i.selfCode ^ i.parentCode ];

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
					Cinn * (AREA(bvh[freeBVHNode[i]].bbmin, bvh[freeBVHNode[i]].bbmax));
			}
		}

	}



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

	
	/*void collapseLeaf(const std::vector<BVHNode>& bvh, std::vector<int>& ranges) {
		int objCount = (bvh.size() >> 1) + 1;
		std::vector<int> flags;
		flags.resize(objCount, 0);

		for (int i = objCount - 1; i < bvh.size(); ++i) {
			int idx = bvh[i].parent;

			if (!flags[idx]) {

			}
		}
	}*/
}




struct TreeletBVH<CPU>::Impl {
	std::vector<float> sahValue;
	float rootArea;
	~Impl() {}
};

TreeletBVH<CPU>::TreeletBVH(std::unique_ptr<CPUBVH> pBVH) {
	primitiveCount = static_cast<BVH*>(pBVH.get())->primitiveCount;
	TreeletBVH<CPU>(pBVH->releaseBVH());
}




TreeletBVH<CPU>::TreeletBVH(std::vector<BVHNode> bvh) {
	if (primitiveCount == 0) {
		primitiveCount = (bvh.size() >> 1) + 1;
	}

	bvhnode = std::move(bvh);

	pImpl = std::make_unique<Impl>();
	pImpl->sahValue.clear();
	

	getInformation(bvhnode, pImpl->sahValue);
	pImpl->rootArea = AREA(bvhnode[0].bbmin, bvhnode[0].bbmax);

	std::vector<int> flag;

	flag.resize(primitiveCount - 1);

	for (int i = primitiveCount - 1; i < bvhnode.size(); ++i) {

		//reconstructTreelet(bvhnode, i);
		// Maybe some splits can take here

		auto nowParent = bvhnode[i].parent;
		while (nowParent != -1) {
			if (!flag[nowParent]) { // another child not ready
				flag[nowParent] = 1;
				break;
			}
			else {
				reconstructTreelet(bvhnode, nowParent, pImpl->sahValue, pImpl->rootArea);
				nowParent = bvhnode[nowParent].parent;
			}
		}
	}
}


const std::vector<BVHNode>& MCPT::BVH::TreeletBVH<CPU>::getBVH() {
	return bvhnode;
}


std::vector<BVHNode>&& MCPT::BVH::TreeletBVH<CPU>::releaseBVH() {
	return std::move(bvhnode);
}


///////////////// below : in processing

TreeletBVH_<CPU>::TreeletBVH_(const std::vector<BVHNode>& bvh) {
	auto tBVH = bvh;
	TreeletBVH_<CPU>(std::move(tBVH));
}


TreeletBVH_<CPU>::TreeletBVH_(std::vector<BVHNode>&& bvh) {
	
	TreeletBVH<CPU> tempTreeletBVH{ std::move(bvh) };
	primitiveCount = tempTreeletBVH.primitiveCount;


	auto sahValue = std::move(tempTreeletBVH.pImpl->sahValue);
	auto rootArea = tempTreeletBVH.pImpl->rootArea;
	auto tempTree = tempTreeletBVH.releaseBVH();
	// fully decompose the information for older treelet bvh tree
	
	indices.reserve(primitiveCount);

	// flag stores the count of primitives
	std::vector<int> flag;
	flag.resize(primitiveCount << 1 , 0);

	// get the count of primitives for all inner nodes
	for (int i = primitiveCount - 1; i < tempTree.size(); ++i) {
		flag[i] = -1;

		auto now = i;
		auto nowParent = tempTree[i].parent;

		while (nowParent != -1) { // another child ready
			if (!flag[nowParent]) {
				flag[nowParent] = flag[now];
				break;
			}
			flag[nowParent] += flag[now];
			now = nowParent;
			nowParent = tempTree[nowParent].parent;
		}
	}


	std::unordered_set<int> collapseIDs;
	/*for (int i = primitiveCount - 1; i < tempTree.size(); ++i) {
		auto nowParent = tempTree[i].parent;

		int toCollapse = -1;

		while (nowParent != -1) {
			if (flag[nowParent] < 0) {
				flag[nowParent] = -flag[nowParent];
			}
			else {
				auto collapseCost = Ctri * flag[nowParent] * AREA(tempTree[nowParent].bbmin, tempTree[nowParent].bbmax);
				if (collapseCost <= sahValue[nowParent]) {
					toCollapse = nowParent;
					sahValue[nowParent] = collapseCost;
				}
				nowParent = tempTree[nowParent].parent;
			}
		}
		if (toCollapse >= 0) {
			collapseIDs.insert(toCollapse);
		}
	}*/



	// top-down check all nodes that need to take a collapse
	struct StackNode {
		unsigned int self;
		unsigned int parent;
		bool isLeft;
	};
	std::deque<StackNode> stack;
	stack.push_back({ 0, 0, true });

	while (!stack.empty()) {
		auto backNode = stack.back();
		auto back = backNode.self;
		stack.pop_back();

		auto left = tempTree[back].left;
		auto right = tempTree[back].right;
		auto backArea = AREA(tempTree[back].bbmin, tempTree[back].bbmax);

		if (flag[back] == -1) {
			//MARK
			auto selfID = bvhNode.size();

			auto singlePrimitiveID = left;
			auto l = indices.size();
			indices.push_back(singlePrimitiveID);

			MultiPrimBVHNode tempnode;
			tempnode.bbmin = tempTree[back].bbmin;
			tempnode.bbmax = tempTree[back].bbmax;
			tempnode.parent = backNode.parent;

			tempnode.left = l;
			tempnode.right = l + 1;
			tempnode.isLeaf = 1;

			bvhNode.push_back(tempnode);
			if (backNode.isLeft)
				bvhNode[backNode.parent].left = selfID;
			else
				bvhNode[backNode.parent].right = selfID;
		}
		else if (collapseIDs.find(back) != collapseIDs.end()) {
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

			auto primitiveIDs = getPrimitiveIDs(tempTree, back);
			//if (primitiveIDs.size() > 4) goto NOCOLLAPSE;
			auto l = indices.size();
			auto r = l + primitiveIDs.size();
			for (auto id : primitiveIDs) {
				indices.push_back(id);
			}

			auto selfID = bvhNode.size();
			MultiPrimBVHNode tempnode;
			tempnode.bbmin = tempTree[back].bbmin;
			tempnode.bbmax = tempTree[back].bbmax;
			tempnode.parent = backNode.parent;
			tempnode.left = l;
			tempnode.right = r;
			tempnode.isLeaf = 1;
			
			bvhNode.push_back(tempnode);
			if (backNode.isLeft)
				bvhNode[backNode.parent].left = selfID;
			else
				bvhNode[backNode.parent].right = selfID;
		}
		else {
		NOCOLLAPSE:
			auto selfID = bvhNode.size();
			MultiPrimBVHNode tempnode;
			tempnode.bbmin = tempTree[back].bbmin;
			tempnode.bbmax = tempTree[back].bbmax;
			tempnode.parent = backNode.parent;
			tempnode.isLeaf = 0;

			bvhNode.push_back(tempnode);
			if (backNode.isLeft)
				bvhNode[backNode.parent].left = selfID;
			else
				bvhNode[backNode.parent].right = selfID;
			stack.push_back({ (uint)left,(uint)selfID,true });
			stack.push_back({ (uint)right,(uint)selfID,false });
		}
	}
	bvhNode[0].parent = -1;
}







/*

	Below : GPU TreeletBVH

*/





namespace {
	cl::Program treeGPUProg;
	cl::Kernel treeKernel;

	cl::Kernel sahKernel;

	void initProgramAndKernel() {
		static auto initer = []() {
			treeGPUProg = OpenCLBasic::createProgramFromFileWithHeader("./kernels/treeletBVH.cl", "objdef.h"); // , "-D MCPT_ONE_WORKGROUP=1");
			treeKernel = OpenCLBasic::createKernel(treeGPUProg, "reconstructTreelet");
			
			sahKernel = OpenCLBasic::createKernel(treeGPUProg, "calculateSAH");
			return 0;
		}();
	}
}



TreeletBVH<GPU>::TreeletBVH(std::pair<cl::Buffer, cl::Buffer> bvh)
	: TreeletBVH(bvh.first,bvh.second)
{}

TreeletBVH<GPU>::TreeletBVH(cl::Buffer node, cl::Buffer tri) 
	: bvhNode(node), objectNode(tri)
{

	int nodeNum = bvhNode.getInfo<CL_MEM_SIZE>() / sizeof(BVHNode);
	int objNum = objectNode.getInfo<CL_MEM_SIZE>() / sizeof(Triangle);

	initProgramAndKernel();

	cl::Buffer sahV = OpenCLBasic::newBuffer<float>(nodeNum);
	cl::Buffer flagB = OpenCLBasic::newBuffer<int>(objNum);

	cl::Event ev;

	try {
		OpenCLBasic::setKernelArg(sahKernel, bvhNode, sahV, flagB, objNum);
		OpenCLBasic::enqueueNDRange(sahKernel, objNum, cl::NullRange);

		OpenCLBasic::getQueue().enqueueFillBuffer(flagB, 0, 0, objNum*sizeof(int));

		OpenCLBasic::setKernelArg(treeKernel, bvhNode, sahV, flagB, objNum);
		OpenCLBasic::enqueue1DKernelWithGroupCount(treeKernel, objNum, 32, nullptr, &ev);
		OpenCLBasic::getQueue().finish();
	}
	catch (cl::Error & e) {
		std::cout << "Error: " << e.err() << " " << e.what() << std::endl;
		throw "Error";
	}

	std::cout << "Build time: " << OpenCLBasic::timeCost(ev)/1e6 << std::endl;
}



std::pair<cl::Buffer,cl::Buffer> TreeletBVH<GPU>::getBuffer() {
	return { bvhNode,objectNode };
}