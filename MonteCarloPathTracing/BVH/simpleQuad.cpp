#include "simpleQuad.h"

using namespace MCPT;
using namespace MCPT::BVH;

#include <iostream>
#include <deque>

namespace {


	void formQuadBVH(const std::vector<BVHNode>& bvh, std::vector<QuadBVHNode>& qbvh) {
		struct QueueNode {
			int id; // old id
			int newId;
			int newParent; // new parent
		};
		int objCount = (bvh.size() >> 1) + 1;
		std::vector< QueueNode > idQueue;
		idQueue.push_back({ 0,0, -1 });
		qbvh.emplace_back();
		
		struct LeafNode {
			int parent;
			int whichChild;
		};

		std::vector< LeafNode > leafNodes;
		leafNodes.resize(objCount);

		while (!idQueue.empty()) {
			auto nowRoot = idQueue.back();
			idQueue.pop_back();

			auto& temp = qbvh[nowRoot.newId];

			temp.bbmin = bvh[nowRoot.id].bbmin;
			//temp.bbmin = { -1.0f,-1.0f,-1.0f,0.0f };
			temp.bbmax = bvh[nowRoot.id].bbmax;
			//temp.bbmax = { 5.0f,5.0f,5.0f,0.0f };

			temp.parent.w = nowRoot.newParent;
			
			auto left = bvh[nowRoot.id].left;
			auto right = bvh[nowRoot.id].right;

			if (left == right) { //already leaf
				throw "Not Here!!!";
			}
			else {
				auto ll = bvh[left].left;
				auto lr = bvh[left].right;
				auto rl = bvh[right].left;
				auto rr = bvh[right].right;

				if (ll == lr) {
					leafNodes[left - objCount + 1].parent = nowRoot.newId;
					leafNodes[left - objCount + 1 ].whichChild = 0;
					temp.children.y = -2;
				}
				else {
					int peekLLL = bvh[ll].left;
					int peekLLR = bvh[ll].right;
					if (peekLLL == peekLLR) {
						leafNodes[ll - objCount + 1].parent = nowRoot.newId;
						leafNodes[ll - objCount + 1].whichChild = 0;
					}
					else {
						qbvh.emplace_back();
						int tempID1 = qbvh.size() - 1;
						idQueue.push_back({ ll,tempID1,nowRoot.newId });
						temp.children.x = tempID1;
					}
					int peekLRL = bvh[lr].left;
					int peekLRR = bvh[lr].right;
					if (peekLRL == peekLRR) {
						leafNodes[lr - objCount + 1].parent = nowRoot.newId;
						leafNodes[lr - objCount + 1].whichChild = 1;
					}
					else {
						qbvh.emplace_back();
						int tempID2 = qbvh.size() - 1;
						idQueue.push_back({ lr,tempID2,nowRoot.newId });
						temp.children.y = tempID2;
					}
				}


				if (rl == rr) {
					leafNodes[right - objCount + 1].parent = nowRoot.newId;
					leafNodes[right - objCount + 1].whichChild = 2;
					temp.children.w = -2;
				}
				else {
					int peekRLL = bvh[rl].left;
					int peekRLR = bvh[rl].right;
					if (peekRLL == peekRLR) {
						leafNodes[rl - objCount + 1].parent = nowRoot.newId;
						leafNodes[rl - objCount + 1].whichChild = 2;
					}
					else {
						qbvh.emplace_back();
						int tempID1 = qbvh.size() - 1;
						idQueue.push_back({ rl,tempID1,nowRoot.newId });
						temp.children.z = tempID1;
					}
					int peekRRL = bvh[rr].left;
					int peekRRR = bvh[rr].right;
					if (peekRRL == peekRRR) {
						leafNodes[rr - objCount + 1].parent = nowRoot.newId;
						leafNodes[rr - objCount + 1].whichChild = 3;
					}
					else {
						qbvh.emplace_back();
						int tempID2 = qbvh.size() - 1;
						idQueue.push_back({ rr,tempID2,nowRoot.newId });
						temp.children.w = tempID2;
					}
				}
			}
		}
		
		int offset = qbvh.size();
		qbvh.resize(offset + objCount);
		for (int i = 0; i < leafNodes.size(); ++i) {
			qbvh[offset + i].bbmin = bvh[objCount + i - 1].bbmin;
			qbvh[offset + i].bbmax = bvh[objCount + i - 1].bbmax;
			auto temp = qbvh[offset + i].parent.w = leafNodes[i].parent;
			qbvh[temp].children.s[leafNodes[i].whichChild] = offset + i;
			qbvh[offset + i].children.x = qbvh[offset + i].children.y = bvh[objCount + i - 1].left;
		}

		//int totest1 = 0;
		//int totest2 = 0;
		//std::vector<int> flag;
		//flag.resize(qbvh.size(), 0);
		//for (int i = offset; i < qbvh.size(); ++i) {
		//	int now = i; // qbvh[i].parent.w;
		//	while (now >= 0) {
		//		auto notEmptyNodeCount = 4;
		//		for (int j = 0; j < 4; ++j) {
		//			if (qbvh[now].children.s[j] < 0) {
		//				--notEmptyNodeCount;
		//			}
		//		}
		//		flag[now] += 1;
		//		if (flag[now] < notEmptyNodeCount) {
		//			break;
		//		}
		//		auto orimin = qbvh[now].bbmin;
		//		auto orimax = qbvh[now].bbmax;
		//		std::vector< QuadBVHNode* > v;
		//		for (int j = 0; j < 4; ++j) {
		//			if (qbvh[now].children.s[j] >= 0)
		//				v.push_back(qbvh.data() + qbvh[now].children.s[j]);
		//		}
		//		int tpparent = qbvh[now].parent.w;
		//		for (int j = 0; j < v.size(); ++j) {
		//			qbvh[now].bbmin = min(qbvh[now].bbmin, v[j]->bbmin);
		//			qbvh[now].bbmax = max(qbvh[now].bbmax, v[j]->bbmax);
		//		}
		//		qbvh[now].parent.w = tpparent;
		//		auto& t = qbvh[now];
		//		
		//		if (orimin.x != t.bbmin.x || orimin.y != t.bbmin.y || orimin.z != t.bbmin.z) {
		//			std::cout << "s" << now << " ";
		//		}
		//		if (orimax.x != t.bbmax.x || orimax.y != t.bbmax.y || orimax.z != t.bbmax.z) {
		//			std::cout << "m" << now << " ";
		//		}
		//		now = qbvh[now].parent.w;
		//	}
		//}
	}

}

namespace {
	void depth(const std::vector<QuadBVHNode>& toTest, int objCount) {
		std::vector< int > allNode;
		allNode.resize(toTest.size(),0);

		std::vector< int > allLeaf;
		allLeaf.resize(objCount, 0);


		std::vector<int> queue;
		queue.push_back(0);
		while (!queue.empty()) {
			auto temp = queue.back();
			//std::cout << temp << " ";
			queue.pop_back();
			if (allNode[temp] != 0) {
				std::cout << "Twice visited on Inner Node!";
			}
			allNode[temp] = 1;

			if (toTest[temp].children.x == toTest[temp].children.y) {
				if (temp < toTest.size() - objCount) {
					std::cout << "Leaf node inside inner nodes";
				}
				auto t2 = toTest[temp].children.x;
				if (allLeaf[t2] > 0) std::cout << "Twice visited on leaf node!";
				allLeaf[t2] = 1;
				continue;
			}

			if (toTest[temp].children.x <= 0) {
				std::cout << "X shouldn't be zero";
			}
			else {
				queue.push_back(toTest[temp].children.x);
			}

			if (toTest[temp].children.y > 0) {
				queue.push_back(toTest[temp].children.y);
			}

			if (toTest[temp].children.z <= 0) {
				std::cout << "Z shouldn't be zero";
			}
			else {
				queue.push_back(toTest[temp].children.z);
			}

			if (toTest[temp].children.w > 0) {
				queue.push_back(toTest[temp].children.w);
			}

		}
		for (int i = 0; i < allNode.size(); ++i) {
			if (allNode[i] == 0) {
				std::cout << "Inner node not visited!";
			}
		}
		for (int i = 0; i < allLeaf.size(); ++i) {
			if (allLeaf[i] == 0) {
				std::cout << "Leaf node not visited";
			}
		}
	}
}


SimpleBVHQuad<CPU>::SimpleBVHQuad(const std::vector<BVHNode>& bvh) {
	bvhnode.reserve(bvh.size());
	objCount = (bvh.size() >> 1) + 1;
	formQuadBVH(bvh, bvhnode);
	//depth(bvhnode, objCount);
}

const std::vector<BVHNode>& SimpleBVHQuad<CPU>::getBVH() {
	throw "Quad BVH doesn't support this";
}

std::vector<BVHNode>&& SimpleBVHQuad<CPU>::releaseBVH() {
	throw "Quad BVH doesn't support this";
}

const std::vector<QuadBVHNode>& SimpleBVHQuad<CPU>::getQuadBVH() {
	return bvhnode;
}

int MCPT::BVH::SimpleBVHQuad<CPU>::getObjectCount() {
	return objCount;
}
