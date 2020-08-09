#include <iostream>
#include <algorithm>
#include <deque>

#include "sahbvh.h"
#include "../auxiliary.h"


namespace {
    inline float area(const MCPT::BoundingBox& x) { return MCPT::Auxiliary::AREA(x); }
}

namespace {
    using namespace MCPT::BVH;
    using namespace MCPT::Auxiliary;

    void singleAxisSplit(
        int binCount,
        int maxPrimsInNode,
        const std::vector<BBox>& bboxes,
        const std::vector<cl_float3>& centroids,
        int splitAxis,

        std::vector<unsigned int>::iterator begin, std::vector<unsigned int>::iterator end,
        int& splitPos, // split plane is on the left of splitPos

        const BBox& centroidsBox,
        const BBox& inBox,
        BBox& left, /* out */
        BBox& right, /* out */
        float& minCost, /* out */
        float threshold
    ) {
        int count = end - begin;
        float rootArea = AREA(inBox);

        if (count == 1) {
            splitPos = -1;
            minCost = 1.0f;
            return;
        }

        if (binCount > 1) {
            float range = centroidsBox.bbmax.s[splitAxis] - centroidsBox.bbmin.s[splitAxis];
            if (range == 0.0f) {
                splitPos = count / 2;
                BBox tL, tR;
                for (int i = 0; i < splitPos; ++i) {
                    tL.unionBBox(bboxes[*(begin + i)]);
                }
                for (int i = splitPos; i < count; ++i) {
                    tR.unionBBox(bboxes[*(begin + i)]);
                }
                left = tL;
                right = tR;
                if (left.bbmin.x == FLT_MAX || right.bbmin.x == FLT_MAX) {
                    int x = 0;
                }
                return;
            }

            float inMin = centroidsBox.bbmin.s[splitAxis];
            float gap = range / binCount;

            std::vector<BBox> left2right(binCount);
            std::vector<BBox> right2left(binCount);
            std::vector<BBox> binBox(binCount);

            std::vector< std::deque<uint32_t> > bins(binCount);

            for (auto i = begin; i != end; ++i) {
                int groupID = floor((centroids[*i].s[splitAxis] - inMin) / gap);
                if (groupID == binCount) {
                    [[unlikely]]
                    --groupID;
                }
                bins[groupID].push_back(*i);
                binBox[groupID].unionBBox(bboxes[*i]);
            }

            std::vector<uint32_t> binCountPrefix(binCount);
            binCountPrefix[0] = bins[0].size();
            left2right[0] = binBox[0];
            right2left[binCount - 1] = binBox[binCount - 1];

            for (int i = 1; i < binCount; ++i) {
                binCountPrefix[i] = binCountPrefix[i - 1] + bins[i].size();
                left2right[i] = BBox::unionBBox(left2right[i - 1], binBox[i]);
                right2left[binCount - i - 1] = BBox::unionBBox(right2left[binCount - i], binBox[binCount - i - 1]);

                if (left2right[i].bbmin.x == FLT_MAX || right2left[binCount - i - 1].bbmin.x == FLT_MAX) {
                    int x = 0;
                }
            }


            int tempSplitPos = 0; // 0 -> || 1 2...
            float cost = binCountPrefix[tempSplitPos] * area(left2right[tempSplitPos])
                + (count - binCountPrefix[tempSplitPos]) * area(right2left[tempSplitPos + 1]);

            for (int i = 1; i < binCount - 1; ++i) {
                auto la = area(left2right[i]);
                auto lr = area(right2left[i + 1]);
                if (la == 0.0f || lr == 0.0f) continue;
                float tcost = binCountPrefix[i] * la
                    + (count - binCountPrefix[i]) * lr;
                if (tcost < cost) {
                    tempSplitPos = i;
                    cost = tcost;
                }
            } // compute the split pos

            float realCost;
            if (rootArea == 0.0f) {
                [[unlikely]]
                realCost = 0.125f;
            }
            else {
                realCost = cost / rootArea + 0.125f;
            }
            if (((maxPrimsInNode > 0 && maxPrimsInNode >= count) || maxPrimsInNode <= 0) && realCost >= count) {
                splitPos = -1;
                minCost = realCost;
                return;
            }
            else {
                splitPos = binCountPrefix[tempSplitPos];

                // last to set
                if (realCost < threshold) {
                    auto iter = begin;
                    for (auto& bin : bins) {
                        for (auto id : bin) {
                            *iter = id;
                            ++iter;
                        }
                    } // store back the bins
                    left = left2right[tempSplitPos];
                    right = right2left[tempSplitPos + 1];

                    if (left.bbmin.x == FLT_MAX || right.bbmin.x == FLT_MAX) {
                        int x = 0;
                    }
                }
                else {
                    int x = 0;
                }
                minCost = realCost;
                return;
            }
        }
        else {
            std::vector<BBox> left2right(count);
            std::vector<BBox> right2left(count);

            std::sort(
                begin,
                end,
                [&](int i, int j) -> bool {
                    return centroids[i].s[splitAxis] < centroids[j].s[splitAxis];
                }
            );

            left2right[0] = bboxes[*begin];
            right2left[count - 1] = bboxes[*(end - 1)];

            // prefix bounding box
            for (int i = 1; i < count; ++i) {
                left2right[i] = BBox::unionBBox(left2right[i - 1], bboxes[*(begin + i)]);
                right2left[count - i - 1] = BBox::unionBBox(right2left[count - i], bboxes[*(end - i - 1)]);
            }

            auto tempSplitPos = 1; // 0 || <- 1 2...
            float cost = tempSplitPos * area(left2right[tempSplitPos - 1]) + (count - tempSplitPos) * area(right2left[tempSplitPos]);

            for (int i = 2; i < count; ++i) {
                float tcost = i * area(left2right[i - 1]) + (count - i) * area(right2left[i]);
                if (tcost < cost) {
                    cost = tcost;
                    tempSplitPos = i;
                }
            } // find a suitable split position

            auto realCost = cost / rootArea + 0.125f;
            if (((maxPrimsInNode > 0 && maxPrimsInNode >= count) || maxPrimsInNode <= 0) && realCost >= count) {
                splitPos = -1;
                minCost = realCost;
                return;
            }
            else {
                splitPos = tempSplitPos;
                left = left2right[splitPos - 1];
                right = right2left[splitPos];
                minCost = realCost;
                return;
            }
        }
    }
}

namespace MCPT::BVH {

    std::vector<uint32_t> SAHBVH<CPU>::getLeafIDs() const {
        std::vector<uint32_t> ans;
        for (int i = 0; i < nodes.size(); ++i) {
            if (nodes[i].isLeaf) ans.push_back(i);
        }
        return ans;
    }

    SAHBVH<CPU>::SAHBVH(const std::vector<BBox>& bboxes, const std::vector<uint32_t>& indices_, const std::any& arg) {
        std::tuple<bool, uint32_t, int> tArg;
        if (arg.type() == typeid(std::tuple<bool, uint32_t, int>)) {
            tArg = std::any_cast<std::tuple<bool, uint32_t, int>>(arg);
        }
        else {
            tArg = std::tuple<bool, uint32_t, int>({ false, 0, 16 });
        }
        auto [threeAxis, binCount, maxPrimsInNode] = tArg;

        std::cout << "Arg: " << threeAxis << " " << binCount << " " << maxPrimsInNode << std::endl;

        std::vector<uint32_t> primIndex;
        primIndex.resize(bboxes.size());
        for (int i = 0; i < primIndex.size(); ++i) {
            primIndex[i] = i;
        }

        BBox globalBox;
        std::vector< float4 > centroids;
        for (const auto& box : bboxes) {
            centroids.push_back(box.centroid());
            globalBox.unionBBox(box);
        }

        struct SplitRangeNode {
            uint32_t parentID; bool isLeft;
            uint32_t begin, end;
            BBox box;
        };

        std::deque< SplitRangeNode > stack;
        stack.push_back({ 0, true, 0, (uint32_t)bboxes.size(), globalBox });


        while (!stack.empty()) {
            auto range = stack.back();
            stack.pop_back();

            if (threeAxis) {
                BBox centroidBox;
                for (int i = range.begin; i < range.end; ++i) {
                    centroidBox.unionBBoxCentroid(centroids[primIndex[i]]);
                }

                float totalMinCost = FLT_MAX;
                BBox ansLeft, ansRight;
                int ansSplitPos;

                if (binCount > 1) {
                    int splitAxis = 0;

                    singleAxisSplit(binCount, maxPrimsInNode, bboxes, centroids, splitAxis,
                        primIndex.begin() + range.begin, primIndex.begin() + range.end,
                        ansSplitPos, centroidBox, range.box, ansLeft, ansRight, totalMinCost, FLT_MAX
                    );
                    /*std::cout << ansSplitPos << " (";
                    for (int i = 0; i < 3; ++i) std::cout << ansLeft.bbmin.s[i] << " ";
                    std::cout << ") (";
                    for (int i = 0; i < 3; ++i) std::cout << ansLeft.bbmax.s[i] << " ";
                    std::cout << "||| ";
                    for (int i = 0; i < 3; ++i) std::cout << ansRight.bbmin.s[i] << " ";
                    std::cout << ") (";
                    for (int i = 0; i < 3; ++i) std::cout << ansRight.bbmax.s[i] << " ";
                    std::cout << std::endl;*/

                    BBox leftBox, rightBox;
                    int splitPos;
                    float minCost;

                    splitAxis = 1;
                    singleAxisSplit(binCount, maxPrimsInNode, bboxes, centroids, splitAxis,
                        primIndex.begin() + range.begin, primIndex.begin() + range.end,
                        splitPos, centroidBox, range.box, leftBox, rightBox, minCost, totalMinCost
                    );
                    if (minCost < totalMinCost) {
                        totalMinCost = minCost;
                        ansSplitPos = splitPos;
                        ansLeft = leftBox;
                        ansRight = rightBox;
                    }

                    splitAxis = 2;
                    singleAxisSplit(binCount, maxPrimsInNode, bboxes, centroids, splitAxis,
                        primIndex.begin() + range.begin, primIndex.begin() + range.end,
                        splitPos, centroidBox, range.box, leftBox, rightBox, minCost, totalMinCost
                    );
                    if (minCost < totalMinCost) {
                        totalMinCost = minCost;
                        ansSplitPos = splitPos;
                        ansLeft = leftBox;
                        ansRight = rightBox;
                    }
                }
                else {
                    std::vector<uint32_t> bakIndices(primIndex.begin() + range.begin, primIndex.begin() + range.end);

                    int splitAxis = 0;
                    int splitPos;


                    singleAxisSplit(binCount, maxPrimsInNode, bboxes, centroids, splitAxis,
                        bakIndices.begin(), bakIndices.end(),
                        ansSplitPos, centroidBox, range.box, ansLeft, ansRight, totalMinCost, FLT_MAX
                    );

                    BBox leftBox, rightBox;
                    float minCost;

                    auto buffer = bakIndices;
                    splitAxis = 1;
                    singleAxisSplit(binCount, maxPrimsInNode, bboxes, centroids, splitAxis,
                        buffer.begin(), buffer.end(),
                        splitPos, centroidBox, range.box, leftBox, rightBox, minCost, totalMinCost
                    );
                    if (minCost < totalMinCost) {
                        totalMinCost = minCost;
                        ansSplitPos = splitPos;
                        ansLeft = leftBox;
                        ansRight = rightBox;
                        bakIndices.swap(buffer);
                    }

                    splitAxis = 2;
                    singleAxisSplit(binCount, maxPrimsInNode, bboxes, centroids, splitAxis,
                        buffer.begin(), buffer.end(),
                        splitPos, centroidBox, range.box, leftBox, rightBox, minCost, totalMinCost
                    );
                    if (minCost < totalMinCost) {
                        totalMinCost = minCost;
                        ansSplitPos = splitPos;
                        ansLeft = leftBox;
                        ansRight = rightBox;
                        bakIndices.swap(buffer);
                    }

                    std::copy(bakIndices.begin(), bakIndices.end(), primIndex.begin() + range.begin);
                }

                MultiPrimBVHNode tNode;
                uint32_t nowID = nodes.size();
                tNode.bbmin = range.box.bbmin;
                tNode.bbmax = range.box.bbmax;
                tNode.parent = range.parentID;

                nodes.push_back(tNode);

                if (range.isLeft)
                    nodes[range.parentID].left = nowID;
                else
                    nodes[range.parentID].right = nowID;

                auto& tNode2 = nodes.back();

                if (ansSplitPos < 0) {
                    tNode2.isLeaf = 1;
                    tNode2.left = range.begin;
                    tNode2.right = range.end;
                }
                else {
                    tNode2.isLeaf = 0;
                    ansSplitPos += range.begin;

                    stack.push_back({
                        nowID, true,
                        range.begin, (uint32_t)ansSplitPos,
                        ansLeft
                        });
                    stack.push_back({
                        nowID, false,
                        (uint32_t)ansSplitPos, range.end,
                        ansRight
                        });
                }
            }
            else {
                BBox centroidBox;
                for (int i = range.begin; i < range.end; ++i) {
                    centroidBox.unionBBoxCentroid(centroids[primIndex[i]]);
                }

                int splitAxis = 0;
                float4 ra = centroidBox.bbmax - centroidBox.bbmin;
                float temp_ = ra.x;
                if (ra.y > ra.x) {
                    temp_ = ra.y;
                    splitAxis = 1;
                }
                if (ra.z > temp_) {
                    splitAxis = 2;
                }

                int splitPos = 0;
                BBox leftBox, rightBox;

                

                float minCost;
                singleAxisSplit(binCount, maxPrimsInNode, bboxes, centroids, splitAxis,
                    primIndex.begin() + range.begin, primIndex.begin() + range.end,
                    splitPos, centroidBox, range.box, leftBox, rightBox, minCost, FLT_MAX
                );

                MultiPrimBVHNode tNode;
                uint32_t nowID = nodes.size();
                tNode.bbmin = range.box.bbmin;
                tNode.bbmax = range.box.bbmax;
                tNode.parent = range.parentID;

                nodes.push_back(tNode);

                if (range.isLeft)
                    nodes[range.parentID].left = nowID;
                else
                    nodes[range.parentID].right = nowID;

                auto& tNode2 = nodes.back();

                if (splitPos < 0) {
                    tNode2.isLeaf = 1;
                    tNode2.left = range.begin;
                    tNode2.right = range.end;

                    auto toTest = AREA(tNode2.bbmin, tNode2.bbmax);
                    if (isinf(toTest)) {
                        std::cout << "ErrSplitNode: " << nodes.size() << " " << tNode2.left << " " << tNode2.right << std::endl;
                    }
                }
                else {
                    tNode2.isLeaf = 0;
                    splitPos += range.begin;

                    auto toTest = AREA(tNode2.bbmin, tNode2.bbmax);
                    if (isinf(toTest)) {
                        std::cout << "ErrSplit INNER Node: " << nodes.size() << " " << tNode2.left << " " << tNode2.right << std::endl;
                    }

                    stack.push_back({
                        nowID, true,
                        range.begin, (uint32_t)splitPos,
                        leftBox
                        });
                    stack.push_back({
                        nowID, false,
                        (uint32_t)splitPos, range.end,
                        rightBox
                        });
                }
            }
        }
        nodes[0].parent = -1;

        indices.resize(primIndex.size());
        for (int i = 0; i < primIndex.size(); ++i) {
            indices[i] = indices_[primIndex[i]];
        }


        {
            std::function<BoundingBox(int)> combBox;
            combBox = [&](int curID) -> BoundingBox {
                if (nodes[curID].isLeaf) {
                    return { nodes[curID].bbmin, nodes[curID].bbmax };
                }
                else {
                    auto lBox = combBox(nodes[curID].left);
                    auto rBox = combBox(nodes[curID].right);
                    auto selfBox = BoundingBox::unionBBox(lBox, rBox);
                    nodes[curID].bbmin = selfBox.bbmin;
                    nodes[curID].bbmax = selfBox.bbmax;
                    return selfBox;
                }
            };
            combBox(0);
        }
        /*{
            auto leaves = getLeafIDs();
            std::vector<int> flags(nodes.size());
            for (auto id : leaves) {
                auto nowParent = nodes[id].parent;
            AGAIN:
                if (!flags[nowParent]) {
                    flags[nowParent] = 1;
                    continue;
                }
                auto left = nodes[id].left;
                auto right = nodes[id].right;
                auto lBox = BoundingBox{ nodes[left].bbmin,nodes[right].bbmax };
                auto rBox = BoundingBox{ nodes[right].bbmin, nodes[right].bbmax };
                auto unionBox = BoundingBox::unionBBox(lBox, rBox);
                if (len(unionBox.bbmin - nodes[nowParent].bbmin) >= 0.001
                    || len(unionBox.bbmax - nodes[nowParent].bbmax) >= 0.001
                    ) {
                    //std::cout << "Err Node: " << nowParent << std::endl;
                    nodes[nowParent].bbmin = unionBox.bbmin;
                    nodes[nowParent].bbmax = unionBox.bbmax;
                }
                nowParent = nodes[nowParent].parent;
                goto AGAIN;
            }
        }*/
    }
}