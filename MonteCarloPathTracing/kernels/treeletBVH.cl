
#define Cinn (1.2f)
#define Ctri (1.0f)
#define Cleaf (0.0f)

#define MAX_NODE (7)
#define TOTAL_BIT (0x0000007F)

#define WARP_SIZE (32)


inline float AREA(float4 bbmin,float4 bbmax) {
	float4 arg = bbmax - bbmin;
	return 2.0f * (arg.x * arg.y + arg.x * arg.z + arg.y * arg.z);
}

// MAYBE COMBINE WITH RECONSTRUCTION
__kernel void calculateSAH(
	__global BVHNode* nodes,
	__global float* sahValue,
	__global volatile int* flags,
	uint numPrims
) {
	size_t gid = get_global_id(0);
	float rootArea = AREA(nodes[0].bbmin,nodes[0].bbmax);

	if(gid < numPrims) {
		int idx = gid+numPrims-1;

		sahValue[idx] = (Ctri + Cleaf) * AREA(nodes[idx].bbmin, nodes[idx].bbmax) / rootArea;

		do {
			idx = nodes[idx].parent;
			if(atomic_cmpxchg(flags+idx,0,1) != 1)  // another child not ready
				return;
		
			int lc = nodes[idx].left;
			int rc = nodes[idx].right;

			sahValue[idx] = sahValue[lc] + sahValue[rc] + 
				Cinn * (AREA(nodes[idx].bbmin,nodes[idx].bbmax) ) / rootArea;
		} while(idx != 0);
	}
}







typedef struct PickQueueNode_tag {
	int id;
	float sahValue;
} PickQueueNode;









int pickNode(
	__global BVHNode* nodes,
	int idx,
	__global float* sahValue,
	PickQueueNode* queueNode,
	int* freeBVHNode,

	float4* bbmin,
	float4* bbmax
) {
	int lid = get_local_id(0);
	int sp = 1;
	queueNode[0].id = idx;
	queueNode[0].sahValue = sahValue[idx];

	__local volatile float sahvbuffer[MAX_NODE+1];
	__local int maxNodeID;

	int nextFree = 0;

	while(sp < MAX_NODE) {

		// find max node
		if(sp == 1 && lid == 0) {
			maxNodeID = 0;
			goto AFTERREDUCE;
		} //##MARK

		if(lid < sp) {
			sahvbuffer[lid] = queueNode[lid].sahValue;
		}
		else if(lid < MAX_NODE+1) {
			sahvbuffer[lid] = -FLT_MAX;
		}

		if(lid < 4) {
			if(sp < 4) {
				sahvbuffer[lid] = max(sahvbuffer[lid],sahvbuffer[lid+2]);
				sahvbuffer[lid] = max(sahvbuffer[lid],sahvbuffer[lid+1]);
			}
			else {
				sahvbuffer[lid] = max(sahvbuffer[lid],sahvbuffer[lid+4]);
				sahvbuffer[lid] = max(sahvbuffer[lid],sahvbuffer[lid+2]);
				sahvbuffer[lid] = max(sahvbuffer[lid],sahvbuffer[lid+1]);
			}
			if(sahvbuffer[lid] == queueNode[lid].sahValue) {
				maxNodeID = lid;
			}
		}

		AFTERREDUCE:

		if( queueNode[maxNodeID].sahValue < 0.0f) {
			break;
		}
		int id = queueNode[ maxNodeID ].id;
		int left = nodes[id].left;
		int right = nodes[id].right;

		if(left == right) {
			queueNode[maxNodeID].sahValue = -1.0f;
			continue;
		} // if leaf & max, ignore
		else {
			queueNode[maxNodeID].id = left;
			queueNode[maxNodeID].sahValue = sahValue[left];
			queueNode[sp].id = right;
			queueNode[sp].sahValue = sahValue[right];
			++sp;
			freeBVHNode[nextFree++] = id;
		}
	}
	for(int i=0;i<sp;++i) {
		bbmin[i] = nodes[ queueNode[i].id ].bbmin;
		bbmax[i] = nodes[ queueNode[i].id ].bbmax;
	}
	return sp;
}

inline float calcUnionArea(
	float4* bbmin,
	float4* bbmax,

	int nowsize,
	int mask
) {
	float4 boxmin = (float4)(FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX);
	float4 boxmax = (float4)(-FLT_MAX,-FLT_MAX,-FLT_MAX,-FLT_MAX);
	if(mask == 0) return 0;

	int nowID = nowsize-1;
	while(mask > 0 && nowID >= 0) {
		if(mask&0x1) {
			boxmin = min(bbmin[nowID],boxmin);
			boxmax = max(bbmax[nowID],boxmax);
		}
		--nowID;
		mask>>=1;
	}
	return AREA(boxmin,boxmax);
}



typedef struct SplitInnerNode_tag {
	int parentCode;
	int selfCode;

	int parentID;
} SplitInnerNode;

inline int Count1Num(int x)  {
	int ans = 0;
	while (x > 0) {
		ans += (x & 0x1);
		x >>= 1;
	}
	return ans;
}


/*
	2:   3, 5, 6, 9, 10, 12, 17, 18, 20, 24, | 33, 34, 36, 40, 48, 65, 66, 68, 72, 80, 96
	3:   7, 11, 13, 14, 19, 21, 22, 25, 26, | 28, 35, 37, 38, 41, 42, 44, 49, 50, 52, 56, 67, 69, 70, 73, 74, 76, 81, 82, 84, 88, 97, 98, 100, 104, 112
	4:   15, 23, 27, | 29, 30, 39, 43, 45, 46, 51, 53, 54, 57, 58, 60, 71, 75, 77, 78, 83, 85, 86, 89, 90, 92, 99, 101, 102, 105, 106, 108, 113, 114, 116, 120
	5:   31, 47, 55, 59, 61, 62, 79, 87, 91, 93, 94, 103, 107, 109, 110, 115, 117, 118, 121, 122, 124
*/

__constant const char roundConstant[5][32] = {
	{3, 5, 6, 9, 10, 12, 17, 18, 20, 24},
	{7, 11, 13, 14, 19, 21, 22, 25, 26,  /*<-3|2->*/ 33, 34, 36, 40, 48, 65, 66, 68, 72, 80, 96 },
	{15, 23, 27, /*4|3*/ 28, 35, 37, 38, 41, 42, 44, 49, 50, 52, 56, 67, 69, 70, 73, 74, 76, 81, 82, 84, 88, 97, 98, 100, 104, 112 },
	{29, 30, 39, 43, 45, 46, 51, 53, 54, 57, 58, 60, 71, 75, 77, 78, 83, 85, 86, 89, 90, 92, 99, 101, 102, 105, 106, 108, 113, 114, 116, 120},
	{31, 47, 55, 59, 61, 62, 79, 87, 91, 93, 94, 103, 107, 109, 110, 115, 117, 118, 121, 122, 124}
};

__constant const int constLen[] = {
	10,20,29,32,21
};

__constant const char roundSixConstant[7] = {
	63, 95, 111, 119, 123, 125, 126
};
__constant const char roundSixPart[7][32] = {
	{2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48,
			50, 52, 54, 56, 58, 60, 62}, // 63
	{2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 64, 66, 68, 70, 72, 74, 76, 78, 80,
			82, 84, 86, 88, 90, 92, 94}, //95
	{2, 4, 6, 8, 10, 12, 14, 32, 34, 36, 38, 40, 42, 44, 46, 64, 66, 68, 70, 72, 74, 76, 78, 96,
			98, 100, 102, 104, 106, 108, 110}, //111
	{2, 4, 6, 16, 18, 20, 22, 32, 34, 36, 38, 48, 50, 52, 54, 64, 66, 68, 70, 80, 82, 84, 86, 96,
			98, 100, 102, 112, 114, 116, 118}, //119
	{2, 8, 10, 16, 18, 24, 26, 32, 34, 40, 42, 48, 50, 56, 58, 64, 66, 72, 74, 80, 82, 88, 90, 96,
			98, 104, 106, 112, 114, 120, 122}, //123
	{4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96,
			100, 104, 108, 112, 116, 120, 124}, //125
	{4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96,
			100, 104, 108, 112, 116, 120, 124} //126
};
__constant const char roundSeven[64] = {
	2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50,
		52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98,
		100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126
};

__kernel void reconstructTreelet(
	__global BVHNode* nodes,
	__global float* sahValue,
	__global volatile int* flags,
	int numPrims
) {
	__local volatile float a[128];        // 128 * 4 = 512bytes
	__local volatile float copt[128];     // 128 * 4 = 512bytes
	__local volatile char popt[128];      // 128 * 1 = 128bytes

	__local volatile float cbuffer[64];   // (64) * 4 = 256bytes
	__local volatile bool terminalFlag;

	__local volatile SplitInnerNode toSplitBuffer[MAX_NODE]; // (7*3*4=48 bytes)
	__local volatile int prefixSum[MAX_NODE]; //(7*4=28bytes)
	__local volatile int2 SP_Free;

	float rootArea = AREA(nodes[0].bbmin,nodes[0].bbmax);

	
	size_t groupID = get_group_id(0);

	size_t lid = get_local_id(0);
	size_t lsize = get_local_size(0); // must be 32
	if(lsize != WARP_SIZE) {
		if(groupID == 0 && lid == 0)
			printf("Local size must be WARP_SIZE %d\n",WARP_SIZE);
		return;
	}

	if(groupID < numPrims) {
#ifndef MCPT_ONE_WORKGROUP
		int idx = groupID + numPrims - 1;
		sahValue[idx] = (Ctri + Cleaf) * AREA(nodes[idx].bbmin, nodes[idx].bbmax) / rootArea;
		idx = nodes[idx].parent;
#else
		int idx = 0;
		if(groupID != 0) {
			return;
		}
#endif
		while(idx >= 0) {
			if(lid == 0) {
				int lc = nodes[idx].left;
				int rc = nodes[idx].right;

				sahValue[idx] = sahValue[lc] + sahValue[rc] + 
					Cinn * (AREA(nodes[idx].bbmin,nodes[idx].bbmax) ) / rootArea;

				terminalFlag = false;
#ifndef MCPT_ONE_WORKGROUP
				if(atomic_cmpxchg(flags+idx,0,1) != 1) { // another child not ready
					terminalFlag = true;
				}
#endif
			}
			if(terminalFlag) { return; }


			// Firstly: get nodes to reconstruct
			PickQueueNode queueNode[MAX_NODE] = {};
			float4 nodebbmax[MAX_NODE];
			float4 nodebbmin[MAX_NODE];
			int freeBVHNode[MAX_NODE-1];

			
			int size = pickNode(nodes,idx,sahValue,queueNode,freeBVHNode,nodebbmin,nodebbmax);
			
			if(size < 3) {
				idx = nodes[idx].parent;
				continue;
			}
			
			int NOW_NODE = size;
			int NOW_BIT = (1<<size)-1;

			size_t partCount = ( (1<<NOW_NODE)-1 ) / WARP_SIZE +1;
			size_t begin = lid * partCount;
			// Secondly: calculate area
			{
				for(int i=begin;(i<begin+partCount)&&(i<(1<<NOW_NODE));++i) {
					a[i] = calcUnionArea(nodebbmin,nodebbmax,NOW_NODE,i);
				}
				if(lid < NOW_NODE) {
					copt[ 1<<lid ] = sahValue[ queueNode[lid].id ];
				}
			}
			
			// now deal with it here
			for(int i=0;i<5 && i < NOW_NODE;++i) {
				if(lid >= constLen[i]) continue;
				int part = roundConstant[i][lid];
				float cs = FLT_MAX;
				int ps = 0;

				int delta = (part - 1) & part;
				int p = (-delta) & part;

				do {
					float c = copt[p]+copt[part ^ p];
					if(c < cs) {
						cs = c;
						ps = p;
					}
					p = (p-delta) & part;
				} while(p!=0);
				
				copt[part] = cs + Cinn * a[part];
				popt[part] = ps;
			}
			// Then 6 & 7
			// 6: partition count == 31

			if(NOW_NODE < 6) goto RECONSTRUCT;

			for(int i=0;i<7;++i) {
				float c;
				int part;
				if(lid < 31) {
					part = roundSixPart[i][lid];
					c = copt[part]+copt[part^roundSixConstant[i]]; //##MARK
					cbuffer[lid]=c;
				}
				else {
					cbuffer[31] = c = FLT_MAX;
				}
				if(lid < 16) {
					cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+16]);
					cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+8]);
					cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+4]);
					cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+2]);
					cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+1]);
					
				}
				if(lid < 31 && c == cbuffer[0]) {
					copt[roundSixConstant[i]] = cbuffer[0] + Cinn * a[roundSixConstant[i]];
					popt[roundSixConstant[i]] = part;
				}
			}

			if(NOW_NODE < 7) goto RECONSTRUCT;
			
			// 7： partition count == 63
			{
				int toCalc1 = roundSeven[ lid<<1 ];
				int toCalc2 = roundSeven[ (lid<<1)+1];
				
				float c1 = copt[toCalc1] + copt[(127-toCalc1)];
				cbuffer[lid<<1] = c1;
				float c2 = copt[toCalc2] + copt[(127-toCalc2)];
				cbuffer[(lid<<1)+1] = c2;

				if(toCalc2 == 0) {
					cbuffer[63] = FLT_MAX;
				}

				cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+32]);
				cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+16]);
				cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+8]);
				cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+4]);
				cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+2]);
				cbuffer[lid] = min(cbuffer[lid],cbuffer[lid+1]);
				
				if(c1 == cbuffer[0]) {
					copt[127] = c1 + Cinn * a[127];
					popt[127] = toCalc1;
				}
				else if(c2 == cbuffer[0]) {
					copt[127] = c2 + Cinn * a[127];
					popt[127] = toCalc2;
				}
			}
			RECONSTRUCT:
			/*if(lid==0&&groupID==0) {
				printf("Reconstruct %d\n", idx);
				printf("nownode: ");
				for(int i=0;i<NOW_NODE;++i) {
					printf("%d ",queueNode[i].id);
				}
				printf("\nfreenode: ");
				for(int i=0;i<NOW_NODE-1;++i) {
					printf("%d ",freeBVHNode[i]);
				}
				printf("\n");
			}*/
			if(lid == 0) {
				SP_Free = (int2)(1,1);
				toSplitBuffer[0].parentCode = NOW_BIT;
				toSplitBuffer[0].selfCode = popt[NOW_BIT];
				toSplitBuffer[0].parentID = freeBVHNode[0];
			}
			/*if(lid==0&&groupID==0)
				printf("%d %d\n",NOW_NODE, NOW_BIT);*/
			while (SP_Free.x > 0) {
				/*if(lid==0&&groupID==0)
					printf("(%d %d)\n",SP_Free.x,SP_Free.y);*/
				if(lid < SP_Free.x) {
					SplitInnerNode i = toSplitBuffer[lid];
					int leftCode = popt[i.selfCode];
					int rightCode = popt[i.selfCode ^ i.parentCode];
					int parentNodeID = i.parentID;

					/*if(groupID==0&&lid==0) {
						printf("%d:",lid);
						printf("%d ",i.parentCode);
						printf("%d ",i.selfCode);
						printf("%d\n",i.parentID);
					}
					if(groupID==0&&lid==1) {
						printf("%d:",lid);
						printf("%d ",i.parentCode);
						printf("%d ",i.selfCode);
						printf("%d\n",i.parentID);
					}*/

					SplitInnerNode temp;
					if(Count1Num(i.selfCode) == 1) {
						int toRight = 31-clz(i.selfCode);
						int node = queueNode[NOW_NODE-toRight-1].id;
						nodes[parentNodeID].left = node;
						nodes[node].parent = parentNodeID;

						temp.parentID = -1;
					}
					else {
						temp.parentCode = i.selfCode;
						temp.selfCode = leftCode;
						temp.parentID = parentNodeID;
					}
					toSplitBuffer[(lid<<1)] = temp;

					if(Count1Num(i.selfCode ^ i.parentCode) == 1) {
						int toRight = 31 - clz(i.selfCode ^ i.parentCode);
						int node = queueNode[NOW_NODE-toRight-1].id;
						nodes[parentNodeID].right = node;
						nodes[node].parent = parentNodeID;

						temp.parentID = -1;
					}
					else {
						temp.parentCode = i.selfCode ^ i.parentCode;
						temp.selfCode = rightCode;
						temp.parentID = parentNodeID;
					}
					toSplitBuffer[(lid<<1)+1] = temp;
				}
				// split -> 2*node
				// if leaf, parentID == -1
				// else, parentID == parentNodeID to write
				/*if(lid==0&&groupID==0) {
					printf("toSplit: %d\n", SP_Free.x);
					for(int i=0;i<SP_Free.x*2;++i) {
						printf("%d %d %d\n",toSplitBuffer[i].parentCode,toSplitBuffer[i].selfCode,toSplitBuffer[i].parentID);
					}
				}*/
				if(lid == 0) {
					prefixSum[0] = (toSplitBuffer[0].parentID>=0?1:0);
					for(int i = 1; i < (SP_Free.x<<1); ++i) {
						prefixSum[i] = prefixSum[i-1] + (toSplitBuffer[i].parentID>=0?1:0);
					}
				}
				int allocCount = prefixSum[(SP_Free.x<<1)-1];

				if(lid < (SP_Free.x<<1)) {
					prefixSum[lid] -= (toSplitBuffer[lid].parentID>=0?1:0); // now exclusive
					/*if(lid==0&&groupID==0) {
						printf("alloc: %d\n",allocCount);
						printf("%d-expref:",lid);
						for(int x=0;x<(SP_Free.x<<1);++x) {
							printf("%d ",prefixSum[x]);
						}
						printf("\n");
					}*/
					SplitInnerNode i = toSplitBuffer[lid];
					if(i.parentID>=0) {
						int selfNodeID = freeBVHNode[ SP_Free.y + prefixSum[lid] ];
						if(lid&0x1)
							nodes[ i.parentID ].right = selfNodeID;
						else
							nodes[ i.parentID ].left = selfNodeID;
						nodes[ selfNodeID ].parent = i.parentID;
						
						i.parentID = selfNodeID;
						toSplitBuffer[ prefixSum[lid] ] = i;
						// shrink
					}
				}
				if(lid == 0) {
					SP_Free.x = allocCount;
					SP_Free.y += allocCount;
				}
			}
			// Refresh the SAH
			if(lid == 0) {
				int sFlag[MAX_NODE-1] = {0};
				int ssp = 0;
				int rootOver = 0;
				for(int i=0;i<NOW_NODE;++i) {
					int nowNode = queueNode[i].id;
					while(true) {
						nowNode = nodes[nowNode].parent;
						__global BVHNode* node = nodes+nowNode;

						int ready = 0;
						for(int j=0;j<ssp;++j) {
							if(sFlag[j] == nowNode) {
								ready = 1;
								break;
							}
						}
						if(!ready) {
							sFlag[ssp++] = nowNode;
							break;
						}

						node->bbmin = min(nodes[node->left].bbmin,nodes[node->right].bbmin);
						node->bbmax = max(nodes[node->left].bbmax,nodes[node->right].bbmax);
						sahValue[nowNode] = sahValue[node->left]+sahValue[node->right]
							+ Cinn*(AREA(node->bbmin,node->bbmax));
						if(nowNode == freeBVHNode[0]) {
							break;
						}
					}
				}
			}
			idx = nodes[idx].parent;
		}
	}
}


__kernel void combineLeaf(
	__global BVHNode* nodes,
	__global float* sahValue,
	__global volatile int* flags,
	int numPrims
) {
	size_t gid = get_global_id(0);

	if(gid < numPrims) {
		int idx = gid+numPrims-1;

		//sahValue[idx] = (Ctri + Cleaf) * AREA(nodes[idx].bbmin, nodes[idx].bbmax) / rootArea;

		do {
			idx = nodes[idx].parent;
			if(atomic_cmpxchg(flags+idx,0,1) != 1)  // another child not ready
				return;

			//if( sahValue[idx] > Ctri * AREA(nodes[idx].bbmin, nodes[idx].bbmax) * (##MARK) ) {
			//	##colapse
			//}

			//int lc = nodes[idx].left;
			//int rc = nodes[idx].right;

			//sahValue[idx] = sahValue[lc] + sahValue[rc] + 
			//	Cinn * (AREA(nodes[idx].bbmin,nodes[idx].bbmax) ) / rootArea;
		} while(idx != 0);
	}
}