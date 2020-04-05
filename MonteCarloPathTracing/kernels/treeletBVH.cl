
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
			if(atomic_cmpxchg(flags+idx,0,1) != 1) { // another child not ready
				return;
			}
			idx = nodes[idx].parent;
			if( atomic_cmpxchg(flags+idx,0,1) == 1 ) {
				int lc = nodes[idx].left;
				int rc = nodes[idx].right;

				sahValue[idx] = sahValue[lc] + sahValue[rc] + 
					Cinn * (AREA(bvh[idx].bbmin,bvh[idx].bbmax) ) / rootArea;
			}
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
	int sp = 1;
	queueNode[sp].id = idx;
	queueNode[sp].sahValue = sahValue[idx];

	int nextFree = 0;

	while(sp < MAX_NODE) {
		maxNodeID = 0;
		for(int i = 1;i < sp;++i) {
			if(queueNode[i].sahValue > queueNode[maxNodeID].sahValue) {
				maxNodeID = i;
			}
		}
		// find max node
		if( queueNode[maxNodeID].sahValue < 0.0f) {
			break;
		}
		int id = queueNode[ maxNodeID ].id;
		int left = nodes[id].left;
		int right = nodes[id].right;

		if(left == right) {
			queueNode[maxNodeID].sahValue = -queueNode[maxNodeID].sahValue;
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

	int nowID = nowsize-1;
	while(mask > 0) {
		if(mask&0x1) {
			boxmin = min(bbmin[nowID],boxmin);
			boxmax = max(bbmax[nowID],boxmax);
		}
		--nowID;
		mask>>=1;
	}
	return AREA(boxmin,boxmax);
}

__kernel void reconstructTreelet(
	__global BVHNode* nodes,
	__global float* sahValue,
	__global volatile int* flags,
	uint numPrims,

	__local float* a, // 128 * 4=512bytes
	__local float* copt, // 128 * 4 = 512bytes
	__local char* popt, // 128 * 1 = 128bytes

	__local float* cbuffer // (32+1) * 4 = 128bytes+4
) {
	size_t groupID = get_group_id(0);

	size_t lid = get_local_id(0);
	size_t lsize = get_local_size(0); // must be 32
	if(lsize != WARP_SIZE) {
		if(groupID == 0 && lid == 0)
			printf("Local size must be WARP_SIZE %d\n",WARP_SIZE);
		return;
	}

	const char roundConstant[5][32] = {
		{},
		{},
		{},
		{},
		{}
	};
	const char roundSixConstant[7] = {

	};

	if(groupID < numPrims) {
		int idx = gid + numPrims - 1;

		do {
			if(atomic_cmpxchg(flags+idx,0,1) != 1) { // another child not ready
				return;
			}
			// Firstly: get nodes to reconstruct
			PickQueueNode queueNode[MAX_NODE] = {0};
			float4 nodebbmax[MAX_NODE];
			float4 nodebbmin[MAX_NODE];
			int freeBVHNode[MAX_NODE-1];
			int size = pickNode(nodes,idx,sahValue,queueNode,freeBVHNode,nodebbmin,nodebbmax);
			if(size < 2) goto NEXT;

			int NOW_NODE = size;
			int NOW_BIT = (1<<size)-1;

			size_t parCount = ( (1<<NOW_NODE)-1 ) / WARP_SIZE +1;
			size_t begin = lid * partCount;

			// Secondly: calculate area
			for(int i=begin;i<begin+parCount;++i) {
				a[i] = calcUnionArea(nodebbmin,nodebbmax,NOW_NODE,i);
			}
			if(lid < NOW_NODE) {
				copt[ 1<<lid ] = sahValue[ queueNode[lid].id ];
			}

			// now deal with it here

			for(int i=0;i<5;++i) {
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
					p = (p-delta) & s;
				} while(p!=0);
				
				copt[part] = cs;
				popt[part] = ps;
			}
			// Then 6 & 7
			// 6: partition count == 31
			for(int i=0;i<7;++i) {
				if( lid < 31 ) {
					int x = roundSixConstant[i];
					int part = lid;
					
					float c = copt[part]+copt[part^x];
					cbuffer[lid] = c;
					// reduce here
					copt[x] = #xxxx;
					popt[x] = #xxxx;
				}
			}

			// 7： partition count == 63
			
			// Then reconstruct


			// Refresh the SAH
			if(lid < size) {
				int nowBVHID = nodes[ queueNode[lid].id ].parent;
				while(true) {
					// 
					nodes[nowBVHID].bbmin = #xxxx;
					nodes[nowBVHID].bbmax = #xxxx;
					sahValue[nowBVHID] = #xxxx;
					if(nowBVHID == idx) {
						break;
					}
					nowBVHID = nodes[nowBVHID].parent;
				}
			}
		NEXT:
			idx = nodes[idx].parent;
		} while(idx != -1);
	}
}
