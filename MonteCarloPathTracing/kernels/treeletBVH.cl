
#define Cinn (1.2f)
#define Ctri (1.0f)
#define Cleaf (0.0f)

#define MAX_NODE (7)
#define TOTAL_BIT (0x0000007F)

inline float AREA(float4 bbmin,float4 bbmax) {
	float4 arg = bbmax - bbmin;
	return 2.0f * (arg.x * arg.y + arg.x * arg.z + arg.y * arg.z);
}


__kernel void calculateSAH(
	__global BVHNode* nodes,
	__global float* sahValue,
	__global int* flags,
	uint numPrims
) {
	size_t gid = get_global_id(0);
	float rootArea = AREA(nodes[0].bbmin,nodes[0].bbmax);

	if(gid < numPrims) {
		int idx = gid+numPrims-1;

		sahValue[idx] = (Ctri + Cleaf) * AREA(nodes[idx].bbmin, nodes[idx].bbmax) / rootArea;

		do {
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
	int* freeBVHNode
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
	return sp;
}



__kernel void reconstructTreelet(
	__global BVHNode* nodes,
	__global float* sahValue,
	__global int* flags,
	uint numPrims,

	__local float* a
) {
	size_t groupID = get_group_id(0);

	if(groupID < numPrims) {
		int idx = gid + numPrims - 1;

		do {
			
			// Firstly: get nodes to reconstruct
			PickQueueNode queueNode[MAX_NODE] = {0};
			int freeBVHNode[MAX_NODE-1];
			int size = pickNode(nodes,idx,sahValue,queueNode,freeBVHNode);
			if(size < MAX_NODE) {
				goto NEXT;
			}
			int NOW_NODE = MAX_NODE;
			int NOW_BIT = TOTAL_BIT;

			// Secondly: calculate area





		NEXT:
			idx = nodes[idx].parent;
		} while(idx != -1);
	}

	

}
