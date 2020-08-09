#define Ctri (1.0f)
#define Cinn (1.2f)

typedef struct MyVec3_tag {
	float s[3];
} MyVec3;

inline void initVec3(MyVec3* vec, float3 value) {
	vec->s[0] = value.x;
	vec->s[1] = value.y;
	vec->s[2] = value.z;
}
inline float3 getValue(MyVec3* vec) {
	float3 ans;
	ans.x = vec->s[0];
	ans.y = vec->s[1];
	ans.z = vec->s[2];
	return ans;
}
inline MyVec3 myVecMinus(MyVec3* a,MyVec3* b) {
	MyVec3 ans;
	ans.s[0] = a->s[0]-b->s[0];
	ans.s[1] = a->s[1]-b->s[1];
	ans.s[2] = a->s[2]-b->s[2];
	return ans;
}


int pointInBox(float3 p, float3 bbmin, float3 bbmax) {
	if( 
		p.x >= bbmin.x && p.x <= bbmax.x &&
		p.y >= bbmin.y && p.y <= bbmax.y &&
		p.z >= bbmin.z && p.z <= bbmax.z
	) {
		return 1;
	}
	else {
		return 0;
	}
}


void roundTr(MyVec3* points, int* size, int axis, float pos, int arg) {
	if(*size == 0) return;
	
	MyVec3 buffer[8];
	int bSize = *size;
	for(int i=0;i<bSize;++i) {
		buffer[i] = points[i];
	}
	*size = 0;

	int inside[8];
	if(arg > 0) {
		for(int i=0;i<bSize;++i) {
			inside[i] = (buffer[i].s[axis] >= pos ? 1 : 0);
		}
	}
	else {
		for(int i=0;i<bSize;++i) {
			inside[i] = (buffer[i].s[axis] <= pos ? 1 : 0);
		}
	}

	for(int i=0;i<bSize;++i) {
		int i_1 = ((i+1)==bSize?(0):(i+1));
		if(!inside[i]&&!inside[i_1]) {
			continue;
		}
		else if(inside[i] && inside[i_1]) {
			points[*size] = buffer[i];
			++(*size);
			continue;
		}
		else {
			if(inside[i]) {
				points[*size] = buffer[i];
				++(*size);
			}
			MyVec3 dir = myVecMinus(buffer+i_1,buffer+i);
			float tans = (pos - buffer[i].s[axis])/dir.s[axis];
			float3 finalPoint = getValue(buffer+i)+tans*getValue(&dir);
			initVec3(&dir,finalPoint);
			points[*size] = dir;
			++(*size);
		}
	}
}


float pArea(MyVec3* points, int size) {
	float ans = 0.0f;
	if(size < 2) return ans;
	for(int i = 1; i < size - 1; ++i) {
		float3 x1 = getValue(points+i) - getValue(points);
		float3 x2 = getValue(points+(i+1)) - getValue(points);
		ans += length(cross(x1,x2)) * 0.5f;
	}
	return ans;
}

float INTERSECT(Triangle* tr,float4 bbmin,float4 bbmax) {
	int inside[3];
	inside[0] = pointInBox(tr->v[0].s012,bbmin.s012,bbmax.s012);
	inside[1] = pointInBox(tr->v[1].s012,bbmin.s012,bbmax.s012);
	inside[2] = pointInBox(tr->v[2].s012,bbmin.s012,bbmax.s012);

	if(inside[0] && inside[1] && inside[2]) {
		float3 e1 = tr->v[1].s012 - tr->v[0].s012;
		float3 e2 = tr->v[2].s012 - tr->v[0].s012;
		return length(cross(e1,e2)) * 0.5f;
	}

	MyVec3 points[8];
	int nowSize = 3;

	initVec3(points+0,tr->v[0].s012);
	initVec3(points+1,tr->v[1].s012);
	initVec3(points+2,tr->v[2].s012);

	roundTr(points,&nowSize, 0, bbmin.x, 1);
	roundTr(points,&nowSize, 1, bbmin.y, 1);
	roundTr(points,&nowSize, 2, bbmin.z, 1);

	roundTr(points,&nowSize, 0, bbmax.x, -1);
	roundTr(points,&nowSize, 1, bbmax.y, -1);
	roundTr(points,&nowSize, 2, bbmax.z, -1);

	return pArea(points,nowSize);
}


__kernel void calculateEPO(
	
	__global BVHNode* bvh,
	__global Triangle* triangles,
	__global float* trianglesEPO,
	__global float* trianglesArea,

	uint numPrims
) {
	size_t gid = get_global_id(0);
	size_t myID = gid + numPrims - 1;

	Triangle tr = triangles[ bvh[myID].left ];

	float epo_area = 0.0f;

	int ancestor[256];
	ancestor[0] = myID;
	int anSize = 1;

	int nowParent = bvh[myID].parent;
	while(nowParent != -1) {
		ancestor[anSize++] = nowParent;
		nowParent = bvh[nowParent].parent;
	}
	if(anSize >= 256) printf("anSize overflow\n");

	// now ancestor get
	// next, hit with root

	int toTest[128];
	toTest[0] = 0;
	int testSize = 1;

	while(testSize > 0) {
		if(testSize >= 256) printf("testSize overflow");

		int nowTest = toTest[--testSize];
		bool skip = false;

		for(int i=0;i<anSize;++i) {
			if(nowTest == ancestor[i]) {
				if(bvh[nowTest].left != bvh[nowTest].right) {
					toTest[testSize++] = bvh[nowTest].right;
					toTest[testSize++] = bvh[nowTest].left;
				}
				skip = true;
				break;
			}
		}
		if(skip) continue;

		float tempArea = INTERSECT(&tr, bvh[nowTest].bbmin, bvh[nowTest].bbmax);
		if(tempArea > 0) {
			epo_area += tempArea * ((nowTest>=(numPrims-1))?Ctri:Cinn);
			if(bvh[nowTest].left != bvh[nowTest].right) {
				toTest[testSize++] = bvh[nowTest].right;
				toTest[testSize++] = bvh[nowTest].left;
			}
		}
	}
	
	trianglesEPO[gid] = epo_area;
	trianglesArea[gid] = length(cross(tr.v[1].s012-tr.v[0].s012,tr.v[2].s012-tr.v[0].s012)) * 0.5f;
}


__kernel void calculateEPO_Multi(
	__global BVHNode* bvh,
	__global uint* indices,
	__global uint* leafIDs,
	__global Triangle* triangles,

	__global float* trianglesEPO,
	__global float* trianglesArea
) {
	size_t gid = get_global_id(0);
	size_t nodeID = leafIDs[gid];

	float epo_area = 0.0f;
	float areaSum = 0.0f;

	int ancestor[256];
	int anSize;
	int myID;

	int toTest[256];
	int testSize;

	for (int i = bvh[nodeID].left; i < bvh[nodeID].right; ++i) {
		myID = indices[i];
		Triangle tr = triangles[ myID ];

		areaSum += length(cross(tr.v[1].s012 - tr.v[0].s012, tr.v[2].s012 - tr.v[0].s012)) * 0.5f;

		ancestor[0] = nodeID;
		anSize = 1;

		int nowParent = bvh[nodeID].parent;
		while (nowParent != -1) {
			ancestor[anSize++] = nowParent;
			nowParent = bvh[nowParent].parent;

			if (anSize >= 256) printf("anSize overflow\n");
		}
		// now ancestor get
		// next, hit with root
		toTest[0] = 0;
		testSize = 1;

		while (testSize > 0) {
			if (testSize >= 256) printf("testSize overflow");
			int nowTest = toTest[--testSize];
			bool skip = false;

			for (int i = 0; i < anSize; ++i) {
				if (nowTest == ancestor[i]) {
					if (!bvh[nowTest].isLeaf) {
						toTest[testSize++] = bvh[nowTest].right;
						toTest[testSize++] = bvh[nowTest].left;
					}
					skip = true;
					break;
				}
			}
			if (skip) continue;

			float tempArea = INTERSECT(&tr, bvh[nowTest].bbmin, bvh[nowTest].bbmax);
			if (tempArea > 0) {
				epo_area += tempArea * (  bvh[nowTest].isLeaf ? Ctri : Cinn);
				if (!bvh[nowTest].isLeaf) {
					toTest[testSize++] = bvh[nowTest].right;
					toTest[testSize++] = bvh[nowTest].left;
				}
			}
		}

	}
	

	trianglesEPO[gid] = epo_area;
	trianglesArea[gid] = areaSum;
}





__kernel void calculateQuadEPO(
	__global QuadBVHNode* bvh,
	__global Triangle* triangles,
	__global float* trianglesEPO,
	__global float* trianglesArea,

	uint offset
) {
	size_t gid = get_global_id(0);
	size_t myID = gid + offset;

	Triangle tr = triangles[ bvh[myID].children.x ];

	float epo_area = 0.0f;

	int ancestor[128];
	ancestor[0] = myID;
	int anSize = 1;

	int nowParent = bvh[myID].parent.w;
	while(nowParent != -1) {
		ancestor[anSize++] = nowParent;
		nowParent = bvh[nowParent].parent.w;
	}
	if(anSize >= 128) printf("anSize overflow\n");

	// now ancestor get
	// next, hit with root

	int toTest[128];
	toTest[0] = 0;
	int testSize = 1;

	while(testSize > 0) {
		if(testSize >= 128) printf("testSize overflow");

		int nowTest = toTest[--testSize];
		bool skip = false;

		int4 children = bvh[nowTest].children;

		for(int i=0;i<anSize;++i) {
			if(nowTest == ancestor[i]) {
				if(children.x != children.y) {
					toTest[testSize++] = children.x;
					if(children.y > 0) toTest[testSize++] = children.y;
					toTest[testSize++] = children.z;
					if(children.w > 0) toTest[testSize++] = children.w;
				}
				skip = true;
				break;
			}
		}
		if(skip) continue;

		float tempArea = INTERSECT(&tr, bvh[nowTest].bbmin, bvh[nowTest].bbmax);
		if(tempArea > 0) {
			int notEmptyChildrenCount = 4;
			if(children.y <= 0) --notEmptyChildrenCount;
			if(children.w <= 0) --notEmptyChildrenCount;

			epo_area += tempArea * ((nowTest>=(offset))?Ctri:(Cinn/2*notEmptyChildrenCount));

			if(children.x != children.y) {
				toTest[testSize++] = children.x;
				if(children.y > 0) toTest[testSize++] = children.y;
				toTest[testSize++] = children.z;
				if(children.w > 0) toTest[testSize++] = children.w;
			}
		}
	}
	
	trianglesEPO[gid] = epo_area;
	trianglesArea[gid] = length(cross(tr.v[1].s012-tr.v[0].s012,tr.v[2].s012-tr.v[0].s012)) * 0.5f;
}




bool improveIntersectAABB(float4 bbmin, float4 bbmax, Ray* r, float tmin, float* tmax) {
	float4 ori = r->origin;
	float4 dir = r->direction;
	dir.w = 1;
	float4 off1 = (bbmin - ori) / dir;
	float4 off2 = (bbmax - ori) / dir;

	float4 tempMin = fmin(off1, off2);
	float4 tempMax = fmax(off1, off2);

	float tnear = fmax(fmax(tempMin.x, tempMin.y), tempMin.z);
	float tfar = fmin(fmin(tempMax.x, tempMax.y), tempMax.z);
	if (tfar < tnear || tfar < tmin) return false;
	if (tnear >= *tmax) return false;
	return true;
}




__kernel void LCV(
	__global Ray* ray,
	__global BVHNode* nodes,

	__global Triangle* triangles,

	__global int* counts,
	__global int* innerCounts,
	__global int* primCounts
) {
	size_t gid = get_global_id(0);
	int stack[64] = {0};
	int sp = 1;
	float tmin = EPSILON;

	int count = 0;
	int innerCount = 0;
	int primCount = 0;


	Ray r = ray[gid];


	float tmax = FLT_MAX;


	while(sp > 0) {
		BVHNode node = nodes[stack[--sp]];
	NEXT:
		if (node.left == node.right) {
			++count;
		}
		else {
			++innerCount;
		}
		if(improveIntersectAABB(node.bbmin, node.bbmax, &r, tmin, &tmax)) {
			int leftNext = node.left;
			int rightNext = node.right;
			if(leftNext == rightNext) {
				++count;
				++primCount;

				Hit hit;
				hit.t = FLT_MAX;
				Triangle tr = triangles[leftNext];
				intersectTriangle(&r, &tr, 0, &hit);

				if (tmax >= hit.t) {
					tmax = hit.t;
				}
			}
			else {
				stack[sp++] = rightNext;
				node = nodes[leftNext];

				goto NEXT;
			}
		}
	}
	counts[gid] = count;

	innerCounts[gid] = innerCount;
	primCounts[gid] = primCount;
}


__kernel void LCV_Multi(
	__global Ray* ray,
	__global BVHNode* nodes,
	__global uint* indices,
	
	__global Triangle* triangles,

	__global int* counts,
	__global int* innerCounts,
	__global int* primCounts
) {
	size_t gid = get_global_id(0);
	int stack[64] = { 0 };
	int sp = 1;
	float tmin = EPSILON;

	int count = 0;
	int innerCount = 0;
	int primCount = 0;

	Ray r = ray[gid];

	float tmax = FLT_MAX;

	while (sp > 0) {
		BVHNode node = nodes[stack[--sp]];
	NEXT:
		if (node.isLeaf) {
			++count;
		}
		else {
			++innerCount;
		}
		if (improveIntersectAABB(node.bbmin, node.bbmax, &r, tmin, &tmax)) {
			if (node.isLeaf) {
				primCount += node.right - node.left;

				Hit hit;
				hit.t = FLT_MAX;
				for (int i = node.left; i < node.right; ++i) {
					Triangle tr = triangles[indices[i]];
					intersectTriangle(&r, &tr, 0, &hit);
				}
				if (tmax >= hit.t) {
					tmax = hit.t;
				}
			}
			else {
				int leftNext = node.left;
				int rightNext = node.right;
				stack[sp++] = rightNext;
				node = nodes[leftNext];

				goto NEXT;
			}
		}
	}
	counts[gid] = count;
	innerCounts[gid] = innerCount;
	primCounts[gid] = primCount;
}




















__kernel void QuadLCV(
	__global Ray* ray,
	__global QuadBVHNode* nodes,
	__global int* counts
) {
	size_t gid = get_global_id(0);
	int stack[64] = {0};
	int sp = 1;
	float tmin = EPSILON;

	Ray r = ray[gid];

	int count = 0;

	while(sp > 0) {
		QuadBVHNode node = nodes[stack[--sp]];
		if(intersectAABB(node.bbmin, node.bbmax,&r, tmin)) {
			int4 children = node.children;
			if(children.x == children.y) {
				++count;
			}
			else {
				stack[sp++] = children.x;
				if(children.y > 0)
					stack[sp++] = children.y;
				stack[sp++] = children.z;
				if(children.w > 0)
					stack[sp++] = children.w;
			}
		}
	}
	counts[gid] = count;
}