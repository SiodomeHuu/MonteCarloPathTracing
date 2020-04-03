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

	int ancestor[128];
	ancestor[0] = myID;
	int anSize = 1;

	int nowParent = bvh[myID].parent;
	while(nowParent != -1) {
		ancestor[anSize++] = nowParent;
		nowParent = bvh[nowParent].parent;
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