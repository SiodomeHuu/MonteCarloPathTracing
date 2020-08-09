#ifdef __cplusplus

#pragma once

#include "oclbasic.h"

namespace MCPT {

/*
#define __global
#define global
#define __local
#define local
#define __kernel
*/
    
#else
	#define EPSILON 1e-7f
#endif
	typedef float16 Matrix;


	typedef struct Camera_tag {
		float4 center, direction, up, horizontal;
		float arg;
		float tmin;
		uint cameraType; // 0 pers, 1 ortho
		float __padding;
	} Camera;

	typedef struct Ray_tag {
		union {
			float4 origin;
			int4 term_depth; // [terminate] [InObj?] [depth] [depth] - 4byte
		};
		union {
			float4 direction;
			int4 id;
		};
		float4 ratio;
	} Ray;

	typedef struct Hit_tag {
		float4 normal;
		float4 intersectPoint;
		float t;
		uint triangleID;
		uint materialID;
		uint __padding;
	} Hit;

	typedef struct Triangle_tag {
		float4 v[3];
		union {
			float4 normal;
			int4 materialID;
		};
	} Triangle;

	typedef enum MaterialType_tag
#ifdef __cplusplus
		: int32_t
#endif
	{
		MCPT_DIFFUSE = 1,
		MCPT_GLOSSY = 2,
		MCPT_TRANSPARENT = 3,
		MCPT_LIGHT = 4
	} MaterialType;

	typedef struct Material_tag {
		MaterialType type;
		float Ni;
		float Ns;
		float __padding;
		float4 kd;
		union {
			float4 ka;
			float4 ks;
		};
	} Material;
	
	typedef struct BoundingBox_tag {
		float4 bbmin;
		float4 bbmax;
#ifdef __cplusplus
		BoundingBox_tag()
			: bbmin({ FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX }), bbmax({ -FLT_MAX,-FLT_MAX,-FLT_MAX,-FLT_MAX }) {}
		BoundingBox_tag(float4 min, float4 max) : bbmin(min), bbmax(max) {}
		float4 centroid() const { return 0.5f * (bbmin + bbmax); }
		void unionBBox(const BoundingBox_tag& ano) { bbmin = min(bbmin, ano.bbmin); bbmax = max(bbmax, ano.bbmax); }
		void unionBBoxCentroid(const float4& ano) { bbmin = min(bbmin, ano); bbmax = max(bbmax, ano); }
		static BoundingBox_tag unionBBox(const BoundingBox_tag& a, const BoundingBox_tag& b) { BoundingBox_tag ans = a; ans.unionBBox(b); return ans; }
		static BoundingBox_tag unionBBoxCentroid(const BoundingBox_tag& a, const float4& b) { BoundingBox_tag ans = a; ans.unionBBoxCentroid(b); return ans; }
#endif
	} BoundingBox;

	typedef struct MortonPrimitive_tag {
		int id;
		int mortonCode;
	} MortonPrimitive;

	typedef struct BVHNode_tag {
		float4 bbmin;
		float4 bbmax;
		int parent;
		int left;
		int right;
		int isLeaf; // used in MultiPrimBVHNode
	} BVHNode;

	// when isLeaf > 0
	// left&right = range in vector<uint> indices;
	typedef BVHNode MultiPrimBVHNode;

	typedef struct QuadBVHNode_tag {
		union {
			float4 bbmin;
			int4 parent;
		};
		float4 bbmax;
		int4 children;
	} QuadBVHNode;


#ifndef __cplusplus
	float det2x2(float a, float b,
		float c, float d) {
		return a * d - b * c;
	}
	float det3x3(float a1, float a2, float a3,
		float b1, float b2, float b3,
		float c1, float c2, float c3) {
		return
			a1 * det2x2(b2, b3, c2, c3)
			- b1 * det2x2(a2, a3, c2, c3)
			+ c1 * det2x2(a2, a3, b2, b3);
	}

	float det4x4(float a1, float a2, float a3, float a4,
		float b1, float b2, float b3, float b4,
		float c1, float c2, float c3, float c4,
		float d1, float d2, float d3, float d4) {
		return
			a1 * det3x3(b2, b3, b4, c2, c3, c4, d2, d3, d4)
			- b1 * det3x3(a2, a3, a4, c2, c3, c4, d2, d3, d4)
			+ c1 * det3x3(a2, a3, a4, b2, b3, b4, d2, d3, d4)
			- d1 * det3x3(a2, a3, a4, b2, b3, b4, c2, c3, c4);
	}

	Matrix Inverse(Matrix m) {
		Matrix ans;

		float arr[16];
		vstore16(m, 0, arr);

		float a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4;
		a1 = arr[0];
		a2 = arr[1];
		a3 = arr[2];
		a4 = arr[3];
		b1 = arr[4];
		b2 = arr[5];
		b3 = arr[6];
		b4 = arr[7];
		c1 = arr[8];
		c2 = arr[9];
		c3 = arr[10];
		c4 = arr[11];
		d1 = arr[12];
		d2 = arr[13];
		d3 = arr[14];
		d4 = arr[15];

		float det = det4x4(a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4);

		if (fabs(det) < EPSILON) {
			ans.s0 = FLT_MAX;
			return ans;
		}

		ans.s0 = det3x3(b2, b3, b4, c2, c3, c4, d2, d3, d4) / det;
		ans.s1 = -det3x3(a2, a3, a4, c2, c3, c4, d2, d3, d4) / det;
		ans.s2 = det3x3(a2, a3, a4, b2, b3, b4, d2, d3, d4) / det;
		ans.s3 = -det3x3(a2, a3, a4, b2, b3, b4, c2, c3, c4) / det;

		ans.s4 = -det3x3(b1, b3, b4, c1, c3, c4, d1, d3, d4) / det;
		ans.s5 = det3x3(a1, a3, a4, c1, c3, c4, d1, d3, d4) / det;
		ans.s6 = -det3x3(a1, a3, a4, b1, b3, b4, d1, d3, d4) / det;
		ans.s7 = det3x3(a1, a3, a4, b1, b3, b4, c1, c3, c4) / det;

		ans.s8 = det3x3(b1, b2, b4, c1, c2, c4, d1, d2, d4) / det;
		ans.s9 = -det3x3(a1, a2, a4, c1, c2, c4, d1, d2, d4) / det;
		ans.sa = det3x3(a1, a2, a4, b1, b2, b4, d1, d2, d4) / det;
		ans.sb = -det3x3(a1, a2, a4, b1, b2, b4, c1, c2, c4) / det;

		ans.sc = -det3x3(b1, b2, b3, c1, c2, c3, d1, d2, d3) / det;
		ans.sd = det3x3(a1, a2, a3, c1, c2, c3, d1, d2, d3) / det;
		ans.se = -det3x3(a1, a2, a3, b1, b2, b3, d1, d2, d3) / det;
		ans.sf = det3x3(a1, a2, a3, b1, b2, b3, c1, c2, c3) / det;
		return ans;
	}
	bool intersectTriangle(Ray* ray, Triangle* object, float tmin, Hit* hit) {

		float3 realN = object->normal.s012;
		float3 Rd = ray->direction.s012;
		float3 AB, AC, A_Ro;
		float t, b, c;


		if (fabs(dot(realN, Rd)) < EPSILON) {
			return false;
		}

		AB = (object->v[1] - object->v[0]).s012;
		AC = (object->v[2] - object->v[0]).s012;
		A_Ro = (object->v[0] - ray->origin).s012;

		Matrix equalization = (float16)(
			Rd, 0,
			-AB, 0,
			-AC, 0,
			0, 0, 0, 1
			);

		equalization = Inverse(equalization);
		if (equalization.s0 == FLT_MAX)
		return false;

		t = dot(A_Ro, equalization.s048);
		b = dot(A_Ro, equalization.s159);
		c = dot(A_Ro, equalization.s26a);

		if (b < 0 || c < 0 || b + c > 1 || t <= tmin) {
			return false;
		}

		if (hit->t - t >= EPSILON) {
			hit->t = t;
			hit->normal = object->normal;
			hit->normal.w = 0.0f;
			hit->materialID = object->materialID.w;
			hit->intersectPoint = ray->origin + t * ray->direction;
		}
		return true;
	}

	bool intersectAABB(float4 bbmin, float4 bbmax, Ray* r, float tmin) {
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
		return true;
	}


	bool intersectObjects(
		__global BVHNode* bvhnodes,
		__global Triangle* triangles,
		Ray* ray,
		Hit* hit,
		float tmin
	) {
		int stack[128] = { 0 };
		int sp = 1;

		bool isIntersect = false;

		while (sp > 0) {
			BVHNode node = bvhnodes[stack[--sp]];
		NEXT:
			if (intersectAABB(node.bbmin, node.bbmax, ray, tmin)) {
				int leftNext = node.left;
				int rightNext = node.right;
				if (leftNext == rightNext) { // then triangle
					Triangle tr = triangles[leftNext];

					//MARK####
					if (intersectTriangle(ray, &tr, tmin, hit)) {
						isIntersect = true;
						hit->triangleID = leftNext;
					}
				}
				else { //then children
					stack[sp++] = rightNext;
					node = bvhnodes[leftNext];
					goto NEXT;
				}
			}
		}
		return isIntersect;
	}

	bool intersectObjectsMulti(
		__global BVHNode* bvhnodes,
		__global uint* indices,
		__global Triangle* triangles,
		Ray* ray,
		Hit* hit,
		float tmin
	) {
		int stack[128] = { 0 };
		int sp = 1;

		bool isIntersect = false;

		while (sp > 0) {
			BVHNode node = bvhnodes[stack[--sp]];
		NEXT:
			if (intersectAABB(node.bbmin, node.bbmax, ray, tmin)) {
				int leftNext = node.left;
				int rightNext = node.right;
				if (node.isLeaf) {
					for (int i = leftNext; i < rightNext; ++i) {
						int j = indices[i];
						Triangle tr = triangles[j];
						if (intersectTriangle(ray, &tr, tmin, hit)) {
							isIntersect = true;
							hit->triangleID = j;
						}
					}
				}
				else { //then children
					stack[sp++] = rightNext;
					node = bvhnodes[leftNext];
					goto NEXT;
				}
			}
		}
		return isIntersect;
	}
#endif



#ifdef __cplusplus
}
#endif