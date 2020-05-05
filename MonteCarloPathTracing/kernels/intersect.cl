bool intersectObjectsQuad(
	__global QuadBVHNode* bvhnodes,
	__global Triangle* triangles,
	Ray* ray,
	Hit* hit,
	float tmin
) {
	int stack[256] = { 0 };
	int sp = 1;

	bool isIntersect = false;
		
	while (sp > 0) {
		QuadBVHNode node = bvhnodes[stack[--sp]];
	NEXT:
		if (intersectAABB( (float4)(node.bbmin.s012,0.0f), node.bbmax, ray, tmin)) {
			int first = node.children.x;
			int second = node.children.y;
			int third = node.children.z;
			int fouth = node.children.w;

			if (first == second) { // then triangle
				Triangle tr = triangles[first];

				//MARK####
				if (intersectTriangle(ray, &tr, tmin, hit)) {
					isIntersect = true;
					hit->triangleID = first;
				}
			}
			else { //then children
				//if (first > 0)
					stack[sp++] = first;
				if(second > 0)
					stack[sp++] = second;
				//if (third > 0)
					stack[sp++] = third;
				if (fouth > 0)
					stack[sp++] = fouth;
				//int next = stack[--sp];
				//node = bvhnodes[next];
				//goto NEXT;
			}
		}
	}

	return isIntersect;
}



__kernel void intersectRays(
	__global Ray* rays,
#ifndef MCPT_USE_QUADBVH
	__global BVHNode* bvhnodes,
#else
	__global QuadBVHNode* bvhnodes,
#endif
	__global Triangle* triangles,
	__global Hit* hits,
	float tmin
) {

	size_t id = get_global_id(0);

	Hit hit = { 0 };
	hit.t = FLT_MAX;

	Ray myRay = rays[id];
	if(myRay.term_depth.w & 0xFF000000) {
		return;
	}

	bool isIntersect = false;
#ifndef MCPT_USE_QUADBVH
	isIntersect = intersectObjects(bvhnodes, triangles, &myRay, &hit, tmin);
#else
	isIntersect = intersectObjectsQuad(bvhnodes, triangles, &myRay, &hit, tmin);
#endif
	if (isIntersect && dot(myRay.direction.s012, hit.normal.s012) > 0) {
		hit.normal = -hit.normal;
	}
	
	hits[id] = hit;
}