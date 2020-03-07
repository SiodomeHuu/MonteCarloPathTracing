__kernel void intersectRays(
	__global Ray* rays,
	__global BVHNode* bvhnodes,
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
	
	isIntersect = intersectObjects(bvhnodes, triangles, &myRay, &hit, tmin);
	if (isIntersect && dot(myRay.direction.s012, hit.normal.s012) > 0) {
		hit.normal = -hit.normal;
	}

	hits[id] = hit;
}