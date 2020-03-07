__kernel void generateRay(
	__global Camera* obj,
	__global Ray* ray
) {
	size_t idx = get_global_id(0);
	size_t idy = get_global_id(1);
	size_t width = get_global_size(0);
	size_t height = get_global_size(1);
	size_t id = idy * width + idx;
	float2 point = (float2)(((float)idx) / width,(float)idy / height);
	float ratio = width * 1.0f / height;

	if (obj->cameraType == 0) { //Perspective
		float4 origin = obj->center;
		float temp1 = point.x - 0.5f;
		float temp2 = point.y - 0.5f;
		float distance = 0.5f / tan(obj->arg / 2);
		float4 d = obj->direction * distance + temp1 * obj->horizontal * ratio + temp2 * obj->up;

		ray[id].origin = origin;
		ray[id].direction = normalize(d);
	}
	else { //Orthographic
		float4 o;
		o = obj->center + (point.x - 0.5f)*(obj->arg)*(obj->horizontal) * ratio + (point.y - 0.5f)*(obj->arg)*(obj->up);
		ray[id].origin = o;
		ray[id].direction = normalize(obj->direction);
	}
	ray[id].term_depth.w = 0;
	ray[id].id.w = id;
}