#define USE_HISTORY

__kernel void func(__global float4* newColors, __global float4* histories, __global int* sampleCounts) {
	size_t idx = get_global_id(0);
    size_t idy = get_global_id(1);
    size_t width = get_global_size(0);
    size_t height = get_global_size(1);
	size_t id = idy * width + idx;


#ifdef USE_HISTORY
	float4 nowColor = newColors[id];
	float4 hist = histories[id];

	if(length(nowColor) == 0 || sampleCounts[id] >= MAX_ATTEMPT) {
		newColors[id] = hist;
		return;
	}
	else {
		nowColor = (nowColor + hist * sampleCounts[id]) / (sampleCounts[id] + 1);
		histories[id] = newColors[id] = nowColor;
		histories[id].w = 0.0f;
		++sampleCounts[id];
	}
#else
	
#endif
}