__kernel void func(__write_only image2d_t pixels, __global float4* color) {
    size_t idx = get_global_id(0);
    size_t idy = get_global_id(1);
    size_t width = get_global_size(0);
    size_t height = get_global_size(1);
	size_t id = idy * width + idx;

	
    float4 col = color[id];
	col.w = 0.0f;
	write_imagef(pixels,(int2)(idx,idy),col);
}