#include "auxiliary.h"

using namespace MCPT;

BoundingBox Auxiliary::unionBox(const BoundingBox& a, const BoundingBox& b) {
	BoundingBox ans;
	ans.bbmax = max(a.bbmax, b.bbmax);
	ans.bbmin = min(a.bbmin, b.bbmin);
	return ans;
}
float Auxiliary::AREA(const BoundingBox& a) {
	auto arg = a.bbmax - a.bbmin;
	return 2.0f * (arg.x * arg.y + arg.x * arg.z + arg.y * arg.z);
}
float Auxiliary::AREA(float4 bbmin, float4 bbmax) {
	auto arg = bbmax - bbmin;
	return 2.0f * (arg.x * arg.y + arg.x * arg.z + arg.y * arg.z);
}

Camera Auxiliary::parseCamera(const json& jsonCamera) {
	Camera ans = { 0 };
	ans.cameraType = 0; // always perspective camera

	auto jsPos = jsonCamera["position"].get<std::vector<json> >();
	auto jsLookat = jsonCamera["lookat"].get<std::vector<json> >();
	auto jsUp = jsonCamera["up"].get<std::vector<json> >();

	for (int i = 0; i < 3; ++i) {
		ans.center.s[i] = float(jsPos[i]);
	}

	float4 lookat;
	for (int i = 0; i < 3; ++i) {
		lookat.s[i] = float(jsLookat[i]);
	}
	ans.direction = lookat - ans.center;
	ans.direction.w = 0.0f;

	for (int i = 0; i < 3; ++i) {
		ans.up.s[i] = float(jsUp[i]);
	}
	ans.arg = float(jsonCamera["fov"]) * M_PI / 180.0f;

	if (ans.cameraType == 0) {
		float4 horizontal = cross(ans.direction, ans.up);
		ans.up = cross(horizontal, ans.direction);
		ans.tmin = 0;
		ans.direction = normalize(ans.direction);
		ans.up = normalize(ans.up);
		ans.horizontal = normalize(horizontal);
	}
	else {
		float4 realup;
		float temp = dot(ans.up, ans.direction) / dot(ans.direction, ans.direction);
		realup = {
				ans.up.x - temp * ans.direction.x,
				ans.up.y - temp * ans.direction.y,
				ans.up.z - temp * ans.direction.z,
				ans.up.w - temp * ans.direction.w
		};
		ans.up = realup;
		float4 horizontal = cross(ans.direction, ans.up);

		ans.tmin = -FLT_MAX;
		ans.direction = normalize(ans.direction);
		ans.up = normalize(ans.up);
		ans.horizontal = normalize(horizontal);
	}

	return ans;
}