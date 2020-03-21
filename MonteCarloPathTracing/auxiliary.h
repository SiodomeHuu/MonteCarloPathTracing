#pragma once

#include "objdef.h"
#include "json.hpp"

using namespace nlohmann;

namespace MCPT::Auxiliary {
	constexpr float Cinn = 1.2f;
	constexpr float Cleaf = 0.0f;
	constexpr float Ctri = 1.0f;


	BoundingBox unionBox(const BoundingBox& a, const BoundingBox& b);
	float AREA(const BoundingBox& a);
	float AREA(float4 bbmin, float4 bbmax);



	Camera parseCamera(const json& jsonCamera);
}