#include "raygeneration.h"

#include "config.h"
#include "objdef.h"
#include <cassert>

using namespace MCPT;
using namespace MCPT::RayGeneration;

namespace {

	bool has_init = false;

	cl::Program program;
	cl::Kernel rayGenerator;
	
	cl::Buffer cameraBuffer;
	//cl::Buffer rayBuffer;



	Camera parseCamera(json::JsonObject& jsonCamera) {
		Camera ans = { 0 };
		ans.cameraType = 0; // always perspective camera

		auto jsPos = std::get<json::JsonArray>(jsonCamera["position"].v);
		auto jsLookat = std::get<json::JsonArray>(jsonCamera["lookat"].v);
		auto jsUp = std::get<json::JsonArray>(jsonCamera["up"].v);

		for (int i = 0; i < 3; ++i) {
			ans.center.s[i] = static_cast<float>( std::get<double>(jsPos[i].v ) );
		}

		float4 lookat;
		for (int i = 0; i < 3; ++i) {
			lookat.s[i] = static_cast<float>(std::get<double>(jsLookat[i].v));
		}
		ans.direction = lookat - ans.center;
		ans.direction.w = 0.0f;

		for (int i = 0; i < 3; ++i) {
			ans.up.s[i] = static_cast<float>(std::get<double>(jsUp[i].v));
		}
		ans.arg = std::get<double>(jsonCamera["fov"].v) * M_PI / 180.0f;

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

	Camera camera;
	uint2 res;

	RayCL rayCL;
}

RayBase* MCPT::RayGeneration::generateRay() {
	assert(has_init);

	OpenCLBasic::setKernelArg(rayGenerator, cameraBuffer, rayCL.rayBuffer);
	OpenCLBasic::enqueueNDRange(rayGenerator, { res.x,res.y }, cl::NullRange);

	return &rayCL;
}

void MCPT::RayGeneration::init()
{
	static auto initer = [&]() {
		program = OpenCLBasic::createProgramFromFileWithHeader(Config::RAYGENERATOR(), "objdef.h");
		rayGenerator = OpenCLBasic::createKernel(program, "generateRay");

		auto jsonCamera = Config::GETCAMERA();
		camera = parseCamera(jsonCamera);

		auto resolution = std::get<json::JsonArray>(jsonCamera["resolution"].v);
		res = { static_cast<uint>(std::get<double>(resolution[0].v)), static_cast<uint>(std::get<double>(resolution[1].v)) };

		cameraBuffer = OpenCLBasic::newBuffer<Camera>(1, &camera);

		rayCL.rayBuffer = OpenCLBasic::newBuffer<Ray>(res.x * res.y);

		has_init = true;

		return 0;
	}();
}

