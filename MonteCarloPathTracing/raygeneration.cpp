#include "raygeneration.h"

#include "config.h"
#include "objdef.h"
#include <cassert>
#include <vector>

using namespace MCPT;
using namespace MCPT::RayGeneration;

using namespace nlohmann;

namespace {

	bool has_init = false;

	cl::Program program;
	cl::Kernel rayGenerator;
	
	cl::Buffer cameraBuffer;
	
	Camera parseCamera(const json& jsonCamera) {
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

	Camera camera;
	uint2 res;

	RayCL rayCL;
}

RayBase* MCPT::RayGeneration::generateRay() {
	if (Config::USEOPENCL()) {
		assert(has_init);
		OpenCLBasic::setKernelArg(rayGenerator, cameraBuffer, rayCL.rayBuffer);
		OpenCLBasic::enqueueNDRange(rayGenerator, { res.x,res.y }, cl::NullRange);
		return &rayCL;
	}
	else {
		throw "Not Implemented";
	}
}

void MCPT::RayGeneration::init()
{
	static auto initer = [&]() {
		if (Config::USEOPENCL()) {
			program = OpenCLBasic::createProgramFromFileWithHeader(Config::RAYGENERATOR(), "objdef.h");
			rayGenerator = OpenCLBasic::createKernel(program, "generateRay");

			auto jsonCamera = Config::GETCAMERA();
			camera = parseCamera(jsonCamera);

			auto resolution = jsonCamera["resolution"].get<std::vector<json> >();
			res = { uint(resolution[0]), uint(resolution[1]) };

			cameraBuffer = OpenCLBasic::newBuffer<Camera>(1, &camera);

			rayCL.rayBuffer = OpenCLBasic::newBuffer<Ray>(res.x * res.y);

			has_init = true;

			return 0;
		}
		else {
			throw "Not Implemented";
		}
		
	}();
}

