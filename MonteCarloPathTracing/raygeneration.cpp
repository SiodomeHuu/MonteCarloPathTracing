#include "raygeneration.h"

#include "config.h"
#include "objdef.h"
#include <cassert>
#include <vector>
#include "auxiliary.h"

using namespace MCPT;
using namespace MCPT::RayGeneration;

using namespace nlohmann;

namespace {

	bool has_init = false;

	cl::Program program;
	cl::Kernel rayGenerator;
	
	cl::Buffer cameraBuffer;

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
			camera = Auxiliary::parseCamera(jsonCamera);

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

