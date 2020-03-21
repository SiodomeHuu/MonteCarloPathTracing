#include "OpenCLApp.h"
#include "oclbasic.h"
#include "openglapp.h"
#include "config.h"

#include "raygeneration.h"
#include "scenebuild.h"
#include "colorout.h"

#include "objdef.h"

#include <unordered_map>

using namespace MCPT::Config;
using namespace MCPT;

namespace {
	int width, height;
	cl::ImageGL clTex;

	
	void createAllKernels() {
		RayGeneration::init();
		SceneBuild::init();
		ColorOut::init();
	}

	cl::Buffer colorBuffer;
	std::vector<float4> colorBufferConstant;

	std::unique_ptr< SceneBuild::SceneBase > scene;

}

namespace MCPT::OpenCL {
	void init() {
		static auto initer = [&]() {
			width = WIDTH();
			height = HEIGHT();

			OpenCLBasic::init();

			createAllKernels();

			auto tex = OpenGL::getTex();
			clTex = cl::ImageGL(OpenCLBasic::getContext(), CL_MEM_WRITE_ONLY, GL_TEXTURE_RECTANGLE_ARB, 0, tex);

			auto [tri, mat, matId] = ThirdPartyWrapper::loadObject(GETDIRECTORY(), GETOBJNAME());
			scene = SceneBuild::buildScene(std::move(tri), std::move(mat), std::move(matId));

			colorBufferConstant.resize(width * height, { 1.0f,1.0f,1.0f,1.0f });
			colorBuffer = OpenCLBasic::newBuffer<cl_float4>(width * height,colorBufferConstant.data());
			return 0;
		}();
	}

	void update() {
		if (!Config::TESTBVH()) {
			std::vector<cl::Memory> tp = { clTex };
			OpenCLBasic::getQueue().enqueueAcquireGLObjects(&tp);
			OpenCLBasic::getQueue().finish();

			OpenCLBasic::writeBuffer(colorBuffer, colorBufferConstant.data());


			auto pRayBase = MCPT::RayGeneration::generateRay();


			for (int i = 0; i < Config::MAXDEPTH(); ++i) {
				scene->intersect(pRayBase);
				scene->shade(pRayBase, colorBuffer);
			}


			ColorOut::outputColorCL(clTex, colorBuffer);

			OpenCLBasic::getQueue().finish();

			OpenCLBasic::getQueue().enqueueReleaseGLObjects(&tp);
			OpenCLBasic::getQueue().finish();
		}
	}
}