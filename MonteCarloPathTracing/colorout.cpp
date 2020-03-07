#include "colorout.h"

#include "config.h"

#include "objdef.h"

#include<iostream>
#include "thirdpartywrapper.h"

using namespace MCPT;
using namespace MCPT::ColorOut;

namespace {

	cl::Program histProgram;
	cl::Kernel histKernel;

	cl::Program program;
	cl::Kernel outKernel;
	bool has_init = false;

	cl::Buffer frameBuffer;
	cl::Buffer sampleCount;
	std::vector<int> constantVec;

	int attemptCount = 0;

}

void ColorOut::init() {
	program = OpenCLBasic::createProgramFromFile("kernels/testkernel.cl");
	outKernel = OpenCLBasic::createKernel(program, "func");

	histProgram = OpenCLBasic::createProgramFromFileWithHeader("kernels/history.cl","objdef.h");
	histKernel = OpenCLBasic::createKernel(histProgram, "func");
}


void MCPT::ColorOut::outputColorCL(cl::ImageGL& tex,cl::Buffer& bf) {
	static auto initer = [&]() {
		int len = bf.getInfo <CL_MEM_SIZE>() / sizeof(float4);

		constantVec.resize(len, 0);

		std::vector<float4> t;
		t.resize(len, { 0,0,0,0 });

		frameBuffer = OpenCLBasic::newBuffer<float4>(len,t.data());
		sampleCount = OpenCLBasic::newBuffer<int>(len,constantVec.data());

		return 0;
	}();

	if (attemptCount <= MAX_ATTEMPT) {
		OpenCLBasic::setKernelArg(histKernel, bf, frameBuffer, sampleCount);
		OpenCLBasic::enqueueNDRange(histKernel, { (size_t)Config::WIDTH()  , (size_t)Config::HEIGHT() }, cl::NullRange);

		OpenCLBasic::setKernelArg(outKernel, tex, bf);
		OpenCLBasic::enqueueNDRange(outKernel, { (size_t)Config::WIDTH()  , (size_t)Config::HEIGHT() }, cl::NullRange);
		++attemptCount;
	}
	else {
		static auto initer = [&]() {
			std::cout << "Finished Attempting" << std::endl;
			ThirdPartyWrapper::outputPicture(Config::GETOBJNAME() + ".hdr", frameBuffer);
			return 0;
		}();

		OpenCLBasic::setKernelArg(outKernel, tex, frameBuffer);
		OpenCLBasic::enqueueNDRange(outKernel, { (size_t)Config::WIDTH()  , (size_t)Config::HEIGHT() }, cl::NullRange);
	}

}
