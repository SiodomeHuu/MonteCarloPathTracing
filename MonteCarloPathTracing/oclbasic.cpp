#include "oclbasic.h"
#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
#include <windows.h>

#include "config.h"

using namespace MCPT;

cl::Context OpenCLBasic::context;
bool OpenCLBasic::has_init = false;
cl::CommandQueue OpenCLBasic::queue;

std::vector< cl::Platform > OpenCLBasic::platforms;
std::vector< cl::Device > OpenCLBasic::devices;

#pragma comment(lib,"OpenCL.lib")

namespace {
	constexpr int CL_INFOS[] = {
		CL_DEVICE_NAME,

		CL_DEVICE_VENDOR_ID,
		CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
		CL_DEVICE_MAX_WORK_GROUP_SIZE,
		CL_DEVICE_MAX_WORK_ITEM_SIZES,
		CL_DEVICE_LOCAL_MEM_TYPE,
		CL_DEVICE_LOCAL_MEM_SIZE,
		CL_DEVICE_GLOBAL_MEM_CACHE_SIZE,
		CL_DEVICE_GLOBAL_MEM_SIZE,
		CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
		CL_DEVICE_MAX_COMPUTE_UNITS
	};
	constexpr const char* const CL_INFOS_strings[] = {
		"CL_DEVICE_NAME",
		"CL_DEVICE_VENDOR_ID",
		"CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS",
		"CL_DEVICE_MAX_WORK_GROUP_SIZE",
		"CL_DEVICE_MAX_WORK_ITEM_SIZES",
		"CL_DEVICE_LOCAL_MEM_TYPE",
		"CL_DEVICE_LOCAL_MEM_SIZE",
		"CL_DEVICE_GLOBAL_MEM_CACHE_SIZE",
		"CL_DEVICE_GLOBAL_MEM_SIZE",
		"CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE",
		"CL_DEVICE_MAX_COMPUTE_UNITS"
	};


	void print(const char* str, size_t i) {
		std::cout << str << " = " << i << std::endl;
	}
	void print(const char* str, const std::string& s) {
		std::cout << str << " = " << s << std::endl;
	}
	void print(const char* str, const std::vector<size_t>& vec) {
		std::cout << str << " = { ";
		for (auto i : vec) {
			std::cout << i << " ";
		}
		std::cout << "}" << std::endl;
	}
	template<size_t i>
	void printSingleInfo(cl::Device& device) {
		print(CL_INFOS_strings[i], device.getInfo<CL_INFOS[i]>());
		if constexpr (i + 1 < std::size(CL_INFOS)) {
			printSingleInfo<i + 1>(device);
		}
	}

}


void OpenCLBasic::init() {
	if (has_init) return;
	cl::Platform::get(&platforms);

	/*for (auto& plat : platforms) {
		std::vector<cl::Device> td;
		plat.getDevices(CL_DEVICE_TYPE_ALL, &td);
		for (auto& dd : td) {
			printDeviceInformation(dd);
			std::cout << std::endl;
		}
		std::cout << std::endl << std::endl;
	}*/

	for (auto& plat : platforms) {
#ifdef PT_USE_NVIDIA
		if (plat.getInfo<CL_PLATFORM_NAME>().find("NVIDIA") == std::string::npos) continue;
#endif
#if (defined ANDROID) || (!defined USE_CPU)
			plat.getDevices(CL_DEVICE_TYPE_GPU, &devices);
#else
			plat.getDevices(CL_DEVICE_TYPE_CPU, &devices);
#endif


#if (defined PT_OPENGL_COOP) && (defined PT_USE_NVIDIA)
			if (Config::TESTALL()) {
				context = cl::Context(devices[0]);
			}
			else {
				cl_context_properties props[] =
				{
					CL_GL_CONTEXT_KHR, (cl_context_properties)wglGetCurrentContext(),
					CL_WGL_HDC_KHR, (cl_context_properties)wglGetCurrentDC(),
					CL_CONTEXT_PLATFORM, (cl_context_properties)plat(),
					0
				};
				context = cl::Context(devices[0], props);
			}
#else
			context = cl::Context(devices[0]);
#endif
			queue = cl::CommandQueue(context , CL_QUEUE_PROFILING_ENABLE);
			break;
	}
	has_init = true;
	return;
}

cl::Context& OpenCLBasic::getContext() {
	return context;
}

cl::CommandQueue& OpenCLBasic::getQueue() {
	return queue;
}



cl::Program OpenCLBasic::createProgram(const std::string& code, std::string option) {
	cl::Program::Sources source({ {code.c_str(),code.size()} });
	cl::Program program(context, source);

	try {
		if (option == "")
			program.build(devices);
		else
			program.build(devices, option.c_str());
	}
	catch (cl::Error & e) {
		std::string log = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]);
		std::cout << log;
		return cl::Program();
	}
	//std::string log = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]);
	//std::cout << log;
	return program;
}

cl::Program OpenCLBasic::createProgramFromFile(const std::string& code, std::string option) {
	std::stringstream ss;
	std::ifstream fin(code.c_str());
	if (fin) {
		ss << fin.rdbuf();
		return createProgram(ss.str(), option);
	}
	else {
		std::cout << "Cannot open file" << std::endl;
		return cl::Program();
	}
}

cl::Program MCPT::OpenCLBasic::createProgramFromFileWithHeader(const std::string& code, const std::string& header, std::string option) {
	std::stringstream ss;
	std::ifstream hfin(header.c_str());
	std::ifstream fin(code.c_str());
	if (fin && hfin) {
		ss << hfin.rdbuf();
		auto headerC = ss.str();
		ss.str("");
		ss << fin.rdbuf();
		headerC += "\n" + ss.str();
		return createProgram(headerC, option);
	}
	else {
		std::cout << "Cannot open file" << std::endl;
		return cl::Program();
	}
}

cl::Kernel OpenCLBasic::createKernel(cl::Program& program, const std::string& entry) {
	cl::Kernel ans;
	
	try {
		ans = cl::Kernel(program, entry.c_str());
	}
	catch (cl::Error & e) {
		std::cout << "Kernel Build Error" << std::endl;
		return cl::Kernel();
	}
	return ans;
}

cl::Buffer OpenCLBasic::createReadBuffer(size_t size, void* pos) {
	cl::Buffer ans(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, size, pos);
	return ans;
}


cl::Buffer OpenCLBasic::createWriteBuffer(size_t size) {
	cl::Buffer ans(context, CL_MEM_WRITE_ONLY, size, NULL);
	return ans;
}

cl::Buffer OpenCLBasic::createRWBuffer(size_t size, void* pos) {
	if (pos == nullptr) {
		cl::Buffer ans(context, CL_MEM_READ_WRITE, size, pos);
		return ans;
	}
	else {
		cl::Buffer ans(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, size, pos);
		return ans;
	}
}

void OpenCLBasic::enqueueReadBuffer(const cl::Buffer& buffer, size_t offset, size_t size, void* pt) {
	queue.enqueueReadBuffer(buffer, CL_TRUE, offset, size, pt);
}

void OpenCLBasic::enqueueWriteBuffer(const cl::Buffer& buffer, size_t offset, size_t size, void* pt) {
	queue.enqueueWriteBuffer(buffer, CL_TRUE, offset, size, pt);
}

void OpenCLBasic::enqueueNDRange(cl::Kernel& kernel, cl::NDRange global, cl::NDRange local, const std::vector<cl::Event>* evs, cl::Event* ev) {
	queue.enqueueNDRangeKernel(kernel, cl::NullRange, global, local, evs, ev);
}

float MCPT::OpenCLBasic::timeCost(const cl::Event& ev, int arg) {
	auto end = ev.getProfilingInfo<CL_PROFILING_COMMAND_END>();
	auto start = ev.getProfilingInfo<CL_PROFILING_COMMAND_START>();
	auto queued = ev.getProfilingInfo<CL_PROFILING_COMMAND_QUEUED>();
	auto submit = ev.getProfilingInfo<CL_PROFILING_COMMAND_SUBMIT>();
	
	switch (arg) {
	case 0: // end-start
		return end - start;
	case 1: // end-queued
		return end - queued;
	case 2:
		return end - submit;
	}
	return 0.0f;
}

void OpenCLBasic::readBuffer(const cl::Buffer& buffer, void* pt) {
	size_t size = buffer.getInfo<CL_MEM_SIZE>();
	enqueueReadBuffer(buffer, 0, size, pt);
}

void OpenCLBasic::writeBuffer(const cl::Buffer& buffer, void* pt) {
	size_t size = buffer.getInfo<CL_MEM_SIZE>();
	enqueueWriteBuffer(buffer, 0, size, pt);
}

void OpenCLBasic::enqueue1DKernelWithGroupCount(cl::Kernel& kernel, size_t workGroupCount, size_t singleGroupWorkItemCount, const std::vector<cl::Event>* evs, cl::Event* ev) {
	size_t globalSize = workGroupCount * singleGroupWorkItemCount;
	size_t localSize = singleGroupWorkItemCount;
	queue.enqueueNDRangeKernel(kernel, 0, globalSize, localSize, evs, ev);
}

void OpenCLBasic::printDeviceInformation(cl::Device& device) {
	printSingleInfo<0>(device);
}