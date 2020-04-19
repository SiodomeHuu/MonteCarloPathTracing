#include "scenebuild.h"
#include <deque>
#include "config.h"
#include <ctime>

#include <iostream>
#include <cassert>

#include "BVH/hlbvh.h"
#include "BVH/treeletBVH.h"

using namespace MCPT;
using namespace MCPT::SceneBuild;



namespace {
	std::vector<MCPT::Triangle> triangles;
	std::vector<MCPT::Material> materials;
	std::vector<int> matIndices;

	std::unique_ptr< BVH::HLBVH<BVH::CPU> > pt;
	std::unique_ptr< BVH::TreeletBVH<BVH::CPU> > pt2;

	std::unique_ptr< BVH::CPUBVH > bvhpt;
	std::unique_ptr< BVH::GPUBVH > gpubvhpt;

	cl::Buffer trBuffer;
	cl::Buffer matBuffer;
	cl::Buffer matIDBuffer;
	cl::Buffer bvhBuffer;
	cl::Buffer hitBuffer;
	cl::Buffer randBuffer;

	cl::Program intersectProgram;
	cl::Kernel intersectKernel;

	cl::Program shadeProgram;
	cl::Kernel shadeKernel;

	size_t rayCount;


}





SceneCL::SceneCL(std::vector<MCPT::Triangle> tr, std::vector<MCPT::Material> mat, std::vector<int> matID)
	: SceneBase()
{
	triangles = std::move(tr);
	materials = std::move(mat);
	matIndices = std::move(matID);


	for (auto i = 0; i < matIndices.size(); ++i) {
		auto& tr = triangles[i];
		tr.normal = normalize(cross(tr.v[1] - tr.v[0], tr.v[2] - tr.v[0]));
		tr.materialID.w = matIndices[i];
	}

	

	auto bvhtype = Config::BVHTYPE();
	if (bvhtype == "hlbvh") {
		bvhpt = std::make_unique<BVH::HLBVH<BVH::CPU>>(BVH::HLBVH<BVH::CPU>(triangles));
	}
	else if (bvhtype == "treelet") {
		auto temp = std::make_unique<BVH::HLBVH<BVH::CPU>>(BVH::HLBVH<BVH::CPU>(triangles));
		bvhpt = std::make_unique< BVH::TreeletBVH<BVH::CPU> >(temp->releaseBVH());
	}
	else if (bvhtype == "treeletGPU") {
		goto GPUBVH;
	}
	else {
		throw "BVH Not Implemented";
	}
	{
		auto bvh = bvhpt->getBVH();

		trBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), triangles.data());
		bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(bvh.size(), bvh.data());
		
	}
GPUBVH:
	{
		auto temp = std::make_unique<BVH::HLBVH<BVH::CPU>>(BVH::HLBVH<BVH::CPU>(triangles));
		auto bvh = temp->releaseBVH();
		bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(bvh.size(), bvh.data());
		trBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), triangles.data());

		gpubvhpt = std::make_unique<BVH::TreeletBVH<BVH::GPU>>(bvhBuffer, trBuffer);
	}
	
	matBuffer = OpenCLBasic::newBuffer<Material>(materials.size(), materials.data());
	matIDBuffer = OpenCLBasic::newBuffer<int>(matIndices.size(), matIndices.data());

	rayCount = 0;
}

void SceneCL::intersect(MCPT::RayGeneration::RayBase* rays)
{
	if (typeid(*rays) == typeid(MCPT::RayGeneration::RayCL)) {
		auto rayPt = dynamic_cast<MCPT::RayGeneration::RayCL*>(rays);

		static auto initer = [&]() {
			assert(rayCount == 0);
			rayCount = rayPt->rayBuffer.getInfo<CL_MEM_SIZE>() / sizeof(Ray);
			hitBuffer = OpenCLBasic::newBuffer<Hit>(rayCount);

			std::vector<uint> randNum;
			randNum.resize(rayCount);

			srand(time(NULL));
			for (int i = 0; i < rayCount; ++i) {
				randNum[i] = rand();
			}
			randBuffer = OpenCLBasic::newBuffer<uint>(rayCount,randNum.data());

			return 0;
		} ();

		OpenCLBasic::setKernelArg(intersectKernel, rayPt->rayBuffer,bvhBuffer,trBuffer,hitBuffer,EPSILON);
		OpenCLBasic::enqueueNDRange(intersectKernel, rayCount, cl::NullRange);


	}
	else {
		throw "Not Implemented";
	}
}


void SceneCL::shade(MCPT::RayGeneration::RayBase* rays,cl::Buffer& colorBuffer) {
	if (typeid(*rays) == typeid(MCPT::RayGeneration::RayCL)) {
		auto pt = dynamic_cast<MCPT::RayGeneration::RayCL*>(rays);
		OpenCLBasic::setKernelArg(shadeKernel, matBuffer, pt->rayBuffer, hitBuffer, colorBuffer, randBuffer);
		OpenCLBasic::enqueueNDRange(shadeKernel, rayCount, cl::NullRange);
	}
	else {
		throw "Not Implemented";
	}
}



std::unique_ptr< SceneBase > MCPT::SceneBuild::buildScene(std::vector<MCPT::Triangle> tr, std::vector<MCPT::Material> mat, std::vector<int> matID) {
	if (Config::USEOPENCL()) {
		return std::make_unique<SceneCL>(std::move(tr), std::move(mat), std::move(matID));
	}
	else {
		throw "Not Implemented";
	}
}

void MCPT::SceneBuild::init() {
	if (Config::USEOPENCL()) {
		intersectProgram = OpenCLBasic::createProgramFromFileWithHeader(Config::INTERSECTKERNELPATH(), "objdef.h");
		intersectKernel = OpenCLBasic::createKernel(intersectProgram, "intersectRays");

		shadeProgram = OpenCLBasic::createProgramFromFileWithHeader(Config::SHADEKERNELPATH(), "objdef.h",("-D MAX_DEPTH="+std::to_string(Config::MAXDEPTH())));
		shadeKernel = OpenCLBasic::createKernel(shadeProgram, "shade");
	}
	else {
		throw "Not Implemented";
	}
}
