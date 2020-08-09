#include "scenebuild.h"
#include <deque>
#include "config.h"
#include <ctime>

#include <iostream>
#include <cassert>

#include "BVH/hlbvh.h"
#include "BVH/treeletBVH.h"
#include "BVH/simpleQuad.h"
#include "BVH/sahbvh.h"

#include "bvhtest.h"

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
	std::unique_ptr< BVH::SimpleBVHQuad<BVH::CPU> > quadbvhpt;

	cl::Buffer trBuffer;
	cl::Buffer matBuffer;
	cl::Buffer matIDBuffer;
	cl::Buffer bvhBuffer;
	cl::Buffer indicesBuffer; // use in MultiPrim
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
	if (!materials.empty()) {
		matBuffer = OpenCLBasic::newBuffer<Material>(materials.size(), materials.data());
		matIDBuffer = OpenCLBasic::newBuffer<int>(matIndices.size(), matIndices.data());
	}
	else {
		matBuffer = OpenCLBasic::newBuffer<Material>(1);
		matIDBuffer = OpenCLBasic::newBuffer<int>(1);
	}
	
	rayCount = 0;
	auto bvhtype = Config::BVHTYPE();
	if (bvhtype == "hlbvh") {
		bvhpt = std::make_unique<BVH::HLBVH<BVH::CPU>>(BVH::HLBVH<BVH::CPU>(triangles));
	}
	else if (bvhtype == "treelet") {
		auto temp = std::make_unique<BVH::HLBVH<BVH::CPU>>(BVH::HLBVH<BVH::CPU>(triangles));
		bvhpt = std::make_unique< BVH::TreeletBVH<BVH::CPU> >(temp->releaseBVH());
	}
	else if (bvhtype == "treelet2") {
		auto temp = std::make_unique<BVH::HLBVH<BVH::CPU>>(BVH::HLBVH<BVH::CPU>(triangles));
		bvhpt = std::make_unique< BVH::TreeletBVH<BVH::CPU> >(temp->releaseBVH());
		auto nodes = bvhpt->releaseBVH();
		bvhpt.release();
		bvhpt = std::make_unique< BVH::TreeletBVH_<BVH::CPU> >(std::move(nodes));

		indicesBuffer = OpenCLBasic::newBuffer<uint>(bvhpt->getIndices().size(), (void*)bvhpt->getIndices().data());
	}
	else if (bvhtype == "treeletGPU") {
		goto GPUBVH;
	}
	else if (bvhtype.substr(0,3) == "sah") {
		//throw "Not Implemented";
		std::vector<BoundingBox> bboxes(triangles.size());
		for (int i = 0; i < triangles.size(); ++i) {
			bboxes[i].unionBBoxCentroid(triangles[i].v[0]);
			bboxes[i].unionBBoxCentroid(triangles[i].v[1]);
			bboxes[i].unionBBoxCentroid(triangles[i].v[2]);
		}
		std::vector<uint32_t> indices(triangles.size());
		for (int i = 0; i < triangles.size(); ++i) {
			indices[i] = i;
		}

		auto arg = std::tuple{ false, 0u, 16 };
		switch (bvhtype[3]) {
		case '1': { // 1axis binning
			arg = { false, 16u ,0 };
			break;
		}
		case '2': { // 1axis full
			arg = { false, 0u, 0 };
			break;
		}
		case '3': { // 3axis binning
			arg = { true, 16u, 0 };
			break;
		}
		case '4': { // 3axis full
			arg = { true, 0u, 0 };
			break;
		}
		default:
			throw "Not Implemented";
		}

		bvhpt = std::make_unique< BVH::SAHBVH<BVH::CPU> >(bboxes, std::move(indices), arg);

		indicesBuffer = OpenCLBasic::newBuffer<uint>(bvhpt->getIndices().size(), (void*)bvhpt->getIndices().data());
	}
	else {
		throw "BVH Not Implemented";
	}
	{
		if (Config::USEQUAD()) {
			quadbvhpt = std::make_unique< BVH::SimpleBVHQuad<BVH::CPU> >(bvhpt->releaseBVH());
			auto bvh = quadbvhpt->getQuadBVH();
			trBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), triangles.data());
			bvhBuffer = OpenCLBasic::newBuffer<QuadBVHNode>(bvh.size(), bvh.data());

			if ((bool)Config::getConfig()["testbvh"]) {
				BVH::TEST::singleTest(bvhBuffer, trBuffer);
			}
		}
		else {
			auto bvh = bvhpt->getBVH();
			trBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), triangles.data());
			bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(bvh.size(), bvh.data());

			if (Config::BVHTYPE() == "treelet2" || Config::BVHTYPE().substr(0, 3) == "sah") {
				BVH::TEST::singleTest(bvh, bvhpt->getIndices(), trBuffer);
			}
			else if ((bool)Config::getConfig()["testbvh"]) {
				BVH::TEST::singleTest(bvhBuffer, trBuffer);
			}
			
		}
		
		return;
	}
GPUBVH:
	{
		auto temp = std::make_unique<BVH::HLBVH<BVH::CPU>>(BVH::HLBVH<BVH::CPU>(triangles));
		auto bvh = temp->releaseBVH();
		bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(bvh.size(), bvh.data());
		trBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), triangles.data());
		
		gpubvhpt = std::make_unique<BVH::TreeletBVH<BVH::GPU>>(bvhBuffer, trBuffer);

		if ((bool)Config::getConfig()["testbvh"]) {
			BVH::TEST::singleTest(bvhBuffer, trBuffer);
		}
	}
	return;
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

		if (Config::BVHTYPE() == "treelet2" || Config::BVHTYPE().substr(0, 3) == "sah") {
			OpenCLBasic::setKernelArg(intersectKernel, rayPt->rayBuffer, bvhBuffer, indicesBuffer, trBuffer, hitBuffer, EPSILON);
			OpenCLBasic::enqueueNDRange(intersectKernel, rayCount, cl::NullRange);
		}
		else {
			OpenCLBasic::setKernelArg(intersectKernel, rayPt->rayBuffer, bvhBuffer, trBuffer, hitBuffer, EPSILON);
			OpenCLBasic::enqueueNDRange(intersectKernel, rayCount, cl::NullRange);
		}
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
		if (Config::BVHTYPE() == "treelet2" || Config::BVHTYPE().substr(0, 3) == "sah") {
			intersectProgram = OpenCLBasic::createProgramFromFileWithHeader(Config::INTERSECTKERNELPATH(), "objdef.h", "-D MCPT_MULTI=1");
			intersectKernel = OpenCLBasic::createKernel(intersectProgram, "intersectRays");
		}
		else {
			if (Config::USEQUAD()) {
				intersectProgram = OpenCLBasic::createProgramFromFileWithHeader(Config::INTERSECTKERNELPATH(), "objdef.h", "-D MCPT_USE_QUADBVH=1");
				intersectKernel = OpenCLBasic::createKernel(intersectProgram, "intersectRays");
			}
			else {
				intersectProgram = OpenCLBasic::createProgramFromFileWithHeader(Config::INTERSECTKERNELPATH(), "objdef.h");
				intersectKernel = OpenCLBasic::createKernel(intersectProgram, "intersectRays");
			}
		}

		

		if (Config::TESTBVH()) {
			shadeProgram = OpenCLBasic::createProgramFromFileWithHeader(Config::SHADEKERNELPATH(), "objdef.h", ("-D MCPT_TEST_BVH=1 -D MAX_DEPTH=" + std::to_string(Config::MAXDEPTH())));
			shadeKernel = OpenCLBasic::createKernel(shadeProgram, "shade");
		}
		else {
			shadeProgram = OpenCLBasic::createProgramFromFileWithHeader(Config::SHADEKERNELPATH(), "objdef.h", ("-D MAX_DEPTH=" + std::to_string(Config::MAXDEPTH())));
			shadeKernel = OpenCLBasic::createKernel(shadeProgram, "shade");
		}
	}
	else {
		throw "Not Implemented";
	}
}
