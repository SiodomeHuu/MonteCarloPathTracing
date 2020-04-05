#include "openglapp.h"
#include "OpenCLApp.h"

#include "config.h"
#include "bvhtest.h"

#include "thirdpartywrapper.h"

using namespace MCPT;

int main(int argc,char** argv) {
	if (Config::TESTALL()) {
		BVH::TEST::testall();
		return 0;
	}
	else if (Config::TESTBVH()) {
		BVH::TEST::test();
		return 0;
	}

	OpenGL::init(argc, argv);
	OpenCL::init();
	OpenGL::enterLoop();
	
	return 0;
}