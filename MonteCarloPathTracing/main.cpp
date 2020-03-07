#include "openglapp.h"
#include "OpenCLApp.h"

#include "thirdpartywrapper.h"

using namespace MCPT;

int main(int argc,char** argv) {
	OpenGL::init(argc, argv);
	OpenCL::init();
	OpenGL::enterLoop();
	
	return 0;
}