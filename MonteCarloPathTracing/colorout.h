#pragma once
#include "oclbasic.h"

namespace MCPT::ColorOut {

	void init();

	void outputColorCL(cl::ImageGL &tex,cl::Buffer& bf);
	
	void refresh();
}