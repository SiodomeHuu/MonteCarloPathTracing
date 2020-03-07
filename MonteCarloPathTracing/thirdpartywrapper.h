#pragma once

#include <vector>
#include "oclbasic.h"
#include <string>
#include <tuple>

#include "objdef.h"

namespace MCPT::ThirdPartyWrapper {


	std::tuple<std::vector<MCPT::Triangle>, std::vector<MCPT::Material>,std::vector<int> > loadObject(const std::string& directory, const std::string& objname);


	void outputPicture(const std::string& str,cl::Buffer pic);
}