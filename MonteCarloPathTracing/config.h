#pragma once

#include <string>
#include "jsonrw.h"


namespace MCPT::Config {
	int WIDTH();
	int HEIGHT();
	std::string PLATFORM();

	std::string RAYGENERATOR();
	json::JsonObject GETCAMERA();

	std::string GETDIRECTORY();
	std::string GETOBJNAME();
}