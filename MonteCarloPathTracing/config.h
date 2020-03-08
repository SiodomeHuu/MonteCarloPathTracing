#pragma once



#include "json.hpp"
#include <string>


namespace MCPT::Config {
	int WIDTH();
	int HEIGHT();
	bool USEOPENCL();
	std::string PLATFORM();

	std::string RAYGENERATOR();
	nlohmann::json GETCAMERA();

	std::string GETDIRECTORY();
	std::string GETOBJNAME();

	std::string INTERSECTKERNELPATH();
	std::string SHADEKERNELPATH();
	int MAXDEPTH();
	int MAXATTEPMT();
}