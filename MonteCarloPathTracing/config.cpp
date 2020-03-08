#include "config.h"
#include <fstream>
#include <sstream>
#include <vector>

#define RETURNFUNC(A,B) decltype(B) A() {return B;}

using namespace nlohmann;

namespace {
	int width;
	int height;
	std::string platform;

	std::string rayGenerator;
	std::string intersectKernelPath;
	std::string shadeKernelPath;

	std::string directory;
	std::string objname;

	bool useOpencl;

	json configObj;
	json config;
	json camera;

	int maxDepth;
	int maxAttempt;

	struct CONFIG {
		CONFIG() {
			std::ifstream fin("config.json");
			std::stringstream ss;
			ss << fin.rdbuf();
			auto str = ss.str();
			
			configObj = json::parse(str);


			auto configs = configObj["config"];

			int configID = int(configObj["configid"]);

			config = configs[configID];

			width = double(config["width"]);
			height = double(config["height"]);

			platform = config["platform"].get<std::string>();

			rayGenerator = config["raygenerator"].get<std::string>();

			camera = config["camera"];

			directory = config["directory"].get<std::string>();
			objname = config["objname"].get<std::string>();

			useOpencl = bool(config["opencl"]);

			intersectKernelPath = config["intersect"].get<std::string>();
			shadeKernelPath = config["shade"].get<std::string>();

			maxDepth = config["maxdepth"];
			maxAttempt = config["attempt"];
		}
	} cfg;
}

namespace MCPT::Config {
	RETURNFUNC(WIDTH, width);
	RETURNFUNC(HEIGHT, height);
	RETURNFUNC(PLATFORM, platform);
	RETURNFUNC(RAYGENERATOR, rayGenerator);
	RETURNFUNC(GETCAMERA, camera);
	RETURNFUNC(GETDIRECTORY, directory);
	RETURNFUNC(GETOBJNAME, objname);
	RETURNFUNC(USEOPENCL, useOpencl);
	RETURNFUNC(INTERSECTKERNELPATH, intersectKernelPath);
	RETURNFUNC(SHADEKERNELPATH, shadeKernelPath);
	RETURNFUNC(MAXDEPTH, maxDepth);
	RETURNFUNC(MAXATTEPMT, maxAttempt);
}

