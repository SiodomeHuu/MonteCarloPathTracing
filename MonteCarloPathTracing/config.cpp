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

	bool testBVH;
	bool testAll;
	std::string bvhType;


	template<class T>
	T tryRead(json obj, const std::string& str) {
		T temp = { 0 };

		auto iter = obj.find(str.c_str());
		if (iter != obj.end()) {
			return T(obj[str.c_str()]);
		}
		return temp;
	}
	template<>
	std::string tryRead<std::string>(json obj, const std::string& str) {
		std::string temp = "";

		auto iter = obj.find(str.c_str());
		if (iter != obj.end()) {
			return obj[str.c_str()].get<std::string>();
		}
		return temp;
	}
	template<>
	json tryRead<json>(json obj, const std::string& str) {
		json temp;

		auto iter = obj.find(str.c_str());
		if (iter != obj.end()) {
			return obj[str.c_str()];
		}
		return temp;
	}



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



			camera = tryRead<json>(config, "camera");

			directory = config["directory"].get<std::string>();
			objname = config["objname"].get<std::string>();
			width = double(config["width"]);
			height = double(config["height"]);

			bvhType = tryRead<std::string>(config, "bvhtype");
			if (bvhType == "") {
				bvhType = "hlbvh";
			}

			testAll = tryRead<bool>(config, "testall");
			if (testAll) {
				return;
			}
			testBVH = tryRead<bool>(config, "testbvh");
			if (testBVH) {
				return;
			}
			
			platform = config["platform"].get<std::string>();
			rayGenerator = config["raygenerator"].get<std::string>();

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
	RETURNFUNC(TESTBVH, testBVH);
	RETURNFUNC(TESTALL, testAll);
	RETURNFUNC(BVHTYPE, bvhType);
}

