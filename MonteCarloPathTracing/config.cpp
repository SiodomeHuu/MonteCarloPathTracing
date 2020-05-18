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
	json objs;

	bool useOpencl;

	json configObj;
	json config;
	json camera;

	int maxDepth;
	int maxAttempt;

	bool testBVH;
	bool testAll;
	std::string bvhType;
	bool useQuad;


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

			bvhType = tryRead<std::string>(config, "bvhtype");
			if (bvhType == "") {
				bvhType = "hlbvh";
			}
			useQuad = tryRead<bool>(config,"usequad");

			testAll = tryRead<bool>(config, "testall");
			if (testAll) {
				directory = config["directory"].get<std::string>();
				objs = config["objname"];
				return;
			}


			camera = tryRead<json>(config, "camera");

			directory = config["directory"].get<std::string>();
			objname = config["objname"].get<std::string>();

			width = tryRead<double>(config,"width");
			if (width == 0.0) width = 800.0;
			height = tryRead<double>(config, "height");
			if (height == 0.0) height = 600.0;
			

			useOpencl = tryRead<bool>(config,"opencl");

			testBVH = tryRead<bool>(config, "testbvh");
			
			platform = config["platform"].get<std::string>();
			rayGenerator = config["raygenerator"].get<std::string>();
			

			intersectKernelPath = config["intersect"].get<std::string>();
			shadeKernelPath = config["shade"].get<std::string>();

			maxDepth = tryRead<int>(config, "maxdepth");
			maxAttempt = tryRead<int>(config, "attempt");
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
	RETURNFUNC(GETOBJS, objs);
	RETURNFUNC(USEQUAD, useQuad);
}


namespace MCPT::Config {
	ConfigTrait::ConfigTrait() : empty(true) {}
	ConfigTrait::ConfigTrait(nlohmann::json input) : obj(input), empty(false) {}


	ConfigTrait ConfigTrait::operator[](const char* str) {
		if (empty) return ConfigTrait();
		auto iter = obj.find(str);
		if (iter != obj.end()) {
			return ConfigTrait(obj[str]);
		}
		return ConfigTrait();
	}
	ConfigTrait ConfigTrait::operator[](const std::string& str) {
		return operator[](str.c_str());
	}
	ConfigTrait ConfigTrait::operator[](size_t idx) {
		if (empty || idx >= obj.size()) return ConfigTrait();
		return obj.at(idx);
	}

	ConfigTrait getConfig() {
		return ConfigTrait(config);
	}	
}
