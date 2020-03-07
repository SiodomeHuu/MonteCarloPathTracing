#include "config.h"
#include "jsonrw.h"
#include <fstream>
#include <sstream>


#define RETURNFUNC(A,B) decltype(B) A() {return B;}

namespace {
	int width;
	int height;
	std::string platform;

	std::string rayGenerator;
	MCPT::json::JsonObject camera;

	std::string directory;
	std::string objname;

	struct CONFIG {
		CONFIG() {
			std::ifstream fin("config.json");
			std::stringstream ss;
			ss << fin.rdbuf();
			
			jobj = MCPT::json::parseJson(ss.str());

			width = std::get<double>(jobj["width"].v);
			height = std::get<double>(jobj["height"].v);
			platform = std::get<std::string>(jobj["platform"].v);
			rayGenerator = std::get<std::string>(jobj["raygenerator"].v);
			camera = std::get<MCPT::json::JsonObject>(jobj["camera"].v);

			directory = std::get<std::string>(jobj["directory"].v);
			objname = std::get<std::string>(jobj["objname"].v);
		}
		MCPT::json::JsonObject jobj;
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
}

