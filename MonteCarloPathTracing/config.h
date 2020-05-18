#pragma once



#include "json.hpp"
#include <string>


namespace MCPT::Config {
	[[deprecated]]
	int WIDTH();
	[[deprecated]]
	int HEIGHT();
	[[deprecated]]
	bool USEOPENCL();
	[[deprecated]]
	std::string PLATFORM();

	[[deprecated]]
	std::string RAYGENERATOR();
	[[deprecated]]
	nlohmann::json GETCAMERA();

	[[deprecated]]
	std::string GETDIRECTORY();
	[[deprecated]]
	std::string GETOBJNAME();
	[[deprecated]]
	nlohmann::json GETOBJS();

	[[deprecated]]
	std::string INTERSECTKERNELPATH();
	[[deprecated]]
	std::string SHADEKERNELPATH();
	[[deprecated]]
	int MAXDEPTH();
	[[deprecated]]
	int MAXATTEPMT();

	[[deprecated]]
	bool TESTBVH();
	[[deprecated]]
	bool TESTALL();
	[[deprecated]]
	std::string BVHTYPE();
	[[deprecated]]
	bool USEQUAD();


	
	struct ConfigTrait {
		ConfigTrait();
		ConfigTrait(nlohmann::json input);
		ConfigTrait operator[](const std::string& str);
		ConfigTrait operator[](const char* str);
		ConfigTrait operator[](size_t idx);

		template<class T>
		operator T() {
			if (empty) return T();
			return obj.get<T>();
		}

		const bool empty;
		const nlohmann::json obj;
	};
	ConfigTrait getConfig();
}