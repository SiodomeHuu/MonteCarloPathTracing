#pragma once

#include <unordered_map>
#include <string>
#include <variant>

namespace MCPT::json {
	struct JsonValue;
	class nil {};
	typedef std::unordered_map< std::string, JsonValue> JsonObject;
	typedef std::vector<JsonValue> JsonArray;
	struct JsonValue {
		std::variant<JsonObject, JsonArray, double, std::string, bool, nil> v;
	};
	JsonObject parseJson(std::string line);
}

