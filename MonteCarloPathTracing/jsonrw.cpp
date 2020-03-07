#include<string>
#include<vector>
#include<unordered_map>
#include<variant>
#include<cmath>
#include"Windows.h"

#include"jsonrw.h"

using namespace MCPT::json;

namespace {

	JsonValue expectValue(char*&pt);

	inline char hex2char(char a, char b) {
		char ans = 0;
		if (!((a >= '0'&&a <= '9') || (a >= 'a'&&a <= 'f') || (a >= 'A' || a <= 'F'))) throw "error expecting string";
		if (!((b >= '0'&&b <= '9') || (b >= 'a'&&b <= 'f') || (b >= 'A' || b <= 'F'))) throw "error expecting string";
		if (a >= '0'&&a <= '9') a -= '0';
		else if (a >= 'a'&&a <= 'f') a -= 'a';
		else a -= 'A';
		if (b >= '0'&&b <= '9') b -= '0';
		else if (b >= 'a'&&b <= 'f') b -= 'a';
		else b -= 'A';
		ans = (a << 4) + b;
		return ans;
	}
	inline bool isBlank(char a) {
		if (a == ' ' || a == '\r' || a == '\n' || a == '\t') {
			return true;
		}
		return false;
	}

	void expectNull(char* &pt) {
		char *slide = pt;

		while (isBlank(*pt)) {
			++pt;
		}

		if (*slide == 'n' && *(slide + 1) == 'u' && *(slide + 2) == 'l' && *(slide + 3) == 'l') {
			pt += 4;
			return;
		}
		throw "error expecting null";
	}
	bool expectBool(char* &pt) {

		while (isBlank(*pt)) {
			++pt;
		}


		if (*pt == 't' && *(pt + 1) == 'r' && *(pt + 2) == 'u' && *(pt + 3) == 'e') {
			pt += 4;
			return true;
		}
		if (*pt == 'f'&&*(pt + 1) == 'a'&&*(pt + 2) == 'l'&&*(pt + 3) == 's'&&*(pt + 4) == 'e') {
			pt += 5;
			return false;
		}
		throw "error expecting boolean";
	}
	double expectNumber(char* &pt) {
		/*
		'-'? (0|[1-9][0-9]*) ('.' [0-9]+)? ( 'e'|'E' ('+'|'-')?[0-9]+)?
		*/

		while (isBlank(*pt)) {
			++pt;
		}

		double ans = 0;
		bool negative = false;
		char peek = *pt;
		if (peek == '-') {
			negative = true; ++pt; peek = *pt;
		}
		if (peek != '0') {
			if (!(peek >= '1'&&peek <= '9')) throw "error expecting num";
			while (peek >= '0'&&peek <= '9') {
				ans *= 10; ans += (peek - '0'); ++pt; peek = *pt;
			}
		}
		else {
			++pt; peek = *pt;
		}
		if (peek == '.') {
			++pt; peek = *pt;
			double acc = 0; double pri = 0.1;
			if (!(peek >= '0'&&peek <= '9')) throw "error expecting num";
			while (peek >= '0'&&peek <= '9') {
				acc += ((peek - '0') * pri);
				pri = pri * 0.1;
				++pt; peek = *pt;
			}
			ans += acc;
		}
		if (peek == 'e' || peek == 'E') {
			int jie = 0;
			bool jie_pos = true;
			++pt; peek = *pt;
			if (peek == '-') {
				jie_pos = false; ++pt; peek = *pt;
			}
			else if (peek == '+') {
				++pt; peek = *pt;
			}
			if (!(peek >= '0'&&peek <= '9')) throw "error expecting num";
			while (peek >= '0'&&peek <= '9') {
				jie *= 10;
				jie += (peek - '0');
				++pt; peek = *pt;
			}
			if (!jie_pos) jie = -jie;
			ans = ans * pow(10, jie);
		}
		if (negative) ans = -ans;
		return ans;
	}
	std::string expectString(char* &pt) {

		while (isBlank(*pt)) {
			++pt;
		}

		std::string ans;
		char peek = *pt;
		if (peek == '"') {
			++pt; peek = *pt;
			while (peek != '"') {
				if (peek == '\\') {
					char peek2 = *(pt + 1);
					switch (peek2) {
					case '"':
						ans += '"';
						break;
					case '\\':
						ans += '\\';
						break;
					case '/':
						ans += '/';
						break;
					case 'b':
						ans += '\b';
						break;
					case 'f':
						ans += char(12);
						break;
					case 'n':
						ans += char(10);
						break;
					case 'r':
						ans += char(13);
						break;
					case 't':
						ans += char(9);
						break;
					case 'u':
					{char peek3 = *(pt + 2), peek4 = *(pt + 3), peek5 = *(pt + 4), peek6 = *(pt + 5);
					char tempn = (hex2char(peek3, peek4) << 8) + hex2char(peek5, peek6);
					ans += tempn;
					pt += 4; }
					break;
					default:
						throw "error expecting string";
					}
					pt += 2; peek = *pt; continue;
				}
				else {
					ans += peek;
				}
				++pt; peek = *pt;
			}
			++pt; peek = *pt;

			return ans;
		}
		else {
			throw "error expecting string";
		}
	}
	JsonObject expectObject(char* &pt) {

		while (isBlank(*pt)) {
			++pt;
		}

		JsonObject ans;
		char peek = *pt;
		if (peek == '{') {
		loopget:
			auto&& a1 = expectString(++pt);

			while (isBlank(*pt)) {
				++pt;
			}
			if (*pt != ':') throw "error expecting Object";

			auto&& a2 = expectValue(++pt);
			ans.insert(std::make_pair(a1, a2));

			while (isBlank(*pt)) {
				++pt;
			}

			if (*pt == ',') {
				goto loopget;
			}
			else if (*pt == '}') {
				++pt;
				return ans;
			}
			else {
				throw "error expecting Object";
			}
		}
		else {
			throw "error expecting Object";
		}
	}
	JsonArray expectArray(char*&pt) {

		while (isBlank(*pt)) {
			++pt;
		}

		JsonArray ans;
		char peek = *pt;
		if (peek == '[') {
		loopget:
			auto &&v = expectValue(++pt);
			ans.push_back(v);

			while (isBlank(*pt)) {
				++pt;
			}

			if (*pt == ',') goto loopget;
			else if (*pt == ']') {
				++pt; return ans;
			}
			else {
				throw "error expecting Array";
			}
		}
		else {
			throw "error expecting Array";
		}
	}
	JsonValue expectValue(char*&pt) {

		while (isBlank(*pt)) {
			++pt;
		}

		JsonValue vl;
		char peek = *pt;
		if (peek == 'n') {
			expectNull(pt); vl.v = {}; return vl;
		}
		else if (peek == 't' || peek == 'f') {
			bool t = expectBool(pt); vl.v = t; return vl;
		}
		else if (peek == '{') {
			auto t = expectObject(pt); vl.v = t; return vl;
		}
		else if (peek == '[') {
			auto t = expectArray(pt); vl.v = t; return vl;
		}
		else if ((peek >= '0'&&peek <= '9') || peek == '-') {
			auto t = expectNumber(pt); vl.v = t; return vl;
		}
		else if (peek == '"') {
			auto t = expectString(pt); vl.v = t; return vl;
		}
		else {
			throw "error expecting value";
		}
	}
}




JsonObject MCPT::json::parseJson(std::string line) {
	JsonObject t;
	bool now_in_str = false;
	if (line.length() == 0) return t;

	char *pt = &line[0];
	try {
		return expectObject(pt);
	}
	catch (const char* err) {
		MessageBoxA(NULL, err, "Caption", MB_OK);
		return t;
	}
}