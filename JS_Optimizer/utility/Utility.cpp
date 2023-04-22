#include "Utility.h"


namespace SimAnn {

	bool Utility::getNextInt(std::string line, unsigned int& index, long& ret) {
		if (index >= line.size())
			return false;
		std::string string = line.substr(index, line.size());
		if (string.size() < 1)
			return false;
		const char* str = string.c_str();
		char* offsetMes;
		long res = strtol(str, &offsetMes, 10);
		int offset = (int)(offsetMes - str);
		index += offset;
		ret = res;
		if (offset == 0)
			return false;
		else
			return true;
	}


}
