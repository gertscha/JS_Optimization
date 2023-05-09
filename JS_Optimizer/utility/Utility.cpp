#include "Utility.h"

#include "loguru.hpp"


namespace JSOptimizer {

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


	/*////////////////////
		Visualization
	////////////////////*/


	void run_python_script(const std::string& filepath) {
		// called with: python38 createGnatt.py "../JobShopSolutions/small_basic_sampleSol_testing.txt"

	}


	// inspired by https://towardsdatascience.com/gantt-charts-with-pythons-matplotlib-395b7af72d72
	void Utility::visualize(const std::string& sourceFolder, const std::string& sourceName,
		const std::string& outputFolder)
	{
		DLOG_F(WARNING, "called visualize, not implemented");

		// create thread for the visualization
		/*
		std::thread t(run_python_script, filepath);

		// Detach the thread so it can run independently
		t.detach();
		*/

	}


}
