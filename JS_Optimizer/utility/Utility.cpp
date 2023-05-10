#include "Utility.h"

#include <iostream>

#include "loguru.hpp"


namespace JSOptimizer {

  extern std::string g_VSsol_path;
  std::string g_python_path = g_VSsol_path + "/pythonScripts/";

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
