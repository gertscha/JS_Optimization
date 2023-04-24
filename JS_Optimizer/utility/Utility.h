#pragma once

#include <string>


namespace JSOptimzer {

	class Utility
	{
	public:

		// visualizes a Solution saved as a file using python/matplotlib
		static void visualize(const std::string& sourceFolder, const std::string& sourceName,
								const std::string& outputFolder);

		static bool getNextInt(std::string line, unsigned int& index, long& ret);

		// threads, dispatch optimzer to solve something on a separate thread

		// scripts, to validate/visualize/save solutions


	};

}
