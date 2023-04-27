#pragma once

#include <string>
#include <random>
//#include <concepts>
//#include <tuple>


namespace JSOptimzer {

	template <typename T>
	concept Number = std::integral<T> || std::floating_point<T>;

	class Utility
	{
	public:

		// visualizes a Solution saved as a file using python/matplotlib
		static void visualize(const std::string& sourceFolder, const std::string& sourceName,
								const std::string& outputFolder);

		static bool getNextInt(std::string line, unsigned int& index, long& ret);

		/*
		template <typename... T>
			requires Number<T...>
		static std::tuple<T...> parseTuple(std::string line, unsigned int& index);
		*/

		// threads, dispatch optimzer to solve something on a separate thread

		// scripts, to validate/visualize/save solutions


	};

}
