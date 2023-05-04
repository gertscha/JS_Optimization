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


		// requries operator< to be defined for T
		template <typename T>
		class Heap {
		public:
			Heap();

			// add element
			void add(T element);

			// remove biggest element
			T pop();

			// get first element without removing it
			T peek();

			// get number of elements in the heap
			size_t size() { return m_heap.size(); }
			// get vector of all elements in the heap
			const std::vector<T>& getElements() const { return m_heap; }

		private:
			std::vector<T> m_heap;
		};


	};


}
