#pragma once

#include <string>
#include <random>
#include <algorithm>
//#include <concepts>
//#include <tuple>


namespace JSOptimzer {

	template <typename T>
	concept Number = std::integral<T> || std::floating_point<T>;

	class Utility
	{
	public:

		// visualizes a Solution saved as a file using python/matplotlib
		static void visualize(const std::string& sourceFolder, const std::string& sourceName, const std::string& outputFolder);

		// parse string lines for longs, uses strtol
		static bool getNextInt(std::string line, unsigned int& index, long& ret);

		// remove an element efficently
		template <typename T>
		T remove_at(std::vector<T>& v, typename std::vector<T>::size_type n)
		{
			T ans = std::move_if_noexcept(v[n]);
			v[n] = std::move_if_noexcept(v.back());
			v.pop_back();
			return ans;
		}

		/*
		template <typename... T>
			requires Number<T...>
		static std::tuple<T...> parseTuple(std::string line, unsigned int& index);
		*/

		// threads, dispatch optimzer to solve something on a separate thread

		// scripts, to validate/visualize/save solutions


		/*////////////////////
			Value Wrapper
		////////////////////*/

		template<typename T>
		struct Wrapper {
			T* ptr;
			Wrapper(T* pointer) :ptr(pointer) {}
			bool operator<(const Wrapper<T>& rhs) { return false; } // specialize if needed
		};

		/*///////////
			Heap
		///////////*/

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

			// adds contender and removes and returns the biggest element
			T replace(T contender);

			// get number of elements in the heap
			size_t size() { return m_heap.size(); }
			// get vector of all elements in the heap
			const std::vector<T>& getElements() const { return m_heap; }

		private:
			std::vector<T> m_heap;
		};


	};


	/*///////////
		Heap
	///////////*/


	template<typename T>
	inline Utility::Heap<T>::Heap()
	{
		m_heap = std::vector<T>();
	}

	template<typename T>
	inline void Utility::Heap<T>::add(T element)
	{
		if (!std::is_heap(m_heap.begin(), m_heap.end()))
			std::make_heap(m_heap.begin(), m_heap.end());

		m_heap.push_back(element);
		std::push_heap(m_heap.begin(), m_heap.end());
	}

	template<typename T>
	inline T Utility::Heap<T>::pop()
	{
		std::pop_heap(m_heap.begin(), m_heap.end()); // moves the largest to the end
		T largest = m_heap.back();
		m_heap.pop_back(); // actually removes the largest element
		return largest;
	}

	template<typename T>
	inline T Utility::Heap<T>::peek()
	{
		std::pop_heap(m_heap.begin(), m_heap.end());
		T largest = m_heap.back();
		std::make_heap(m_heap.begin(), m_heap.end());
		return largest;
	}

	template<typename T>
	inline T Utility::Heap<T>::replace(T contender)
	{
		std::pop_heap(m_heap.begin(), m_heap.end());
		T largest = m_heap.back();
		m_heap.pop_back();

		T larger = contender;
		if (!(contender < largest))
			m_heap.push_back(largest);
		else {
			m_heap.push_back(contender);
			larger = largest;
		}
		std::make_heap(m_heap.begin(), m_heap.end());
		return larger;
	}


}
