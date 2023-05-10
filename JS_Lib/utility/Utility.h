#pragma once

#include <string>
#include <random>
#include <algorithm>
//#include <concepts>
//#include <tuple>


namespace JSOptimizer {

	template <typename T>
	concept Number = std::integral<T> || std::floating_point<T>;


	namespace Utility {

		// visualizes a Solution saved as a file using python/matplotlib
		void visualize(const std::string& sourceFolder, const std::string& sourceName, const std::string& outputFolder);

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
			T* pointer;
			Wrapper(T* pointer) :pointer(pointer) {}
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
			size_t size() { return heap_.size(); }
			// get vector of all elements in the heap
			const std::vector<T>& getElements() const { return heap_; }

		private:
			std::vector<T> heap_;
		};


	}


	/*///////////
		Heap 
	///////////*/


	template<typename T>
	inline Utility::Heap<T>::Heap()
	{
		heap_ = std::vector<T>();
	}

	template<typename T>
	inline void Utility::Heap<T>::add(T element)
	{
		if (!std::is_heap(heap_.begin(), heap_.end()))
			std::make_heap(heap_.begin(), heap_.end());
        
		heap_.push_back(element);
		std::push_heap(heap_.begin(), heap_.end());
	}

	template<typename T>
	inline T Utility::Heap<T>::pop()
	{
		std::pop_heap(heap_.begin(), heap_.end()); // moves the largest to the end
		T largest = heap_.back();
		heap_.pop_back(); // actually removes the largest element
		return largest;
	}

	template<typename T>
	inline T Utility::Heap<T>::peek()
	{
		std::pop_heap(heap_.begin(), heap_.end());
		T largest = heap_.back();
		std::make_heap(heap_.begin(), heap_.end());
		return largest;
	}

	template<typename T>
	inline T Utility::Heap<T>::replace(T contender)
	{
		std::pop_heap(heap_.begin(), heap_.end());
		T largest = heap_.back();
		heap_.pop_back();

		T larger = contender;
		if (largest < contender)
			heap_.push_back(largest);
		else {
			heap_.push_back(contender);
			larger = largest;
		}
		std::make_heap(heap_.begin(), heap_.end());
		return larger;
	}


}
