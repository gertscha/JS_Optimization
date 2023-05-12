#ifndef UTILITY_UTILITY_H_
#define UTILITY_UTILITY_H_

#include <string>
#include <tuple>
#include <concepts>
#include <random>
#include <algorithm>
#include <sstream>

#include "loguru.hpp"



namespace JSOptimizer {

	/*
	* Declarations of the Utility Functions & Objects
	*/
	namespace Utility {

		template <typename T>
		concept Integer = std::integral<T>;

		template <Integer... T>
		int parseTuples(std::istringstream line, int expected_tuple_count, std::vector<std::tuple<T...>>& result_tuples, bool clear_vector);

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

	/*
	* Implementations of the Utility Functions & Objects
	*/


	/*//////////////
		Parsing
	//////////////*/


	template<Utility::Integer ...T>
	int Utility::parseTuples(std::istringstream line, int expected, std::vector<std::tuple<T...>>& result, bool clear_vector)
	{
		if (clear_vector)
			result.clear();
		
		size_t tuple_size = std::tuple_size<std::tuple<T...>>{};

		unsigned int tuple_count = 0;
		T current = 0;

		while (in_ss >> current) // read first value
		{
			if (in_ss.fail())
				break;
			if (tuple_count >= expected)
				ABORT_F("on line %i there are more pairs than expected", (commentCount + 2 + task_index));
			// read the duration
			in_ss >> duration;
			// check all valid
			if (in_ss.fail())
				ABORT_F("on line %i pair %i is bad", (commentCount + 2 + task_index), (tuple_count + 1));
			if (machine < 0 || duration < 0)
				ABORT_F("only postive numbers allowed in pair %i on line %i", (tuple_count + 1), (commentCount + 2 + task_index));
			if (machine >= machine_count_)
				ABORT_F("invalid machine on line %i pair %i", (commentCount + 2 + task_index), (tuple_count + 1));

			tasks_.back().AppendStep(machine, duration);

			if (!machine_has_steps[machine])
				machine_has_steps[machine] = true;

			in_ss.ignore(1, ',');
			++tuple_count;
		}



		LOG_F(INFO, "tuple size is %i", tuple_size);

		return tuple_count;
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

#endif // !UTILITY_UTILITY_H_
