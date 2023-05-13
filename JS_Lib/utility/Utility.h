#ifndef UTILITY_UTILITY_H_
#define UTILITY_UTILITY_H_


namespace JSOptimizer {

	namespace Utility {

		// remove an element efficently (changes order of elements)
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



	}

}

#endif // !UTILITY_UTILITY_H_
