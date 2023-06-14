#ifndef UTILITY_UTILITY_H_
#define UTILITY_UTILITY_H_

#include <vector>
#include <random>
#include <numeric>


namespace JSOptimizer {

	namespace Utility {

		// remove an element by index efficently (changes order of elements)
		template <typename T>
		T remove_at(std::vector<T>& v, typename std::vector<T>::size_type n)
		{
			T ans = std::move_if_noexcept(v[n]);
			v[n] = std::move_if_noexcept(v.back());
			v.pop_back();
			return ans;
		}

    template<typename T>
    std::vector<T> randomPullUniqueFromRange(T lower_bound, T upper_bound,
      size_t count, std::mt19937_64& generator) {
      auto range = std::vector<T>(upper_bound - lower_bound + 1);
      std::iota(range.begin(), range.end(), lower_bound);
      std::shuffle(range.begin(), range.end(), generator);
      range.resize(count);
      return range;
    }

	}

}

#endif // !UTILITY_UTILITY_H_
