#ifndef UTILITY_RANDOM_H_
#define UTILITY_RANDOM_H_

#include <random>
#include <vector>
#include <numeric>

namespace JSOptimizer::Utility {
  
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

#endif // UTILITY_RANDOM_H_