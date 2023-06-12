#ifndef UTILITY_RANDOM_H_
#define UTILITY_RANDOM_H_

#include <random>
#include <vector>

namespace JSOptimizer::Utility {
  
  template<typename T>
  std::vector<T> randomPullUnique(const std::vector<T>& source, size_t num_random,
      std::mt19937_64& generator) {
    auto out = std::vector<T>();
    std::ranges::sample(source.begin(), source.end(), std::back_inserter(out),
      num_random, generator);
    return out;
  }

}

#endif // UTILITY_RANDOM_H_