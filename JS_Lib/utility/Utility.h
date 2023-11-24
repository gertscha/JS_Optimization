#ifndef UTILITY_UTILITY_H_
#define UTILITY_UTILITY_H_

#include <vector>
#include <string>
#include <random>
#include <numeric>


namespace JSOptimizer {
	namespace Utility {
    namespace {

      // remove an element by index efficiently (changes order of elements)
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

      std::string getFilenameFromPathString(std::string_view path) {
        std::string s = std::string(path);
        size_t pos = 0;
        while ((pos = s.find('/')) != std::string::npos) {
          s = s.substr(pos+1);
        }
        if ((pos = s.find('.')) != std::string::npos) {
          return s.substr(0, pos);
        }
        return s;
      }

      std::string getFilepathFromString(std::string_view path) {
        std::string s = std::string(path);
        std::string name = "";
        size_t pos = 0;
        while ((pos = s.find('/')) != std::string::npos) {
          name += s.substr(0, pos + 1);
          s.erase(0, pos + 1);
        }
        return name;
      }


    }
	}
}

#endif // !UTILITY_UTILITY_H_
