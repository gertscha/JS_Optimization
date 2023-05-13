#ifndef UTILITY_PARSER_H_
#define UTILITY_PARSER_H_

#include <concepts>
#include <tuple>
#include <vector>
#include <sstream>
#include <iostream>


namespace JSOptimizer::Utility {

  namespace Private
  {

    // Helper function to recursively access tuple elements
    template <size_t I, typename... Ints>
    void read_tuple(std::istringstream& iss, std::tuple<Ints...>& t) noexcept(false)
    {
      if (!(iss >> std::get<I>(t))) {
        // Error: failed to read value from input stream
        throw std::runtime_error("Failed to read value from input stream");
      }
      if constexpr (I + 1 < sizeof...(Ints)) {
        read_tuple<I + 1>(iss, t);
      }
    }

    template<typename TupType, size_t... I>
    void print(std::ostream& out, const TupType& _tup, std::index_sequence<I...>)
    {
      out << "(";
      (..., (out << (I == 0 ? "" : ", ") << std::get<I>(_tup)));
      out << "), ";
    }

  }

  // export parsing function
  template <typename... Ints>
  unsigned int parseTuples(std::istringstream& iss, std::vector<std::tuple<Ints...>>& result) noexcept(false)
  {
    static_assert((std::is_integral_v<Ints> && ...), "All types must be integral");

    unsigned int tuple_count = 0;
    while (!iss.eof()) {
      std::tuple<Ints...> group;
      Private::read_tuple<0>(iss, group);
      result.push_back(group);
      ++tuple_count;
    }
    return tuple_count;
  }


  // export printing function
  template<typename... T>
  void printTuple(std::ostream& out, const std::tuple<T...>& _tup)
  {
    Private::print(out, _tup, std::make_index_sequence<sizeof...(T)>());
  }


}

#endif // UTILITY_PARSER_H_
