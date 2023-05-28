#ifndef UTILITY_WRAPPER_H_
#define UTILITY_WRAPPER_H_

#include <memory>

/*////////////////////
    Value Wrapper
////////////////////*/

namespace JSOptimizer::Utility {

  template<typename T>
  struct Wrapper {
    std::shared_ptr<T> pointer;
    Wrapper() : pointer(std::make_shared(nullptr)) {}
    Wrapper(std::shared_ptr<T> pointer) : pointer(pointer) {}
    bool operator<(const Wrapper<T>& rhs) { return false; } // specialize if needed
    bool operator>(const Wrapper<T>& rhs) { return false; } // specialize if needed
  };


}

#endif // UTILITY_WRAPPER_H_