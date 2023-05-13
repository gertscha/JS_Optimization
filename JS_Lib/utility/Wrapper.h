#ifndef UTILITY_WRAPPER_H_
#define UTILITY_WRAPPER_H_


/*////////////////////
    Value Wrapper
////////////////////*/

namespace JSOptimizer::Utility {

  template<typename T>
  struct Wrapper {
    T* pointer;
    Wrapper(T* pointer) :pointer(pointer) {}
    bool operator<(const Wrapper<T>& rhs) { return false; } // specialize if needed
  };


}

#endif // UTILITY_WRAPPER_H_