#ifndef UTILITY_HEAP_H_
#define UTILITY_HEAP_H_

#include <vector>
#include <algorithm>
#include <concepts>


/*///////////
    Heap
///////////*/

namespace JSOptimizer {

  namespace Utility {

    // operator< needs to be defined for T to be usable in a Heap
    template <typename T>
    concept Comparable = requires (T a, T b) { a < b; };

    template <typename T> requires Comparable<T>
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
      inline size_t size() { return heap_.size(); }
      // get vector of all elements in the heap
      inline const std::vector<T>& getElements() const { return heap_; }

    private:
      std::vector<T> heap_;
    };

  }

  template<typename T> requires Utility::Comparable<T>
  inline Utility::Heap<T>::Heap()
  {
    heap_ = std::vector<T>();
  }

  template<typename T> requires Utility::Comparable<T>
  inline void Utility::Heap<T>::add(T element)
  {
    if (!std::is_heap(heap_.begin(), heap_.end()))
      std::make_heap(heap_.begin(), heap_.end());

    heap_.push_back(element);
    std::push_heap(heap_.begin(), heap_.end());
  }

  template<typename T> requires Utility::Comparable<T>
  inline T Utility::Heap<T>::pop()
  {
    std::pop_heap(heap_.begin(), heap_.end()); // moves the largest to the end
    T largest = heap_.back();
    heap_.pop_back(); // actually removes the largest element
    return largest;
  }

  template<typename T> requires Utility::Comparable<T>
  inline T Utility::Heap<T>::peek()
  {
    std::pop_heap(heap_.begin(), heap_.end());
    T largest = heap_.back();
    std::make_heap(heap_.begin(), heap_.end());
    return largest;
  }

  template<typename T> requires Utility::Comparable<T>
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

#endif // UTILITY_HEAP_H_