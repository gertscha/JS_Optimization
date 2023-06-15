#ifndef UTILITY_FILECOLLECTOR_H_
#define UTILITY_FILECOLLECTOR_H_

#include <string>
#include <vector>
#include <iterator>
#include <cstddef> // for std::ptrdiff_t


namespace JSOptimizer::Utility {

  /*
  * Collect relative file paths for all files inside a directory
  * provides iterators to iterate through the filenames (as strings)
  */
  class FileCollector {
  public:

    struct Iterator {
      using iterator_category = std::bidirectional_iterator_tag;
      using difference_type = std::ptrdiff_t;
      using value_type = int;
      using pointer = std::string*;
      using reference = std::string&;

      Iterator() = default;
      Iterator(pointer ptr) : ptr_(ptr) {}

      reference operator*() const { return *ptr_; }
      pointer operator->() { return ptr_; }

      Iterator& operator++() { ptr_++; return *this; }
      Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }
      Iterator& operator+=(int i) { ptr_ += i; return *this; }
      Iterator& operator--() { ptr_--; return *this;}
      Iterator operator--(int) { Iterator tmp = *this; --(*this); return tmp; }
      Iterator& operator-=(int i) { ptr_ -= i; return *this; }

      friend bool operator== (const Iterator& a, const Iterator& b) { return a.ptr_ == b.ptr_; };
      friend bool operator!= (const Iterator& a, const Iterator& b) { return a.ptr_ != b.ptr_; };

    private:
      pointer ptr_;
    };


    // collect all .txt files contained in the folder 'folder_name' and
    // all subfolders of it, gets stores relative paths, including 'folder_name'
    // i.e. if at location 'root_path' there is a folder 'example_folder'
    // that contains files example1.txt, example2.txt and a subfolder 'ex_subfolder'
    // that contains file example3.txt, then the following strings will be stored:
    // - "example_folder/example1.txt"
    // - "example_folder/example2.txt"
    // - "example_folder/ex_subfolder/example3.txt"
    // files that are not .txt files will be ignored!
    FileCollector(std::string root_path, std::string folder_name);

    Iterator begin() { return Iterator(&files_[0]); }
    Iterator end() { return Iterator(&files_.back() + 1); }

  private:
    std::vector<std::string> files_;


  };

}

#endif // UTILITY_FILECOLLECTOR_H_