#ifndef UTILITY_THREADMANAGER_H_
#define UTILITY_THREADMANAGER_H_

#include <vector>
#include <thread>
#include <iostream>

/*
  Manage Threads, by joining them back once the Manager goes out of scope
*/
class ThreadManager {

public:
  ThreadManager() {
    threads_ = std::vector<std::thread*>();
  }

  // letting the threadManager go out of scope in main() causes problems
  // call joinAll() to be safe
  ~ThreadManager() {
    for (std::thread* th_ptr : threads_) {
      th_ptr->join();
      delete th_ptr;
    }
  }

  void addThread(std::thread* th) {
    threads_.push_back(th);
  }

  void joinAll() {
    for (std::thread* th_ptr : threads_) {
      th_ptr->join();
      delete th_ptr;
    }
    threads_.clear();
  }

private:
    
  std::vector<std::thread*> threads_;

};


#endif // UTILITY_THREADMANAGER_H_