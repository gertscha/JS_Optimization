#ifndef UTILITY_THREADMANAGER_H_
#define UTILITY_THREADMANAGER_H_

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>


namespace JSOptimizer::Utility {


  //  Manage Threads, by joining them back once the Manager goes out of scope
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

    template<typename T>
    void addThreadToQueue(T function_ptr, const std::vector<std::string>& arg) {

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
    std::queue<std::thread> queue_;

    std::thread runner_;

    std::condition_variable ready_;
    std::mutex worker_;

    void startNextTask() {

    }

  };

}

#endif // UTILITY_THREADMANAGER_H_