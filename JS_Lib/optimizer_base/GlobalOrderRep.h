#ifndef OPTIMIZER_BASE_GLOBALORDERREP_H_
#define OPTIMIZER_BASE_GLOBALORDERREP_H_

#include <string>

#include "Optimizer.h"


namespace JSOptimizer {

  // GlobalOrderRepresentation
  /*
  * Optimizer of this kind use a list (length total number of Tasks) as search space,
  * entries in the list are job ids, which represent the precedence during the scheduling
  * of the tasks to be processed on a machine offers utilities to subclasses to ease
  * implementation of optimizers that want to use this search space
  */
  class GlobalOrderRep : public Optimizer
  {
  public:

    GlobalOrderRep(Problem* problem_pointer, const TerminationCriteria& termination_criteria,
                   std::string name_prefix, unsigned int seed);

    virtual ~GlobalOrderRep() {}

    inline size_t getTotalTaskCount() const { return task_count_; }


  protected:
    // representation of the sequential solution to the problem
    std::vector<unsigned int> sequential_exec_;
    // length of a internal solution (total number of tasks in the problem)
    size_t task_count_; 


    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const std::vector<unsigned int>& solution, const Problem* const problem, const std::string& prefix);
      SolutionConstructor(SolutionConstructor&& other) noexcept : Solution(other) {}
    }; // SolutionConstructor

  };


}

#endif  // OPTIMIZER_BASE_GLOBALORDERREP_H_