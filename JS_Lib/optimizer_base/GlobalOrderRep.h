#ifndef OPTIMIZER_BASE_GLOBALORDERREP_H_
#define OPTIMIZER_BASE_GLOBALORDERREP_H_

#include <string>

#include "Optimizer.h"
#include "Solution.h"


namespace JSOptimizer {

  // GlobalOrderRepresentation
  /*
  * Optimizer of this kind use a list (length total number of Steps) as search space
  * entries in the list are task ids, which represent the precedence during the scheduling
  * of the steps to be processed on a machine offers utilities to subclasses to ease
  * impelmentation of optimizers that want to use this search space
  */
  class GlobalOrderRep : virtual public Optimizer
  {
  public:

    GlobalOrderRep(Problem* problem_pointer, Optimizer::TerminationCriteria& termination_criteria);

    virtual ~GlobalOrderRep() {}

    inline size_t getStepCount() const { return step_count_; }

  protected:
    // representation of the sequential solution to the problem
    std::vector<unsigned int> sequential_exec_;
    // length of a internal solution (total number of steps in the problem
    size_t step_count_; 


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