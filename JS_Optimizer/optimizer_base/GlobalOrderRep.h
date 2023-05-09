#ifndef OPTIMIZER_BASE_GLOBALORDERREP_H_
#define OPTIMIZER_BASE_GLOBALORDERREP_H_

#include <string>

#include "Optimizer.h"
#include "Solution.h"


namespace JSOptimizer {

  // GlobalOrderRepresentation
  class GlobalOrderRep : virtual public Optimizer
  {
  public:

    GlobalOrderRep(Problem* problem_pointer, Optimizer::TerminationCriteria& termination_criteria);

    virtual ~GlobalOrderRep() {}


    class InternalSolution
    {
    public:
      struct Step {
        unsigned int task_id;
        size_t step_index;
        unsigned int duration;
        long end_time;

        // has constructor to allow efficient creation with emplace_back on vectors
        Step(unsigned int taskId, size_t stepIndex, unsigned int duration, long endTime)
          :task_id(taskId), step_index(stepIndex), duration(duration), end_time(endTime)
        {}
      };

      // assumes solution is well formed (does not perform any checks)
      InternalSolution(const std::vector<unsigned int>& internal_sol_state, const Problem& problem);

      long getMakespan() const { return makespan_; }
      size_t getTaskCount() const { return num_tasks_; }
      size_t getMachineCount() const { return num_machines_; }
      const std::string& getProblemName() const { return problem_name_; }

      const std::vector<std::vector<GlobalOrderRep::InternalSolution::Step>>& getSteps() const { return internal_sol_steps_; }

    private:
      size_t num_tasks_;
      size_t num_machines_;
      std::string problem_name_;
      // rows correspond to machines, columns to steps, in order
      std::vector<std::vector<Step>> internal_sol_steps_;
      // the fitness value of this solution
      long makespan_;
    }; // InternalSolution

  protected:
    // representation of the sequential solution to the problem
    std::vector<unsigned int> sequential_exec_;
    // length of a internal solution (total number of steps in the problem
    size_t step_count_; 


    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const InternalSolution& solution, const std::string& prefix);
    }; // SolutionConstructor

  };


}

#endif  // OPTIMIZER_BASE_GLOBALORDERREP_H_