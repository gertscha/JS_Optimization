#ifndef OPTIMIZER_BASE_DISJUNCTIVEGRAPH_H_
#define OPTIMIZER_BASE_DISJUNCTIVEGRAPH_H_

#include "Optimizer.h"
#include "Solution.h"


namespace JSOptimizer {

  // DisjunctiveGraphRepresentation
  class GraphRep : virtual public Optimizer
  {
  public:

    GraphRep(Problem* problem_pointer, Optimizer::TerminationCriteria& termination_criteria);

    virtual ~GraphRep() {}


    class GraphMatrix {
    public:

    private:

    }; // GraphMatrix


    class InternalSolution
    {
    public:
      
      InternalSolution();

      long getMakespan() const { return makespan_; }
      size_t getTaskCount() const { return num_tasks_; }
      size_t getMachineCount() const { return num_machines_; }
      const std::string& getProblemName() const { return problem_name_; }

    protected:
      size_t num_tasks_;
      size_t num_machines_;
      std::string problem_name_;

      // the fitness value of this solution
      long makespan_;
    }; // InternalSolution


  protected:

    std::vector<GraphMatrix> machineSeq_;


    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const InternalSolution& solution, const std::string& prefix);
    }; // SolutionConstructor

  };
  

}

#endif // OPTIMIZER_BASE_DISJUNCTIVEGRAPH_H_