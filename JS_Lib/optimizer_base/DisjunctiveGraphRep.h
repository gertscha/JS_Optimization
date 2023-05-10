#ifndef OPTIMIZER_BASE_DISJUNCTIVEGRAPH_H_
#define OPTIMIZER_BASE_DISJUNCTIVEGRAPH_H_

#include "Optimizer.h"
#include "Solution.h"


namespace JSOptimizer {

  // DisjunctiveGraphRepresentation
  class DisjunctiveGraphRep : virtual public Optimizer
  {
  public:

    DisjunctiveGraphRep(Problem* problem_pointer, Optimizer::TerminationCriteria& termination_criteria);

    virtual ~DisjunctiveGraphRep() {}


    class Vertex {

    }; // Vertex

    class Edge {

    }; // Edge

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

    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const InternalSolution& solution, const std::string& prefix);
    };

  };
  

}

#endif // OPTIMIZER_BASE_DISJUNCTIVEGRAPH_H_