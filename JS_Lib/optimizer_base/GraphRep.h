#ifndef OPTIMIZER_BASE_GRAPHREP_H_
#define OPTIMIZER_BASE_GRAPHREP_H_

#include <vector>

#include "Optimizer.h"
#include "Solution.h"


namespace JSOptimizer {

  // DisjunctiveGraphRepresentation
  class GraphRep : virtual public Optimizer
  {
  public:

    struct Identifier {
      Identifier(unsigned int id, size_t stepIndex)
        : taskId(id), index(stepIndex) {}
      unsigned int taskId;
      size_t index;
    };

    class MachineClique {
      friend GraphRep;
    public:
      MachineClique() = delete;

      unsigned int getMachine() { return machine_; }

    protected:
      // vector of task id's
      std::vector<unsigned int> machine_order_;

      // maps task id's to lists of vertex id's
      std::vector<std::vector<size_t>> vertex_map_;

    private:
      MachineClique(unsigned int machineId, unsigned int taskCnt);
      
      unsigned int machine_;
    }; // MachineClique


    GraphRep(Problem* problem_pointer, Optimizer::TerminationCriteria& termination_criteria);

    virtual ~GraphRep() {}

    void applyCliqueOrdersToGraph();

    // updates makespan and critical path
    void calculateCurrentPaths();

    std::vector<unsigned int>& getCriticalPath() { return critical_path_; }

    long getMakespan() const { return makespan_; }

    // debug
    void debugPrintGraph();

    // temp to allow for debug instances of this class to be created
    virtual void Initialize() {}
    virtual void Iterate() {}
    virtual bool CheckTermination() { return false; }
    virtual const Solution& getBestSolution() { return Solution(); }


  protected:

    long makespan_;

    // number of steps + 2, index 0 is the source, index vertex_count - 1 is the sink
    size_t vertex_count_;

    std::vector<MachineClique> cliques_;

    std::vector<unsigned int> critical_path_;

    // successor and predecessor list combined
    // [1,vertexCnt] encodes Task sucessors, the same negative range encodes predecessors
    // 0 can only be a predecessor to a task
    // [vertexCnt, 2*vertexCnt] encodes machine successors, negative again predecessors
    std::vector<std::vector<long>> graph_;

    std::vector<std::vector<long>> graph_only_task_pred_;


    // maps vertex_id's to steps, index 0 is invalid (contains UINT_MAX,0)
    // becasue sink has id 0 and it has no step associated with it
    std::vector<Identifier> map_to_steps_;



    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const std::vector<std::vector<long>>& graph, const std::vector<Identifier>& map,
                            const Problem* const problem, const std::string& prefix);
    }; // SolutionConstructor

  };
  

}

#endif // OPTIMIZER_BASE_GRAPHREP_H_