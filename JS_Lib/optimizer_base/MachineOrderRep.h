#ifndef OPTIMIZER_BASE_MACHINEORDERREP_H_
#define OPTIMIZER_BASE_MACHINEORDERREP_H_

#include <string>
#include <vector>
#include <set>

#include "Optimizer.h"


namespace JSOptimizer {

  // MachineOrderRepresentation
  /*
  *
  */
  class MachineOrderRep : public Optimizer
  {
  public:

    struct Identifier {
      Identifier(unsigned int id, unsigned int stepIndex)
        : task_id(id), index(stepIndex) {}
      unsigned int task_id;
      unsigned int index;
    }; // Identifier


    class MachineClique {
      friend MachineOrderRep;
    public:
      MachineClique() = delete;

      inline unsigned int getMachine() const { return machine_; }

      // multiset of task id's representing the order of steps on a machine
      // task id's occur as ofen as the number of step's the task has on the machine
      inline std::vector<unsigned int>& getMachineOrder() { return machine_order_; }
      inline const std::vector<unsigned int>& getMachineOrder() const { return machine_order_; }

      // map to find the vertex_id in the graph associated with the step in the machine order
      // vertex_map[taskId] is the list of vertex_id's that are processed on the machine this
      // clique represents, they are in precedence order of the task
      // empty vector if a task has no step's on this machine
      inline const std::vector<std::vector<size_t>>& getVertexMap() const { return vertex_map_; }

      inline const std::set<size_t>& getCliqueMembers() const { return clique_members_; }

    private:
      unsigned int machine_;
      // vector of task id's
      std::vector<unsigned int> machine_order_;
      // maps task id's to lists of vertex id's
      std::vector<std::vector<size_t>> vertex_map_;
      // all the vertices in this clique
      std::set<size_t> clique_members_;

      // only meant to be constructed in the constructor of MachineOrderRep's
      MachineClique(unsigned int machineId, unsigned int taskCnt);
    }; // MachineClique


    MachineOrderRep(Problem* problem_pointer, const TerminationCriteria& termination_criteria,
      std::string name_prefix, unsigned int seed);

    virtual ~MachineOrderRep() {}


  protected:
    
    unsigned int m_count_;

    size_t step_count_;

    std::vector<MachineClique> cliques_;

    std::vector<Identifier> step_map_;


    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const std::vector<MachineClique>& solution,
        const std::vector<Identifier>& map,
        const Problem* const problem, 
        const std::string& prefix);
      SolutionConstructor(SolutionConstructor&& other) noexcept : Solution(other) {}
    }; // SolutionConstructor

  };


}


#endif // OPTIMIZER_BASE_MACHINEORDERREP_H_