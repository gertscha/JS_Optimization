#ifndef OPTIMIZER_BASE_MACHINEORDERREP_H_
#define OPTIMIZER_BASE_MACHINEORDERREP_H_

#include <string>
#include <vector>
#include <set>

#include "Optimizer.h"


namespace JSOptimizer
{

  // MachineOrderRepresentation
  /*
  * Search Space is the permutations of the machine sequences
  * includes many invalid schedules
  */
  class MachineOrderRep : public Optimizer
  {
  public:

    struct Identifier
    {
      Identifier(unsigned int id, unsigned int taskIndex)
        : job_id(id), index(taskIndex) {}
      unsigned int job_id;
      unsigned int index;
    }; // Identifier


    class MachineClique
    {
      friend MachineOrderRep;
    public:
      MachineClique() = delete;

      inline unsigned int getMachine() const { return machine_; }

      // multiset of job id's representing the order of tasks on a machine
      // job id's occur as often as the number of tasks's the job has on the machine
      inline std::vector<unsigned int>& getMachineOrder() { return machine_order_; }
      inline const std::vector<unsigned int>& getMachineOrder() const { return machine_order_; }

      // map to find the vertex_id in the graph associated with the task in the machine order
      // vertex_map[jobId] is the list of vertex_id's that are processed on the machine this
      // clique represents, they are in precedence order of the job
      // empty vector if a job has no tasks's on this machine
      inline const std::vector<std::vector<size_t>>& getVertexMap() const { return vertex_map_; }

      inline const std::set<size_t>& getCliqueMembers() const { return clique_members_; }

    private:
      unsigned int machine_;
      // vector of job id's
      std::vector<unsigned int> machine_order_;
      // maps job id's to lists of vertex id's
      std::vector<std::vector<size_t>> vertex_map_;
      // all the vertices in this clique
      std::set<size_t> clique_members_;

      // only meant to be constructed in the constructor of MachineOrderRep's
      MachineClique(unsigned int machineId, unsigned int jobCnt);
    }; // MachineClique


    MachineOrderRep(
      Problem* problem_pointer,
      const TerminationCriteria& termination_criteria,
      std::string name_prefix,
      unsigned int seed
    );

    virtual ~MachineOrderRep() {}


  protected:
    // machine count
    unsigned int m_count_;
    // total number of tasks
    size_t task_count_;
    // ids of all the tasks that are on a machine
    // indexed by machine ids
    std::vector<MachineClique> cliques_;
    // maps ids to tasks
    std::vector<Identifier> task_map_;

    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      // may return an uninitialized solution if the internal state cannot
      // be resolved due to deadlocks
      SolutionConstructor(
        const std::vector<MachineClique>& solution,
        const std::vector<Identifier>& map,
        const Problem* const problem,
        const std::string& prefix
      );
      SolutionConstructor(SolutionConstructor&& other) noexcept : Solution(other) {}
    }; // SolutionConstructor

  };

}

#endif // OPTIMIZER_BASE_MACHINEORDERREP_H_