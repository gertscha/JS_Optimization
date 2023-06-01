#ifndef OPTIMIZER_BASE_GRAPHREP_H_
#define OPTIMIZER_BASE_GRAPHREP_H_

#include <vector>
#include <optional>
#include <tuple>
#include <set>
#include <string>

#include "Optimizer.h"
#include "Solution.h"


namespace JSOptimizer {

  // GraphRepresentation
  /*
  * Optimizer of this kind use graph as search space, precedences between Steps are
  * edges in the graph, can differentiate between fixed Task precedences and mutable
  * machine precendeces, offers utilities to subclasses to ease impelmentation of
  * optimizers that want to use this search space
  */
  class GraphRep : virtual public Optimizer
  {
  public:

    struct Identifier {
      Identifier(unsigned int id, unsigned int stepIndex)
        : taskId(id), index(stepIndex) {}
      unsigned int taskId;
      unsigned int index;
    }; // Identifier


    class MachineClique {
      friend GraphRep;
    public:
      MachineClique() = delete;

      inline unsigned int getMachine() const { return machine_; }

      // multiset of task id's representing the order of steps on a machine
      // task id's occur as ofen as the number of step's the task has on the machine
      inline std::vector<unsigned int>& getMachineOrder() { return machine_order_; }

      // map to find the vertex_id in the graph associated with the step in the machine order
      // vertex_map[taskId] is the list of vertex_id's that are processed on the machine this
      // clique represents, they are in precedence order of the task
      // empty vector if a task has no step's on this machine
      inline const std::vector<std::vector<size_t>>& getVertexMap() const { return vertex_map_; }

    private:
      unsigned int machine_;
      // vector of task id's
      std::vector<unsigned int> machine_order_;
      // maps task id's to lists of vertex id's
      std::vector<std::vector<size_t>> vertex_map_;
      // only meant to be constructed in the constructor of GraphRep's
      MachineClique(unsigned int machineId, unsigned int taskCnt);
    }; // MachineClique


    /*
    * Stores Timing information for a GraphRep, tightly bound to a single GraphRep
    */
    class PathsInfo {
      friend class GraphRep;
    public:
      struct Timing {
        Timing() : ESD(0), EFD(0), LSD(0), LFD(0), FF(0), TF(0) {}
        unsigned int ESD; // earliest start date
        unsigned int EFD; // earliest finish date
        unsigned int LSD; // latest start date 
        unsigned int LFD; // latest finish date
        unsigned int FF; // free float, does not affect ESD of any succeeding activity
        unsigned int TF; // total float, does not affect LSD of any succeeding activity
        // print members in declartation order above
        void print(std::ostream& os) const;
      }; // Timing

      // check if timings match current graph state
      inline bool isCurrent() const { return seqno_ == parent_->seqno_; }

      // update timings and critical Path
      void update() {
        if (!isCurrent()) {
          updateTimings();
          updateCriticalPath();
        }
      }

      // lenght of critical path, may be outdated or -1 if isCurrent() returns false
      inline long getMakespan() const { return cp_length_; }
      // vertices of a critical path in topological ordering, may be outdated or empty if isCurrent() returns false
      // there may be multiple critical paths, this method returns a single one
      inline const std::vector<size_t>& getCriticalPath() const { return critical_path_; }
      // get timing for all vertices in the graph, may be outdated or empty if isCurrent() returns false
      inline const std::vector<Timing>& getTimings() const { return timings_; }

    private:
      const GraphRep* const parent_;
      unsigned int seqno_;
      long cp_length_;
      // a critical Path, all vertices on it topologically sorted
      std::vector<size_t> critical_path_;
      // map vertices in vertices_ to timings
      std::vector<Timing> timings_;
      // construction only happens in GraphRep Constructor
      PathsInfo(GraphRep* parent)
        : parent_(parent), seqno_(0), cp_length_(-1)
      {
        critical_path_ = std::vector<size_t>();
        timings_ = std::vector<Timing>();
      }
      void updateTimings();
      void updateCriticalPath();
    }; // PathsInfo



    GraphRep(Problem* problem_pointer, Optimizer::TerminationCriteria& termination_criteria);

    virtual ~GraphRep() {}

    // discards all current machine precedences and sets them according to the cliques
    void applyCliqueOrdersToGraph();
    // checks if successor lists are acyclic
    bool containsCycle();
    // checks if target is reachable from source, if return_a_path is set, it will also return a
    // vector containing a list of vertices constituting a path from source to target in the std::optional
    std::pair<bool, std::optional<std::vector<size_t>>> reachable(size_t source, size_t target, bool return_a_path = false);

    // debug
    void printVertexRelations(std::ostream& os) const;
    void printStepMap(std::ostream& os) const;


  protected:
    // number of steps + 2, index 0 is the source, index vertex_count - 1 is the sink
    size_t vertex_count_;
    // ensure that modifications are tracked to report PathsInfo status correctly
    unsigned int seqno_;
    // cliques for each machine, indexed by machine id's
    std::vector<MachineClique> cliques_;
    // store Timing information, is tightly bound
    PathsInfo graph_paths_info_;
    // successor and predecessor list combined
    // positive values encodes Task sucessors, negative values encode predecessors
    // machine relations are encoded by by first adding vertex_count_ to the vertex id
    // 0 can only be a predecessor to a task
    std::vector<std::vector<long>> graph_;
    // only has task precedence edges
    std::vector<std::vector<long>> graph_only_task_pred_;
    // maps vertex_id's to steps, index 0 is the source, the last entry is the sink
    // source is (UINT_MAX,0), sink is (0,UINT_MAX)
    // source and sink have no step assoicated with them
    std::vector<Identifier> step_map_;
    // map vertex id's to durations of the corresponding step
    std::vector<unsigned int> duration_map_;

    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const std::vector<std::vector<long>>& graph, const std::vector<Identifier>& map,
                            const Problem* const problem, const std::string& prefix);
    }; // SolutionConstructor

    // helper functions
    static void addPredecessorsToSet(size_t vertex, std::set<size_t>& set, const std::vector<std::vector<long>>& graph);
    static void addSuccessorsToSet(size_t vertex, std::set<size_t>& set, const std::vector<std::vector<long>>& graph);
    static bool checkPredecessorsInSet(size_t vertex, const std::set<size_t>& set, const std::vector<std::vector<long>>& graph);
    static bool checkSuccessorsInSet(size_t vertex, const std::set<size_t>& set, const std::vector<std::vector<long>>& graph);

  private:
    // reachable check using only successor edges
    bool reachable_intern(size_t source, size_t target);


  };

}

#endif // OPTIMIZER_BASE_GRAPHREP_H_