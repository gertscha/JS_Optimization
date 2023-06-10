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
        : task_id(id), index(stepIndex) {}
      unsigned int task_id;
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


    class DacExtender {
    public:
      // takes DAC as input
      DacExtender(const std::vector<std::vector<long>>& graph);
      ~DacExtender();

      // determine direction of edge from vertex1 to vertex2 that maintains DAC
      std::pair<size_t, size_t> insertEdge(size_t vertex1, size_t vertex2);

    private:
      struct Node {
        Node* prev_ptr;
        Node* next_ptr;
        unsigned int position;
        // ('vertex id', 'successor node pointer') pairs
        std::set<size_t> vertices;

        Node() : prev_ptr(nullptr), next_ptr(nullptr), position(0)
        {
          vertices = std::set<size_t>();
        }
        Node(Node* previous, Node* next, unsigned int position)
          : prev_ptr(previous), next_ptr(next), position(position)
        {
          vertices = std::set<size_t>();
        }
      }; // Node

      // alias for the first element in the vertex_node_map_
      Node* source_;
      // map vertices to nodes
      std::vector<Node*> vertex_node_map_;
      // map vertices to closest successor vertex
      std::vector<long> successor_map_;

      // start->next_ptr is the first node that gets incremented
      void incrementPositionOfAllSuccessors(Node* start);
      // debug only verification that everything is valid
      void debugVerifyIntegrity(const std::vector<std::vector<long>>& graph);

    }; // DacExtender


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

      // update timings and critical Path
      void update() {
        if (parent_->modified_flag) {
          updateTimings();
          updateCriticalPath();
        }
      }

      // lenght of critical path, may be outdated or -1 if isCurrent() returns false
      inline long getMakespan() const { return cp_length_; }
      // vertices of a critical path in topological ordering, may be outdated or empty
      // there may be multiple critical paths, this method returns a single one
      inline const std::vector<size_t>& getCriticalPath() const { return critical_path_; }
      // get timing for all vertices in the graph, may be outdated or empty
      inline const std::vector<Timing>& getTimings() const { return timings_; }

    private:
      GraphRep* const parent_;
      long cp_length_;
      // a critical Path, all vertices on it topologically sorted
      std::vector<size_t> critical_path_;
      // map vertices in vertices_ to timings
      std::vector<Timing> timings_;
      // construction only happens in GraphRep Constructor
      PathsInfo(GraphRep* parent)
        : parent_(parent), cp_length_(-1)
      {
        critical_path_ = std::vector<size_t>();
        timings_ = std::vector<Timing>();
      }
      void updateCriticalPath();
      void updateTimings();
      // Critical Path Method Forward Pass (calculate ESD, EFD)
      // expects vertex_count length (default constructed) timings_
      void doCPMForwardPass();
      // Critical Path Method Backwards Pass (calculate LSD, LFD, FF, TF)
      // need to have ESD, EFD already calculated and Sink Timings completely initalized
      void doCPMBackwardPass();
    }; // PathsInfo


    GraphRep(Problem* problem_pointer, Optimizer::TerminationCriteria& criteria);

    virtual ~GraphRep() {}

    // apply a clique to the graph, adds elevated edges
    void applyCliqueToGraph(const MachineClique& clique);
    // discards all current machine precedences and sets them according to all the cliques
    void applyAllCliquesToGraph();
    // checks if successor lists are acyclic
    bool containsCycle() const;
    // checks if target is reachable from source, the first element of the pair holds
    // the result of this check if return_a_path is set, the std::optional contains a
    // path (list of vertices constituting a path from source to target) should there
    // be one, if (!return_a_path || !rachable) the optional will be empty
    std::pair<bool, std::optional<std::vector<size_t>>> reachable(
        size_t source, size_t target, bool return_a_path = false) const;

    // debug
    void printVertexRelations(std::ostream& os) const;
    void printStepMap(std::ostream& os) const;


  protected:

    void markModified() { modified_flag = true; }


    // ensure that modifications are tracked to avoid PathsInfo recalculations
    bool modified_flag = true;
    // set by containsCycle() to be the vertex that is the the first root that is found
    mutable size_t cycle_root_ = 0;
    // number of steps + 2, index 0 is the source, index vertex_count - 1 is the sink
    size_t vertex_count_;
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


    // helper functions
    // true for sucessors, changes the value if it is elevated to be a valid index
    static bool filterForSuccessors(long& vertex, const std::vector<std::vector<long>>& graph);
    // true for predecessors, changes the value if it is elevated to be a valid index
    static bool filterForPredecessors(long& vertex, const std::vector<std::vector<long>>& graph);

    static void addPredecessorsToSet(size_t vertex, std::set<size_t>& set,
                                     const std::vector<std::vector<long>>& graph);
    static void addSuccessorsToSet(size_t vertex, std::set<size_t>& set,
                                   const std::vector<std::vector<long>>& graph);
    static bool checkAllPredecessorsInSet(size_t vertex, const std::set<size_t>& set,
                                       const std::vector<std::vector<long>>& graph);
    static bool checkAllSuccessorsInSet(size_t vertex, const std::set<size_t>& set,
                                        const std::vector<std::vector<long>>& graph);
    static bool checkIfAPredecessorInSet(size_t vertex, const std::set<size_t>& set,
                                         const std::vector<std::vector<long>>& graph);
    static bool checkIfASuccessorInSet(size_t vertex, const std::set<size_t>& set,
                                       const std::vector<std::vector<long>>& graph);
    static bool unionIsEmpty(const std::set<size_t>& one, const std::set<size_t>& two);

    

    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(const std::vector<std::vector<long>>& graph,
        const std::vector<Identifier>& map,
        const Problem* const problem,
        const std::string& prefix);
    }; // SolutionConstructor


  private:
    // reachable check using only successor edges, gives path, empty if not reachable
    bool reachable_intern(size_t source, size_t target, bool give_path,
                          std::vector<size_t>& return_path) const;
    // used in constructor
    void initialzeGraphAndState();

  };

}

#endif // OPTIMIZER_BASE_GRAPHREP_H_