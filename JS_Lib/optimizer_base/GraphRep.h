#ifndef OPTIMIZER_BASE_GRAPHREP_H_
#define OPTIMIZER_BASE_GRAPHREP_H_

#include <vector>
#include <string>
#include <optional>
#include <set>

#include "Optimizer.h"
#include "Job.h"


namespace JSOptimizer
{

  // GraphRepresentation
  /*
  * Optimizer of this kind use graph as search space, precedences between Tasks are
  * edges in the graph, can differentiate between fixed Job precedences and mutable
  * machine precedences, offers utilities to subclasses to ease implementation of
  * optimizers that want to use this search space
  */
  class GraphRep : public Optimizer
  {
  public:

    struct Identifier
    {
      Identifier(unsigned int id, unsigned int taskIndex)
        : job_id(id), index(taskIndex) {}
      unsigned int job_id;
      unsigned int index;
    }; // Identifier


    class DacExtender
    {
    public:
      // takes DAC as input
      DacExtender(const std::vector<std::vector<long>>& graph);
      DacExtender() : source_(nullptr) {} // only suitable as place-holder object
      DacExtender(const DacExtender& other);
      DacExtender& operator=(const DacExtender& other);
      ~DacExtender();

      // give directed edge from undirected edge between vertex1 to vertex2 that maintains DAC
      // uses the easier edge in terms of effort, uses edge (vertex1, vertex2) to tie break
      std::pair<size_t, size_t> InsertEdge(size_t vertex1, size_t vertex2);

    private:
      struct Node
      {
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
        Node(const Node& other) = default;
      }; // Node

      // alias for the first element in the vertex_node_map_
      Node* source_;
      // map vertices to nodes
      std::vector<Node*> node_vertex_map_;
      // map vertices to successor vertices, is a list
      // invariant: the vertex that is the closest successor is always at index 0
      std::vector<std::vector<size_t>> successor_map_;

      // start->next_ptr is the first node that gets incremented
      void IncrementPositionOfAllSuccessors(Node* start);
      // ensures invariant holds by updating the list for all vertices that had
      // modified as closest successor, the check_all flag changes the behaviour
      // if set, then a comprehensive update is done, all vertices are checked
      void MaintainInvarinatSuccessorMapMovedVertex(size_t modified, bool check_all = false);
      // ensures invariant holds for a single vertex' successor list
      void MaintainInvarinatSuccessorMapAddedSuccessor(size_t modified);
      // debug only verification that everything is valid
      void debugVerifyIntegrity(const std::vector<std::vector<long>>& graph);

    }; // DacExtender


    // Stores Timing information for a GraphRep, tightly bound to a single GraphRep
    class PathsInfo
    {
      friend class GraphRep;
    public:
      struct Timing
      {
        Timing() : ESD(0), EFD(0), LSD(0), LFD(0), FF(0), TF(0) {}
        unsigned int ESD; // earliest start date
        unsigned int EFD; // earliest finish date
        unsigned int LSD; // latest start date 
        unsigned int LFD; // latest finish date
        unsigned int FF; // free float, does not affect ESD of any succeeding activity
        unsigned int TF; // total float, does not affect LSD of any succeeding activity
        // print members in declaration order above
        void print(std::ostream& os) const;
      }; // Timing

      // update timings and critical Path
      void Update()
      {
        if (parent_->modified_flag) {
          UpdateTimings();
          UpdateCriticalPath();
        }
      }
      inline bool isCurrent() { return !parent_->modified_flag; }
      // length of critical path, may be outdated or -1 if isCurrent() returns false
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
      void UpdateCriticalPath();
      void UpdateTimings();
      // Critical Path Method Forward Pass (calculate ESD, EFD)
      // expects vertex_count length (default constructed) timings_
      void DoCPMForwardPass();
      // Critical Path Method Backwards Pass (calculate LSD, LFD, FF, TF)
      // need to have ESD, EFD already calculated and Sink Timings completely initialized
      void DoCPMBackwardPass();
    }; // PathsInfo


    GraphRep(
      Problem* problem_pointer,
      const TerminationCriteria& criteria,
      std::string name_prefix,
      unsigned int seed
    );

    virtual ~GraphRep() {}

    // checks if successor lists are acyclic (a vertex that is part of the cycle in cycle_root_)
    bool ContainsCycle() const;

    // checks if target is reachable from source, the first element of the pair holds
    // the result of this check if return_a_path is set, the std::optional contains a
    // path (list of vertices constituting a path from source to target) should there
    // be one, if (!return_a_path || !reachable) the optional will be empty
    std::pair<bool, std::optional<std::vector<size_t>>> reachable(
      size_t source,
      size_t target,
      bool return_a_path = false
    ) const;

    // debug print
    void PrintVertexRelations(std::ostream& os) const;
    void PrintStepMap(std::ostream& os) const;

    // important that all modifications to the graph_ are flagged with this
    // otherwise paths_info will not update properly
    void MarkModified() { modified_flag = true; }

    // ensure that modifications are tracked to avoid PathsInfo recalculations
    bool modified_flag = true;

  protected:
    // set by ContainsCycle() to be the vertex that is the first root that is found
    mutable size_t cycle_root_ = 0;
    // number of task + 2, index 0 is the source, index vertex_count - 1 is the sink
    size_t vertex_count_;
    // cliques for each machine, indexed by machine id's
    std::vector<std::set<size_t>> cliques_;
    // store Timing information, is tightly bound
    PathsInfo graph_paths_info_;
    // successor and predecessor list combined
    // positive values encodes Job successors, negative values encode predecessors
    // machine relations are encoded by by first adding vertex_count_ to the vertex id
    // 0 can only be a predecessor to a task
    std::vector<std::vector<long>> graph_;
    // only has task precedence edges
    std::vector<std::vector<long>> graph_only_task_pred_;
    // maps vertex_id's to tasks, index 0 is the source, the last entry is the sink
    // source is (UINT_MAX,0), sink is (0,UINT_MAX)
    // source and sink have no task associated with them
    std::vector<Identifier> task_map_;
    // map vertex id's to durations of the corresponding task
    std::vector<unsigned int> duration_map_;

    // helper functions
    const Job::Task& getTaskFromVertex(size_t vertex);
    // static helper functions
    static size_t getDirectElevatedPredecessor(size_t vertex, const std::vector<std::vector<long>>& graph);
    // true for successors, changes the value if it is elevated to be a valid index
    static bool filterForSuccessors(long& vertex, const std::vector<std::vector<long>>& graph);
    // true for predecessors, changes the value if it is elevated to be a valid index
    static bool filterForPredecessors(long& vertex, const std::vector<std::vector<long>>& graph);

    static void addPredecessorsToSet(
      size_t vertex,
      std::set<size_t>& set,
      const std::vector<std::vector<long>>& graph
    );
    static void addSuccessorsToSet(
      size_t vertex,
      std::set<size_t>& set,
      const std::vector<std::vector<long>>& graph
    );
    static bool checkAllPredecessorsInSet(
      size_t vertex,
      const std::set<size_t>& set,
      const std::vector<std::vector<long>>& graph
    );
    static bool checkAllSuccessorsInSet(
      size_t vertex,
      const std::set<size_t>& set,
      const std::vector<std::vector<long>>& graph
    );
    static bool checkIfAPredecessorInSet(
      size_t vertex,
      const std::set<size_t>& set,
      const std::vector<std::vector<long>>& graph
    );
    static bool checkIfASuccessorInSet(
      size_t vertex,
      const std::set<size_t>& set,
      const std::vector<std::vector<long>>& graph
    );
    // check if there is any overlap between the two sets
    static bool unionIsEmpty(const std::set<size_t>& one, const std::set<size_t>& two);


    class SolutionConstructor : public Solution
    {
    public:
      // construct a generic Solution from the internal representation
      SolutionConstructor(
        const std::vector<std::vector<long>>& graph,
        const std::vector<Identifier>& map,
        const Problem* const problem,
        const std::string& prefix
      );
    }; // SolutionConstructor


  private:
    // reachable check using only successor edges, gives path, empty if not reachable
    bool ReachableIntern(
      size_t source, size_t target,
      bool give_path,
      std::vector<size_t>& return_path
    ) const;
    // used in constructor
    void InitialzeGraphAndState();

  };

}

#endif // OPTIMIZER_BASE_GRAPHREP_H_