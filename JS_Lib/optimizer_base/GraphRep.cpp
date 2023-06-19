#include "GraphRep.h"

#include <iostream>
#include <limits.h>
#include <stack>
#include <queue>
#include <algorithm>

#include "loguru.hpp"


namespace JSOptimizer {


  GraphRep::GraphRep(Problem* problem, const TerminationCriteria& criteria,
                     std::string prefix, unsigned int seed)
    : Optimizer(problem, criteria, prefix, seed), graph_paths_info_(PathsInfo(this))
  {
    unsigned int mCnt = problem->getMachineCount();
    unsigned int tCnt = problem->getTaskCount();
    // prepare cliques
    cliques_ = std::vector<std::set<size_t>>(mCnt);
    cliques_.reserve(mCnt);
    // precompute vertex_count to allocate members
    vertex_count_ = 0;
    const auto& machine_step_cnts = problem->getStepCountForMachines();
    for (unsigned int i = 0; i < mCnt; ++i) {
      cliques_.emplace_back(std::set<size_t>());
      vertex_count_ += machine_step_cnts[i];
    }
    vertex_count_ += 2; // add sink and source to total
    // allocate remaining members
    step_map_ = std::vector<Identifier>();
    step_map_.reserve(vertex_count_);
    duration_map_ = std::vector<unsigned int>();
    duration_map_.reserve(vertex_count_);
    graph_ = std::vector<std::vector<long>>();
    graph_.reserve(vertex_count_);
    
    initialzeGraphAndState();

    // copy graph
    graph_only_task_pred_ = graph_;
  }


  bool GraphRep::containsCycle() const {
    // 0: white, 1: grey, 2: black
    std::vector<char> status(vertex_count_, 0);
    auto stack = std::stack<size_t>();
    stack.push(0);
    // iterative DFS to find back edges in the graph
    while (!stack.empty()) {
      size_t t = stack.top();
      if (status[t] == 0) {
        status[t] = 1;
        for (long vertex : graph_[t])
        {
          if (GraphRep::filterForSuccessors(vertex, graph_)) {
            if (status[vertex] == 1) {
              DLOG_F(INFO, "Found Cycle that includes vertex %i", static_cast<int>(vertex));
              cycle_root_ = vertex;
              return true;
            }
            if (status[vertex] == 0) {
              stack.push(vertex);
            }
          }
        }
      }
      else {
        status[t] = 2;
        stack.pop();
      }
    }
    return false;
  }


  std::pair<bool, std::optional<std::vector<size_t>>> GraphRep::reachable(
      size_t source, size_t target,
      bool return_a_path) const
  {
    auto path = std::vector<size_t>();

    bool reachable = reachable_intern(source, target, return_a_path, path);
    // if we dont want the path or there is none
    if (!return_a_path || !reachable) {
      return { reachable, std::nullopt };
    }
    else {
      // want the path and there is one
      std::optional<std::vector<size_t>> opt(path);
      return { true, opt };
    }
  }


  /*////////////////
      Paths Info
  ////////////////*/

  void GraphRep::PathsInfo::updateTimings()
  {
    if (!parent_->modified_flag) // prevent recomputation
      return;
    // update flag as timings will now match the state
    parent_->modified_flag = false;
    size_t vertex_count = parent_->vertex_count_;
    // reset state
    cp_length_ = 0;
    timings_ = std::vector<Timing>(vertex_count);
    // do Critical Path Method (CPM)
    doCPMForwardPass();
    Timing& sink = timings_[vertex_count - 1];
    // set cp_length_ i.e. the makespan is EFD of the sink
    cp_length_ = sink.EFD;
    // finalize sink timings
    sink.LSD = sink.EFD;
    sink.LFD = sink.EFD;
    sink.FF = cp_length_ - sink.EFD;
    sink.TF = sink.FF;
    doCPMBackwardPass();
  }


  void GraphRep::PathsInfo::doCPMForwardPass()
  {
    const auto& graph = parent_->graph_;
    const auto& duration_map = parent_->duration_map_;
    // setup state variables
    auto calculated = std::set<size_t>();
    auto reachable = std::set<size_t>();
    size_t prevSize = 17; // set != 0 to enter while loop
    // CPM forward pass
    calculated.insert(0); // add source
    addSuccessorsToSet(0, reachable, graph);
    while (calculated.size() != prevSize)
    {
      prevSize = calculated.size();
      // increment inside the loop to allow for deletion of elements from the set
      for (auto current = reachable.begin(); current != reachable.end();)
      {
        // check if timing can be determined for reachable
        if (checkAllPredecessorsInSet(*current, calculated, graph))
        {
          // update state
          calculated.insert(*current);
          addSuccessorsToSet(*current, reachable, graph);
          // calculate timings
          auto predecessors = std::set<size_t>();
          addPredecessorsToSet(*current, predecessors, graph);
          unsigned int ESD = 0;
          for (size_t pred : predecessors) {
            if (timings_[pred].EFD > ESD)
              ESD = timings_[pred].EFD;
          }
          // set timing
          timings_[*current].ESD = ESD;
          timings_[*current].EFD = ESD + duration_map[*current];
          // postfix passes the old position to erase, but first jumps to a newer one
          // old iterator gets invalidated, current remains valid, otherwise removal
          // of set element is not possible inside a foreach loop over the set
          reachable.erase(current++); // increment loop
        }
        else {
          ++current; // increment loop
        }
      }
    }
    DCHECK_F(reachable.size() == 0);
  }


  void GraphRep::PathsInfo::doCPMBackwardPass()
  {
    const auto& graph = parent_->graph_;
    const auto& duration_map = parent_->duration_map_;
    // setup state variables
    auto calculated = std::set<size_t>();
    auto reachable = std::set<size_t>();
    size_t prevSize = 17; // set != 0 to enter while loop
    // CPM backwards pass
    size_t vertex_count = parent_->vertex_count_;
    calculated.insert(vertex_count - 1); // add sink
    addPredecessorsToSet(vertex_count - 1, reachable, graph);
    while (calculated.size() != prevSize)
    {
      prevSize = calculated.size();
      // increment inside the loop to allow for deletion of elements from the set
      for (auto current = reachable.begin(); current != reachable.end();)
      {
        // check if timing can be determined for reachable
        if (checkAllSuccessorsInSet(*current, calculated, graph))
        {
          // update state
          calculated.insert(*current);
          addPredecessorsToSet(*current, reachable, graph);
          // calculate timings
          auto successors = std::set<size_t>();
          addSuccessorsToSet(*current, successors, graph);
          unsigned int LFD = UINT_MAX;
          unsigned int mESD = 0;
          unsigned int mLSD = 0;
          for (size_t succ : successors) {
            Timing& t_succ = timings_[succ];
            if (t_succ.LSD < LFD) {
              LFD = t_succ.LSD;
              mESD = t_succ.ESD;
              mLSD = t_succ.LSD;
            }
          }
          // set timing
          Timing& ct = timings_[*current];
          ct.LFD = LFD;
          ct.LSD = LFD - duration_map[*current];
          // calculate floats
          ct.FF = mESD - ct.EFD;
          ct.TF = mLSD - ct.EFD;
          // increment loop and remove from rachable, see doCPMForwardPass()
          // for more details on why it has to be like this
          reachable.erase(current++);
        }
        else {
          ++current; // increment loop
        }
      }
    }
    DCHECK_F(reachable.size() == 0);
  }


  void GraphRep::PathsInfo::updateCriticalPath()
  {
    if (parent_->modified_flag) // update if not current
    {
      updateTimings();
    }
    size_t vertex_count = parent_->vertex_count_;
    const auto& graph = parent_->graph_;
    if (!critical_path_.empty())
      critical_path_.clear();
    // determine critical path based on Timing info
    // critical activities: ESD = LSD and EFD = LFD
    // traverse the graph with BFS to get a critical path while restricting
    // successors to direct succesors i.e. take longest path
    auto parent_map = std::vector<size_t>(vertex_count, 0);
    auto queue = std::queue<size_t>();
    auto completed = std::set<size_t>();
    auto visited = std::vector<bool>(vertex_count, false);
    // discard all non-critical steps by marking them visited
    for (size_t v = 0; v < vertex_count; ++v) {
      const Timing& t = timings_[v];
      if (!(t.ESD == t.LSD && t.EFD == t.LFD)) {
        completed.insert(v);
        visited[v] = true;
      }
    }
    // do BFS, completed set restricts successors to direct ones
    queue.push(0);
    visited[0] = true;
    while (!queue.empty())
    {
      size_t current = queue.front();
      queue.pop();
      completed.insert(current);
      if (current == vertex_count - 1)
        break;
      for (long vertex : graph[current])
      {
        if (GraphRep::filterForSuccessors(vertex, parent_->graph_)) {
          if (!visited[vertex]
              && checkAllPredecessorsInSet(vertex, completed, parent_->graph_)) {
            visited[vertex] = true;
            parent_map[vertex] = current;
            queue.push(vertex);
          }
        }
      }
    }
    // reconstruct the path
    size_t current_position = vertex_count - 1;
    while (current_position != 0) {
      critical_path_.push_back(current_position);
      current_position = parent_map[current_position];
    }
    critical_path_.push_back(0);
    // path is in reverse, correct
    std::reverse(critical_path_.begin(), critical_path_.end());
  }


  /*//////////////////
      Dac Extender
  //////////////////*/

  GraphRep::DacExtender::DacExtender(const std::vector<std::vector<long>>& graph)
  {
    size_t vertex_count = graph.size();
    node_vertex_map_ = std::vector<Node*>(vertex_count, nullptr);
    successor_map_ = std::vector<std::vector<size_t>>(vertex_count);
    for (size_t i = 0; i < vertex_count; ++i) {
      successor_map_[i] = std::vector<size_t>();
    }
    source_ = new Node();
    source_->vertices.insert(0);
    node_vertex_map_[0] = source_;

    // create toposort based on all edges in the graph
    auto succ_set = std::set<size_t>();
    auto placed = std::set<size_t>();

    placed.insert(0);
    addSuccessorsToSet(0, succ_set, graph);

    Node* current = source_;
    bool advanced_this_iteration = false; // important init
    bool placed_vertex = true; // important init
    // iterations are based on the current Node, which advances once per iteration
    while (placed.size() != vertex_count) {
      if (!placed_vertex) {
        advanced_this_iteration = false;
        current->next_ptr->prev_ptr = current;
        current = current->next_ptr;
        if (current == nullptr) {
          DLOG_F(WARNING, "current was nullptr in DAC extender creation");
        }
      }
      placed_vertex = false;
      for (auto it = succ_set.begin(); it != succ_set.end();)
      {
        if (checkAllPredecessorsInSet(*it, placed, graph)) {
          auto tmp = std::set<size_t>();
          addPredecessorsToSet(*it, tmp, graph);
          // check if conflict with current
          if (unionIsEmpty(current->vertices, tmp))
          {
            // if some predecessors are placed in current->next_ptr, cannot schedule yet
            if (advanced_this_iteration && !unionIsEmpty(current->next_ptr->vertices, tmp)) {
              ++it;
              continue;
            }
            // place the vertex in toposort
            current->vertices.insert(*it);
            node_vertex_map_[*it] = current;
          }
          else {
            // add new node only once per iteration
            if (!advanced_this_iteration)
            {
              if (current->next_ptr == nullptr) {
                current->next_ptr = new Node(current, nullptr, current->position + 1);
              }
              else {
                incrementPositionOfAllSuccessors(current);
                Node* next_next = current->next_ptr;
                current->next_ptr = new Node(current, next_next, current->position + 1);
                next_next->prev_ptr = current->next_ptr;
              }
              advanced_this_iteration = true;
            }
            else if (!unionIsEmpty(current->next_ptr->vertices, tmp)) {
              ++it;
              continue;
            }
            // place the vertex in toposort
            current->next_ptr->vertices.insert(*it);
            node_vertex_map_[*it] = current->next_ptr;
          }
          placed_vertex = true;
          placed.insert(*it);
          addSuccessorsToSet(*it, succ_set, graph);
          succ_set.erase(it++); // delete and increment
        }
        else {
          ++it;
        }
      }
    }
    // fill successor_map_
    for (size_t v = 0; v < vertex_count; ++v)
    {
      unsigned int succ_node_pos = node_vertex_map_[vertex_count - 1]->position + 1;
      for (long vert : graph[v])
      {
        if (GraphRep::filterForSuccessors(vert, graph)){
          successor_map_[v].push_back(vert);
          if (node_vertex_map_[vert]->position < succ_node_pos) {
            succ_node_pos = node_vertex_map_[vert]->position;
            std::swap(successor_map_[v].front(), successor_map_[v].back());
          }
        }
      }
    }
  }


  GraphRep::DacExtender::DacExtender(const DacExtender& other)
    : successor_map_(other.successor_map_)
  {
    node_vertex_map_ = std::vector<Node*>(other.node_vertex_map_.size());
    Node* other_curr = other.source_;
    Node* this_curr = new Node(*other_curr);
    source_ = this_curr;
    while (other_curr->next_ptr != nullptr) {
      other_curr = other_curr->next_ptr;
      this_curr->next_ptr = new Node(*other_curr);
      this_curr->next_ptr->prev_ptr = this_curr;
      this_curr = this_curr->next_ptr;
    }
    unsigned int index = 0;
    for (Node* n : other.node_vertex_map_) {
      unsigned int pos = n->position;
      this_curr = source_;
      while (this_curr->position != pos)
        this_curr = this_curr->next_ptr;
      node_vertex_map_[index] = this_curr;
      ++index;
    }
  }


  GraphRep::DacExtender& GraphRep::DacExtender::operator=(const DacExtender& other)
  {
    if (this != &other) {
      // create temporary copy
      DacExtender temp(other);
      // swap contents of this with temporary copy
      std::swap(source_, temp.source_);
      std::swap(node_vertex_map_, temp.node_vertex_map_);
      std::swap(successor_map_, temp.successor_map_);
    }
    return *this;
  }


  GraphRep::DacExtender::~DacExtender()
  {
    Node* current = source_;
    while (current != nullptr) {
      Node* tmp = current;
      current = current->next_ptr;
      delete tmp;
    }
  }


  std::pair<size_t, size_t> GraphRep::DacExtender::insertEdge(size_t vertex1, size_t vertex2)
  {
    Node* left = node_vertex_map_[vertex1];
    Node* right = node_vertex_map_[vertex2];
    if (left->position != right->position) {
      // orient the edge such that it follows the topo sort
      if (left->position > right->position) {
        std::swap(vertex1, vertex2);
      }
      // update successor map
      successor_map_[vertex1].push_back(vertex2);
      right = node_vertex_map_[vertex2];
      unsigned int left_succ_pos = node_vertex_map_[successor_map_[vertex1][0]]->position;
      if (right->position < left_succ_pos) {
        std::swap(successor_map_[vertex1].front(), successor_map_[vertex1].back());
      }
    }
    else {
      // had no relation until now, need to move one vertex to a different node
      Node* base = left;
      auto& v1_s_vec = successor_map_[vertex1];
      auto& v2_s_vec = successor_map_[vertex2];
      if (v1_s_vec.empty() || node_vertex_map_[v1_s_vec[0]] != base->next_ptr) {
        // move vertex1 into next node
        base->vertices.erase(vertex1);
        base->next_ptr->vertices.insert(vertex1);
        node_vertex_map_[vertex1] = base->next_ptr;
        maintainInvarinatSuccessorMapMovedVertex(vertex1);
        successor_map_[vertex2].push_back(vertex1);
        maintainInvarinatSuccessorMapAddedSuccessor(vertex2);
        std::swap(vertex1, vertex2); // set return values correctly
      }
      else if (v2_s_vec.empty() || node_vertex_map_[v2_s_vec[0]] != base->next_ptr) {
        // move vertex2 into next node
        base->vertices.erase(vertex2);
        base->next_ptr->vertices.insert(vertex2);
        node_vertex_map_[vertex2] = base->next_ptr;
        maintainInvarinatSuccessorMapMovedVertex(vertex2);
        successor_map_[vertex1].push_back(vertex2);
        maintainInvarinatSuccessorMapAddedSuccessor(vertex1);
      }
      else {
        // need to create new node because neither vertex can be moved backwards
        // use edge (vertex1, vertex2) to tie break
        incrementPositionOfAllSuccessors(base);
        Node* next_next = base->next_ptr;
        base->next_ptr = new Node(base, next_next, base->position + 1);
        base->vertices.erase(vertex2);
        base->next_ptr->vertices.insert(vertex2);
        node_vertex_map_[vertex2] = base->next_ptr;
        successor_map_[vertex1].push_back(vertex2);
        maintainInvarinatSuccessorMapMovedVertex(vertex2, true);
      }
    }
    return { vertex1, vertex2 };
  }


  void GraphRep::DacExtender::incrementPositionOfAllSuccessors(Node* start)
  {
    Node* current = start->next_ptr;
    while (current != nullptr) {
      ++current->position;
      current = current->next_ptr;
    }
  }


  void GraphRep::DacExtender::maintainInvarinatSuccessorMapMovedVertex(size_t modified, bool check_all)
  {
    for (size_t v = 0; v < successor_map_.size() - 1; ++v)
    {
      if (check_all || successor_map_[v][0] == modified) {
        unsigned int min_pos = node_vertex_map_[successor_map_[v][0]]->position;
        for (auto vert = successor_map_[v].begin(); vert != successor_map_[v].end(); ++vert)
        {
          if (node_vertex_map_[*vert]->position < min_pos) {
            min_pos = node_vertex_map_[*vert]->position;
            std::swap(*vert, successor_map_[v][0]);
          }
        }
      }
    }
  }


  void GraphRep::DacExtender::maintainInvarinatSuccessorMapAddedSuccessor(size_t modified)
  {
    auto& modified_map = successor_map_[modified];
    unsigned int min_pos = node_vertex_map_[modified_map[0]]->position;
    for (auto vert = modified_map.begin(); vert != modified_map.end(); ++vert)
    {
      if (node_vertex_map_[*vert]->position < min_pos) {
        min_pos = node_vertex_map_[*vert]->position;
        std::swap(*vert, modified_map[0]);
      }
    }
  }


  void GraphRep::DacExtender::debugVerifyIntegrity(const std::vector<std::vector<long>>& graph)
  {
    DCHECK_F(node_vertex_map_[0] == source_, "source does not match");
    //DCHECK_F(node_vertex_map_.back()->vertices.contains(node_vertex_map_.size()), "sink not contained in last node");
    Node* previous = source_;
    Node* current = source_->next_ptr;

    std::cout << "Node " << previous->position << ": ";
    for (size_t vert : previous->vertices) {
      std::cout << vert << ", ";
    }
    std::cout << "\n";

    while (current != nullptr) {
      DCHECK_F(current->position == previous->position + 1, "position index invalid");
      std::cout << "Node " << current->position << ": ";
      for (size_t vert : current->vertices) {
        std::cout << vert << ", ";
      }
      std::cout << "\n";
      current = current->next_ptr;
      previous = previous->next_ptr;
    }
  }


  /*//////////////////////////
      Solution Constructor
  //////////////////////////*/

  GraphRep::SolutionConstructor::SolutionConstructor(const std::vector<std::vector<long>>& graph,
                                                     const std::vector<Identifier>& map,
                                                     const Problem* const problem,
                                                     const std::string& prefix) {
    Solution::task_count_ = problem->getTaskCount();
    Solution::machine_count_ = problem->getMachineCount();
    Solution::name_ = prefix + problem->getName();
    Solution::initalized_ = true;
    Solution::makespan_ = 0;

    // setup solution matrix, contains uninitalized Steps
    Solution::solution_ = std::vector<std::vector<Solution::Step>>(machine_count_);
    const auto& machine_step_counts = problem->getStepCountForMachines();
    for (unsigned int i = 0; i < machine_count_; ++i) {
      solution_[i] = std::vector<Solution::Step>(machine_step_counts[i]);
    }

    // track task lengths for problemView
    auto task_lengths = std::vector<unsigned int>(Solution::task_count_, 0);
    // prepare variables to track progress while cascading the state
    bool progress = true;
    long vertex_count = static_cast<long>(graph.size());
    auto currMachineIndex = std::vector<size_t>(machine_count_, 0);
    auto scheduled = std::set<size_t>();
    auto reachable = std::set<size_t>();
    // setup initial reachable and scheduled
    scheduled.insert(0);
    addSuccessorsToSet(0, reachable, graph);
    // cascade
    // invariant: (reachable set_union scheduled) = empty set
    while (scheduled.size() != vertex_count - 1)
    {
      if (!progress)
        break;
      progress = false;
      for (auto current = reachable.begin(); current != reachable.end();)
      {
        // discard the sink
        if (*current == static_cast<size_t>(vertex_count - 1)) {
          reachable.erase(current++); // erase and increment loop
          continue;
        }
        if (GraphRep::checkAllPredecessorsInSet(*current, scheduled, graph))
        {
          // update state
          scheduled.insert(*current);
          addSuccessorsToSet(*current, reachable, graph);
          // schedule it in the solution
          const GraphRep::Identifier& ident = map[*current];
          const Task::Step& step = problem->getTasks()[ident.task_id].getSteps()[ident.index];
          solution_[step.machine][currMachineIndex[step.machine]] = Solution::Step(step.task_id, step.index, step.machine, -1, -1);
          ++task_lengths[step.task_id];
          ++currMachineIndex[step.machine];
          reachable.erase(current++); // erase and increment loop (as in CPMForwardPass())
          progress = true;
        }
        else {
          ++current; // increment loop
        }
      } // for
    } // while
    if (scheduled.size() != vertex_count - 1) {
      throw std::runtime_error("GraphRep::SolutionConstructor(): failed to complete");
    }
    
    Solution::calculateTimings(*problem);

    // init the problemRep vectors to correct size (filling happens during first validate call)
    Solution::problem_view_ = std::vector<std::vector<Solution::Step*>>(Solution::task_count_);
    for (unsigned int i = 0; i < Solution::task_count_; ++i) {
      Solution::problem_view_[i] = std::vector<Solution::Step*>(task_lengths[i], nullptr);
    }

  }


  /*//////////////////////
      Helper Functions
  //////////////////////*/

  // this is just a getter to make things less verbose and easier to follow
  const Task::Step& GraphRep::getStepFromVertex(size_t vertex)
  {
    const Identifier& iden = step_map_.at(vertex);
    return problem_pointer_->getTasks()[iden.task_id].getSteps()[iden.index];
  }

  void GraphRep::addPredecessorsToSet(size_t vertex, std::set<size_t>& set,
                                      const std::vector<std::vector<long>>& graph) {
    long vertex_count = static_cast<long>(graph.size());

    for (long edge : graph[vertex]) {
      if (edge <= 0) {
        if (edge <= -vertex_count)
          set.insert(static_cast<size_t>(-(edge + vertex_count)));
        else
          set.insert(static_cast<size_t>(-edge));
      }
    }
  }

  void GraphRep::addSuccessorsToSet(size_t vertex, std::set<size_t>& set,
                                    const std::vector<std::vector<long>>& graph) {
    long vertex_count = static_cast<long>(graph.size());

    for (long edge : graph[vertex]) {
      if (edge > 0) {
        if (edge > vertex_count)
          set.insert(static_cast<size_t>(edge - vertex_count));
        else
          set.insert(static_cast<size_t>(edge));
      }
    }
  }

  bool GraphRep::checkAllPredecessorsInSet(size_t vertex, const std::set<size_t>& set,
                                           const std::vector<std::vector<long>>& graph) {
    bool schedulable = true;
    long vertex_count = static_cast<long>(graph.size());
    // check all edges this vertex has
    for (long edge : graph[vertex]) {
      // predecessors are negative, or 0 for the source
      if (edge <= 0)
      {
        long pred = 0;
        if (edge <= -vertex_count) {
          pred = -(edge + vertex_count);
        }
        else
          pred = -edge;

        if (!set.contains(static_cast<size_t>(pred))) {
          schedulable = false;
          break;
        }
      }
    }
    return schedulable;
  }

  bool GraphRep::checkAllSuccessorsInSet(size_t vertex, const std::set<size_t>& set,
                                         const std::vector<std::vector<long>>& graph) {
    bool schedulable = true;
    long vertex_count = static_cast<long>(graph.size());
    // check all edges this vertex has
    for (long edge : graph[vertex]) {
      // successors are positive (0 i.e. source can't be successor)
      if (edge > 0)
      {
        long succ = 0;
        if (edge > vertex_count) {
          succ = edge - vertex_count;
        }
        else
          succ = edge;

        if (!set.contains(static_cast<size_t>(succ))) {
          schedulable = false;
          break;
        }
      }
    }
    return schedulable;
  }

  bool GraphRep::checkIfAPredecessorInSet(size_t vertex, const std::set<size_t>& set,
                                          const std::vector<std::vector<long>>& graph) {
    bool found_one = false;
    long vertex_count = static_cast<long>(graph.size());
    // check all edges this vertex has
    for (long edge : graph[vertex]) {
      // predecessors are negative, or 0 for the source
      if (edge <= 0)
      {
        long pred = 0;
        if (edge <= -vertex_count) {
          pred = -(edge + vertex_count);
        }
        else
          pred = -edge;

        if (set.contains(static_cast<size_t>(pred))) {
          found_one = true;
          break;
        }
      }
    }
    return found_one;
  }

  bool GraphRep::checkIfASuccessorInSet(size_t vertex, const std::set<size_t>& set,
                                        const std::vector<std::vector<long>>& graph) {
    bool found_one = false;
    long vertex_count = static_cast<long>(graph.size());
    // check all edges this vertex has
    for (long edge : graph[vertex]) {
      // successors are positive (0 i.e. source can't be successor)
      if (edge > 0)
      {
        long succ = 0;
        if (edge > vertex_count) {
          succ = edge - vertex_count;
        }
        else
          succ = edge;

        if (set.contains(static_cast<size_t>(succ))) {
          found_one = true;
          break;
        }
      }
    }
    return found_one;
  }

  bool GraphRep::unionIsEmpty(const std::set<size_t>& one, const std::set<size_t>& two)
  {
    if (one.empty() || two.empty())
      return true;
    // use sorted property of sets to check if there are elements that match
    auto elemsOne = one.begin();
    auto elemsTwo = two.begin();
    bool end_reached = false;
    while (!end_reached && *elemsOne != *elemsTwo)
    {
      if (*elemsOne < *elemsTwo) {
        ++elemsOne;
      }
      else {
        ++elemsTwo;
      }

      if (elemsOne == one.end() || elemsTwo == two.end()) {
        end_reached = true;
      }
    }
    return end_reached;
  }

  size_t GraphRep::getDirectElevatedPredecessor(size_t vertex, const std::vector<std::vector<long>>& graph)
  {
    if (vertex == graph.size() - 1) {
      LOG_F(WARNING, "getDirectElevatedPredecessor() called with sink as argument!");
      return graph.size() - 1;
    }
    long vertex_count = static_cast<long>(graph.size());

    size_t elevated_succs_cnt = 0;
    size_t elevated_preds_cnt = 0;
    auto elevated_preds = std::set<size_t>();
    for (long edge : graph[vertex]) {
      if (edge > vertex_count) {
        ++elevated_succs_cnt;
      }
      else if (edge < -vertex_count) {
        elevated_preds.insert(-(edge + vertex_count));
        ++elevated_preds_cnt;
      }
    }
    // first vertex on a machine, no direct predecessor exists
    if (elevated_preds.size() == 0)
      return 0;
    // check predecessors until the direct one is found
    for (size_t pred : elevated_preds) {
      // also only elevated edges
      size_t succs_cnt = 0;
      size_t preds_cnt = 0;
      for (long edge : graph[pred]) {
        if (edge > vertex_count)
          ++succs_cnt;
        else if (edge < -vertex_count)
          ++preds_cnt;
      }
      // direct predecessor has one more successor and one less predecessor
      if (preds_cnt + 1 == elevated_preds_cnt
          && succs_cnt - 1 == elevated_succs_cnt) {
        return pred;
      }
    }
    DLOG_F(INFO, "Failed to find direct elevated predecessor");
    return 0;
  }

  bool GraphRep::filterForSuccessors(long& vertex, const std::vector<std::vector<long>>& graph)
  {
    size_t vertex_count = graph.size();
    if (vertex < 1) // don't care about predecessor
      return false;
    if (vertex >= static_cast<long>(vertex_count)) // align values if elevated edge
      vertex -= static_cast<long>(vertex_count);
    DCHECK_F(vertex != 0, "Source cannot be a successor!");
    return true;
  }

  bool GraphRep::filterForPredecessors(long& vertex, const std::vector<std::vector<long>>& graph)
  {
    size_t vertex_count = graph.size();
    if (vertex > 0) // don't care about successors
      return false;
    if (vertex <= -static_cast<long>(vertex_count)) // align values if elevated edge
      vertex += static_cast<long>(vertex_count);
    // remove marker to make valid vertex id
    vertex = -vertex;
    DCHECK_F(vertex != vertex_count - 1, "Sink cannot be a predecessor!");
    return true;
  }


  /*////////////////////////
      Printing Functions
  ////////////////////////*/

  void GraphRep::printStepMap(std::ostream& os) const
  {
    os << "Map from vertex_id's to Step's (tid, index):\n";
    size_t index = 0;
    for (const Identifier& ident : step_map_) {
      os << index << " -> (" << ident.task_id << ", " << ident.index << ")\n";
      ++index;
    }
  }

  void GraphRep::printVertexRelations(std::ostream& os) const
  {
    os << "Relations for Steps in the Graph:\n";
    long vertex_count = static_cast<long>(vertex_count_);

    for (int i = 0; i < vertex_count; ++i) {
      auto& list = graph_[i];
      const GraphRep::Identifier& baseVert = step_map_[i];
      std::cout << "(" << baseVert.task_id << ", " << baseVert.index << ") predecessors: ";
      for (long edge : list) {
        if (edge > 0)
          continue;
        if (edge < -vertex_count) {
          const GraphRep::Identifier& vert = step_map_[-(edge + vertex_count)];
          std::cout << "(" << vert.task_id << ", " << vert.index << "), ";
        }
        else {
          const GraphRep::Identifier& vert = step_map_[-edge];
          std::cout << "(" << vert.task_id << ", " << vert.index << "), ";
        }
      }
      std::cout << "\n";
      std::cout << "(" << baseVert.task_id << ", " << baseVert.index << ") successors: ";
      for (long edge : list) {
        if (edge < 1)
          continue;
        if (edge > vertex_count) {
          const GraphRep::Identifier& vert = step_map_[edge - vertex_count];
          std::cout << "(" << vert.task_id << ", " << vert.index << "), ";
        }
        else {
          const GraphRep::Identifier& vert = step_map_[edge];
          std::cout << "(" << vert.task_id << ", " << vert.index << "), ";
        }
      }
      std::cout << "\n";
    }

  }

  void GraphRep::PathsInfo::Timing::print(std::ostream& os) const
  {
    os << std::to_string(ESD) + " " << std::to_string(EFD) << " " << std::to_string(LSD) << " ";
    os << std::to_string(LFD) << " " << std::to_string(FF) << " " << std::to_string(TF);
  }



  /*///////////////////////
      Private Functions
  ///////////////////////*/

  bool GraphRep::reachable_intern(size_t source, size_t target, bool give_path,
    std::vector<size_t>& return_path) const {
    bool reachable = false;
    // parent_map[v] contains the vertex that v was found with
    auto parent_map = std::vector<size_t>(vertex_count_, 0);
    // iterative DFS (and building the parent_map)
    auto visited = std::vector<bool>(vertex_count_, false);
    auto stack = std::stack<size_t>();
    stack.push(source);
    while (!stack.empty()) {
      size_t t = stack.top();
      stack.pop();
      if (visited[t])
        continue;
      visited[t] = true;
      for (long vertex : graph_[t])
      {
        if (GraphRep::filterForSuccessors(vertex, graph_)) {
          if (vertex == target) {
            parent_map[target] = t;
            reachable = true;
            stack = std::stack<size_t>(); // clear statck to terminate
          }
          if (!visited[vertex]) {
            stack.push(vertex);
            parent_map[vertex] = t;
          }
        }
      }
    }
    if (reachable) {
      if (give_path) {
        return_path.clear();
        // reconstruct the path
        size_t current_position = target;
        while (current_position != source) {
          return_path.push_back(current_position);
          current_position = parent_map[current_position];
        }
        return_path.push_back(source);
        // path currently in reverse
        std::reverse(return_path.begin(), return_path.end());
      }
      return true;
    }
    return false;
  }


  void GraphRep::initialzeGraphAndState() {
    // helper variables
    size_t vertex_id = 1;
    auto endVerticies = std::vector<size_t>(); // last vertices for each task
    // add source vertex
    graph_.emplace_back(std::vector<long>());
    step_map_.emplace_back(Identifier(UINT_MAX, 0));
    duration_map_.push_back(0);
    // setup graph with task precedence edges & init step_map_ cliques, duration_map_
    // iterate over all step's in the problem
    for (const Task& task : problem_pointer_->getTasks())
    {
      unsigned int tid = task.getId();
      for (const Task::Step& step : task.getSteps())
      {
        duration_map_.push_back(step.duration);
        step_map_.emplace_back(Identifier(tid, step.index));
        cliques_[step.machine].insert(vertex_id);
        graph_.emplace_back(std::vector<long>());
        // set successor and predecessor for task precedence in the graph
        if (step.index == 0) {
          graph_[0].push_back(static_cast<long>(vertex_id));
          graph_[vertex_id].push_back(0);
        }
        else {
          if (step.index == task.size() - 1) {
            // later link to sink, once the vector for it has been created
            endVerticies.push_back(vertex_id);
          }
          graph_[vertex_id].push_back(-static_cast<long>(vertex_id - 1));
          graph_[vertex_id - 1].push_back(static_cast<long>(vertex_id));
        }

        ++vertex_id;
      }
    }
    // finalize graph, duration_map_ and step_map_
    // first add sink vertex
    graph_.emplace_back(std::vector<long>());
    step_map_.emplace_back(Identifier(0, UINT_MAX));
    duration_map_.push_back(0);
    // then set predecessor and successor lists for edges to sink
    for (size_t v : endVerticies) {
      graph_[v].push_back(static_cast<long>(vertex_id));
      graph_[vertex_id].push_back(-static_cast<long>(v));
    }
    ++vertex_id;
    DCHECK_F(vertex_count_ == vertex_id, "GraphRep initalization: unexpected mismatch");
  }


}