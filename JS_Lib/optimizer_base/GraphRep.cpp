#include "GraphRep.h"

#include <limits.h>

#include <iostream>
#include <stack>
#include <algorithm>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  GraphRep::MachineClique::MachineClique(unsigned int machineId, unsigned int taskCnt)
    : machine_(machineId)
  {
    machine_order_ = std::vector<unsigned int>();
    vertex_map_ = std::vector<std::vector<size_t>>(taskCnt);
    for (unsigned int i = 0; i < taskCnt; ++i) {
      vertex_map_[i] = std::vector<size_t>();
    }
  }


  GraphRep::GraphRep(Problem* problem, Optimizer::TerminationCriteria& criteria)
    : Optimizer(problem, criteria), graph_paths_info_(PathsInfo(this)), modified_flag(true)
  {
    unsigned int mCnt = problem->getMachineCount();
    unsigned int tCnt = problem->getTaskCount();
    // prepare clique member
    cliques_ = std::vector<MachineClique>();
    cliques_.reserve(mCnt);
    // precompute vertex_count to allocate members
    vertex_count_ = 0;
    const auto& machine_step_cnts = problem->getStepCountForMachines();
    for (unsigned int i = 0; i < mCnt; ++i) {
      cliques_.emplace_back(MachineClique(i, tCnt));
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
        cliques_[step.machine].machine_order_.push_back(tid);
        cliques_[step.machine].vertex_map_[tid].push_back(vertex_id);
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
    DCHECK_F(vertex_count_ != vertex_id, "GraphRep initalization: unexpected mismatch");
  }

  
  void GraphRep::applyCliqueToGraph(const MachineClique& clique)
  {
    markModified();
    auto clique_indices = std::vector<unsigned int>(problem_pointer_->getTaskCount(), 0);
    size_t prev_vertex = 0;
    for (unsigned int tid : clique.machine_order_)
    {
      size_t next_vertex = clique.vertex_map_[tid][clique_indices[tid]];
      ++clique_indices[tid];
      if (prev_vertex == 0) // don't add edge from source to first step
      {
        prev_vertex = next_vertex;
        continue;
      }
      // add new edges in the elevated range
      graph_[prev_vertex].push_back(static_cast<long>(vertex_count_ + next_vertex));
      graph_[next_vertex].push_back(-static_cast<long>(vertex_count_ + prev_vertex));

      prev_vertex = next_vertex;
    }
  }


  void GraphRep::applyAllCliquesToGraph()
  {
    markModified();
    // reset graph_
    graph_ = graph_only_task_pred_;
    
    // add edges for the machine order defined by the clique
    for (MachineClique& clique : cliques_) {
      applyCliqueToGraph(clique);
    }
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
          if (filterForSuccessors(vertex)) {
            if (status[vertex] == 1) {
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


  bool GraphRep::reachable_intern(size_t source, size_t target, bool give_path,
                                  std::vector<size_t>& return_path) const {
    bool reachable = false;
    // parent_map[v] contains the vertex that v was found with
    auto parent_map = std::vector<size_t>();
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
        if (filterForSuccessors(vertex)) {
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
        if (checkPredecessorsInSet(*current, calculated, graph))
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
        if (checkSuccessorsInSet(*current, calculated, graph))
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
    // determine critical path based on Timing info
    // critical activities: ESD = LSD and EFD = LFD
    // traverse the graph with DFS to get a critical path in topological order
    if (!critical_path_.empty())
      critical_path_.clear();
    
    // iterative DFS (and building the parent_map)
    auto parent_map = std::vector<size_t>();
    auto visited = std::vector<bool>(vertex_count, false);
    auto stack = std::stack<size_t>();
    
    stack.push(0);
    while (!stack.empty())
    {
      size_t current = stack.top();
      stack.pop();
      if (visited[current])
        continue;
      visited[current] = true;
      for (long vertex : graph[current])
      {
        if (parent_->filterForSuccessors(vertex)) {
          const Timing& t = timings_[vertex];
          if (t.ESD == t.LSD && t.EFD == t.LFD) {
            if (!visited[vertex]) {
              stack.push(vertex);
              parent_map[vertex] = current;
            }
            if (vertex = static_cast<long>(vertex_count - 1)) {
              stack = std::stack<size_t>(); // clear stack to terminate
            }
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
    long vertex_count = static_cast<long>(graph.size());
    auto currMachineIndex = std::vector<size_t>(machine_count_, 0);
    auto scheduled = std::set<size_t>();
    auto reachable = std::set<size_t>();
    // setup initial reachable and scheduled
    scheduled.insert(0);
    addSuccessorsToSet(0, reachable, graph);
    // cascade
    // invariant: (reachable set_union scheduled) = empty set
    while (scheduled.size() != vertex_count)
    {
      for (auto current = reachable.begin(); current != reachable.end();)
      {
        // discard the sink
        if (*current == static_cast<size_t>(vertex_count - 1)) {
          reachable.erase(current++); // erase and increment loop (as in CPMForwardPass())
          continue;
        }
        if (GraphRep::checkPredecessorsInSet(*current, scheduled, graph))
        {
          // update state
          scheduled.insert(*current);
          addSuccessorsToSet(*current, reachable, graph);
          // schedule it in the solution
          const GraphRep::Identifier& ident = map[*current];
          const Task::Step& step = problem->getTasks()[ident.taskId].getSteps()[ident.index];
          solution_[step.machine][currMachineIndex[step.machine]] = Solution::Step(step.task_id, step.index, step.machine, -1, -1);
          ++task_lengths[step.task_id];
          ++currMachineIndex[step.machine];
          reachable.erase(current++); // erase and increment loop (as in CPMForwardPass())
        }
        else {
          ++current; // increment loop
        }
      } // for
    } // while

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

  bool GraphRep::checkPredecessorsInSet(size_t vertex, const std::set<size_t>& set,
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

  bool GraphRep::checkSuccessorsInSet(size_t vertex, const std::set<size_t>& set,
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

  bool GraphRep::filterForSuccessors(long& vertex) const {
    if (vertex < 1) // don't care about predecessor
      return false;
    if (vertex >= static_cast<long>(vertex_count_)) // align values if elevated edge
      vertex -= static_cast<long>(vertex_count_);
    DCHECK_F(vertex != 0, "Source cannot be a successor!");
    return true;
  }

  bool GraphRep::filterForPredecessors(long& vertex) const {
    if (vertex > 0) // don't care about successors
      return false;
    if (vertex <= -static_cast<long>(vertex_count_)) // align values if elevated edge
      vertex += static_cast<long>(vertex_count_);
    // remove marker to make valid vertex id
    vertex = -vertex;
    DCHECK_F(vertex != vertex_count_ - 1, "Sink cannot be a predecessor!");
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
      os << index << " -> (" << ident.taskId << ", " << ident.index << ")\n";
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
      std::cout << "(" << baseVert.taskId << ", " << baseVert.index << ") predecessors: ";
      for (long edge : list) {
        if (edge > 0)
          continue;
        if (edge < -vertex_count) {
          const GraphRep::Identifier& vert = step_map_[-(edge + vertex_count)];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
        }
        else {
          const GraphRep::Identifier& vert = step_map_[-edge];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
        }
      }
      std::cout << "\n";
      std::cout << "(" << baseVert.taskId << ", " << baseVert.index << ") successors: ";
      for (long edge : list) {
        if (edge < 1)
          continue;
        if (edge > vertex_count) {
          const GraphRep::Identifier& vert = step_map_[edge - vertex_count];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
        }
        else {
          const GraphRep::Identifier& vert = step_map_[edge];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
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

}