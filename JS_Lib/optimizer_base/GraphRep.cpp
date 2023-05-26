#include "GraphRep.h"

#include <limits.h>
#include <concepts>
#include <set>
#include <iostream>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  template<typename T>
    requires std::integral<T>
  void addSuccessorsToSet(T vertex, const std::vector<std::vector<long>>& graph, std::set<T>& set) {
    long vertex_count = static_cast<long>(graph.size());

    for (long edge : graph[vertex]) {
      if (edge > 0) {
        if (edge > vertex_count)
          set.insert(static_cast<T>(edge - vertex_count));
        else
          set.insert(static_cast<T>(edge));
      }
    }
  }


  /*////////////////////
     Member Functions
  ////////////////////*/


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
    : Optimizer(problem, criteria)
  {
    unsigned int mCnt = problem->getMachineCount();
    unsigned int tCnt = problem->getTaskCount();
    makespan_ = -1;

    cliques_ = std::vector<MachineClique>();
    cliques_.reserve(mCnt);

    vertex_count_ = 0;
    const auto& machine_step_cnts = problem->getStepCountForMachines();
    for (unsigned int i = 0; i < mCnt; ++i) {
      cliques_.emplace_back(MachineClique(i, tCnt));
      vertex_count_ += machine_step_cnts[i];
    }

    map_to_steps_ = std::vector<Identifier>();
    map_to_steps_.reserve(vertex_count_ + 2);
    map_to_steps_.emplace_back(Identifier(UINT_MAX, 0)); // this entry is invalid, corresponds to source
    vertex_count_ += 2; // add sink and source to total
    graph_ = std::vector<std::vector<long>>();
    graph_.reserve(vertex_count_);
    graph_.emplace_back(std::vector<long>()); // add source vertex list

    size_t vertex_id = 1;
    auto endVerticies = std::vector<size_t>();

    for (const Task& task : problem->getTasks()) {
      const auto& steps = task.getSteps();
      size_t stepCnt = task.size() - 1;
      unsigned int tid = task.getId();
      for (const Task::Step& step : steps) {
        map_to_steps_.emplace_back(Identifier(tid, step.index));
        cliques_[step.machine].machine_order_.push_back(tid);
        cliques_[step.machine].vertex_map_[tid].push_back(vertex_id);
        graph_.emplace_back(std::vector<long>());
        // set successor and predecessor for task precedence in the graph
        if (step.index == 0) {
          graph_[0].push_back(static_cast<long>(vertex_id));
          graph_[vertex_id].push_back(0);
        }
        else {
          if (step.index == stepCnt) {
            // later link to sink, once the vector for it has been created
            endVerticies.push_back(vertex_id);
          }
          graph_[vertex_id].push_back(-static_cast<long>(vertex_id - 1));
          graph_[vertex_id - 1].push_back(static_cast<long>(vertex_id));
        }

        ++vertex_id;
      }
    }

    graph_.emplace_back(std::vector<long>()); // add sink vertex
    map_to_steps_.emplace_back(Identifier(0, UINT_MAX)); // this entry is invalid, corresponds to sink
    // set predecessor and successor lists for sink
    for (size_t v : endVerticies) {
      graph_[v].push_back(static_cast<long>(vertex_id));
      graph_[vertex_id].push_back(-static_cast<long>(v));
    }
    ++vertex_id;

    graph_only_task_pred_ = graph_;
  }


  void GraphRep::applyCliqueOrdersToGraph()
  {
    unsigned int taskCnt = problem_pointer_->getTaskCount();
    
    // reset graph_
    graph_ = graph_only_task_pred_;
    
    // add edges for the machine order set by the clique
    for (MachineClique& clique : cliques_)
    {
      size_t prev_vertex = 0;
      auto clique_indices = std::vector<unsigned int>(taskCnt, 0);
      for (unsigned int tid : clique.machine_order_)
      {
        size_t next_vertex = clique.vertex_map_[tid][clique_indices[tid]];
        ++clique_indices[tid];
        if (prev_vertex == 0) {
          prev_vertex = next_vertex;
          continue;
        }
        // add new edges in the machine precedence range
        graph_[prev_vertex].push_back(static_cast<long>(vertex_count_ + next_vertex));
        graph_[next_vertex].push_back(-static_cast<long>(vertex_count_ + prev_vertex));

        prev_vertex = next_vertex;
      }
    }
    
  }

  void GraphRep::calculateCurrentPaths()
  {


  }

  void GraphRep::debugPrintGraph() {
    long vertex_count = static_cast<long>(vertex_count_);

    std::cout << "Graph is:\n";
    for (int i = 0; i < vertex_count; ++i) {
      auto& list = graph_[i];
      GraphRep::Identifier& baseVert = map_to_steps_[i];
      std::cout << "(" << baseVert.taskId << ", " << baseVert.index << ") has successors: ";
      for (long edge : list) {
        if (edge < 1)
          continue;
        if (edge > vertex_count) {
          GraphRep::Identifier& vert = map_to_steps_[edge - vertex_count];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
        }
        else {
          GraphRep::Identifier& vert = map_to_steps_[edge];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
        }
      }
      std::cout << "\n";
      std::cout << "(" << baseVert.taskId << ", " << baseVert.index << ") has predecessors: ";
      for (long edge : list) {
        if (edge > 0)
          continue;
        if (edge < -vertex_count) {
          GraphRep::Identifier& vert = map_to_steps_[-(edge + vertex_count)];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
        }
        else {
          GraphRep::Identifier& vert = map_to_steps_[-edge];
          std::cout << "(" << vert.taskId << ", " << vert.index << "), ";
        }
      }
      std::cout << "\n";
    }

  }


  GraphRep::SolutionConstructor::SolutionConstructor(const std::vector<std::vector<long>>& graph,
                                                      const std::vector<Identifier>& map,
                                                        const Problem* const problem,
                                                          const std::string& prefix)
  {
    Solution::task_count_ = problem->getTaskCount();
    Solution::machine_count_ = problem->getMachineCount();
    Solution::name_ = prefix + problem->getName();
    Solution::initalized_ = true;
    Solution::makespan_ = 0;

    // setup solution matrix, contains uninitalized Steps
    Solution::solution_ = std::vector<std::vector<Solution::Step>>(machine_count_);
    const auto& machineStepCnt = problem->getStepCountForMachines();
    for (unsigned int i = 0; i < machine_count_; ++i) {
      solution_[i] = std::vector<Solution::Step>(machineStepCnt[i]);
    }

    // track task lengths for problemView
    auto task_lengths = std::vector<unsigned int>(Solution::task_count_, 0);
    // prepare variables to track progress while cascading the state
    auto currMachineIndex = std::vector<size_t>(machine_count_, 0);
    auto scheduled = std::set<size_t>();
    auto reachable = std::set<size_t>();
    auto reachableClearBuf = std::vector<size_t>();
    size_t prevSize = 17; // set != 0 to enter while loop
    long vertex_count = static_cast<long>(graph.size());
    // setup initial reachable and scheduled
    scheduled.insert(0);
    addSuccessorsToSet<size_t>(0, graph, reachable);
    // cascade
    // invariant: (reachable union scheduled) = empty set
    while (scheduled.size() != prevSize)
    {
      prevSize = scheduled.size();
      reachableClearBuf.clear();
      for (size_t reachable_vertex : reachable)
      {
        if (reachable_vertex == static_cast<size_t>(vertex_count - 1)) {
          reachableClearBuf.push_back(reachable_vertex);
          continue;
        }
        bool scheduable = true;
        // check all edges this vertex has in his list
        for (long linked_vertex : graph[reachable_vertex]) {
          // predecessors are negative, or 0 for the source (don't care about source)
          if (linked_vertex < 0)
          {
            long pred = 0;
            if (linked_vertex < -vertex_count) {
              pred = -(linked_vertex + vertex_count);
            }
            else
              pred = -linked_vertex;
            // if a predecessor is not scheduled, this vertex cannot be scheduled
            if (!scheduled.contains(static_cast<size_t>(pred)))
              scheduable = false;
          }
        }

        if (scheduable)
        {
          // update state
          scheduled.insert(reachable_vertex);
          addSuccessorsToSet<size_t>(reachable_vertex, graph, reachable);
          reachableClearBuf.push_back(reachable_vertex);
          // schedule it in the solution
          const GraphRep::Identifier& ident = map[reachable_vertex];
          const Task::Step& step = problem->getTasks()[ident.taskId].getSteps()[ident.index];
          solution_[step.machine][currMachineIndex[step.machine]] = Solution::Step(step.task_id, step.index, step.machine, -1, -1);
          ++task_lengths[step.task_id];
          ++currMachineIndex[step.machine];
        }
      }
      for (size_t to_clear : reachableClearBuf) {
        reachable.erase(to_clear);
      }

    } // while

    Solution::calculateTimings(*problem);

    // init the problemRep vectors to correct size (filling happens during first validate call)
    Solution::problem_view_ = std::vector<std::vector<Solution::Step*>>(Solution::task_count_);
    for (unsigned int i = 0; i < Solution::task_count_; ++i) {
      Solution::problem_view_[i] = std::vector<Solution::Step*>(task_lengths[i], nullptr);
    }

  }


}