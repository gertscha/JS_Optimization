#include "GraphRep.h"

#include <limits.h>
#include <iostream>

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
          graph_[0].push_back(vertex_id);
          graph_[vertex_id].push_back(0);
        }
        else {
          if (step.index == stepCnt) {
            // later link to sink, once the vector for it has been created
            endVerticies.push_back(vertex_id);
          }
          graph_[vertex_id].push_back(-static_cast<long>(vertex_id - 1));
          graph_[vertex_id - 1].push_back(vertex_id);
        }

        ++vertex_id;
      }
    }

    graph_.emplace_back(std::vector<long>()); // add sink vertex
    map_to_steps_.emplace_back(Identifier(0, UINT_MAX)); // this entry is invalid, corresponds to sink
    // set predecessor and successor lists for sink
    for (size_t v : endVerticies) {
      graph_[v].push_back(vertex_id);
      graph_[vertex_id].push_back(-static_cast<long>(v));
    }
    ++vertex_id;

    graph_only_task_pred_ = graph_;

    DCHECK_F(vertex_id == vertex_count_, "Step Counts do not match!");
  }


  void GraphRep::applyCliqueOrdersToGraph()
  {
    unsigned int taskCnt = problem_pointer_->getTaskCount();
    
    // reset graph_
    graph_ = graph_only_task_pred_;
    
    // bugs present todo
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
        graph_[prev_vertex].push_back(vertex_count_ + next_vertex);
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
    for (unsigned int i = 0; i < vertex_count; ++i) {
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


  GraphRep::SolutionConstructor::SolutionConstructor(const std::vector<std::vector<long>>& graph, const std::vector<Identifier>& map,
                                                      const Problem* const problem, const std::string& prefix)
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
      solution_.emplace_back(std::vector<Solution::Step>(machineStepCnt[i]));
    }

    // this is bs, need to figure out order of the steps from graph first

    auto currMachineIndex = std::vector<size_t>(task_count_, 0);
    for (const GraphRep::Identifier& ident : map) {
      const Task::Step& step = problem->getTasks()[ident.taskId].getSteps()[ident.index];
      solution_[step.machine][currMachineIndex[step.machine]] = Solution::Step(step.task_id, step.index, step.machine);
      ++currMachineIndex[step.machine];
    }

    // once order is defined, cascade the timings like in global order rep


    // setup problem_view sizes
    // Solution::problem_view_
    

    // Solution::FillProblemView();
  }


}