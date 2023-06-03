#include "ShiftingBottleneck.h"

#include <set>
#include <iostream>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  ShiftingBottleneck::ShiftingBottleneck(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix)
    : Optimizer(problem, terminationCriteria), GraphRep(problem, terminationCriteria),
      prefix_(namePrefix), seed_(seed), temperature_(1.0), total_iterations_(0)
  {
    generator_ = std::mt19937_64(seed);
    best_solution_ = std::make_shared<Solution>();
  }


  void ShiftingBottleneck::Run()
  {
    ++restart_count_;
    Initialize();
    while (!CheckTermination()) {
      Iterate();
    }
  }


  void ShiftingBottleneck::Initialize()
  {
    //reset the graph
    markModified();
    graph_ = graph_only_task_pred_;
    // randomize all the cliques
    for (GraphRep::MachineClique& clique : cliques_) {
      auto& order = clique.getMachineOrder();
      std::shuffle(order.begin(), order.end(), generator_);
    }

    // choose random clique and apply it to the graph
    std::uniform_int_distribution<size_t> ind_dist(0, cliques_.size());
    size_t index = ind_dist(generator_);
    applyCliqueToGraph(cliques_[index]);
    DLOG_F(INFO, "random index was %i", static_cast<int>(index));

    // try to add other cliques, modify them if the schedule is unfeasable
    for (unsigned int i = 0; i < cliques_.size(); ++i) {
      if (i != index) {
        initAddCliqueIncrementally(cliques_[i]);
        if (containsCycle()) {
          break;
        }
      }
    }

    printStepMap(std::cout);
    printVertexRelations(std::cout);

    if (containsCycle()) {
      LOG_F(INFO, "Graph contains Cycles!");
      return;
    }

    auto new_sol = std::make_shared<SolutionConstructor>(graph_, step_map_, problem_pointer_, prefix_);
    if (best_solution_->isInitialized() == false || new_sol->getMakespan() < best_solution_->getMakespan())
      best_solution_ = new_sol;

  }


  void ShiftingBottleneck::Iterate()
  {
    markModified();

  }


  bool ShiftingBottleneck::CheckTermination()
  {
    
    return true;
  }


  void ShiftingBottleneck::initAddCliqueIncrementally(MachineClique& clique)
  {
    markModified();
    const std::vector<std::vector<size_t>>& vertex_map = clique.getVertexMap();
    std::vector<unsigned int>& machine_seq = clique.getMachineOrder();
    const auto& m_tasks = problem_pointer_->getTasks();
    // try to add edges for all other cliques, if it creates a cycle, find
    // cross over of tasks and swap the order to prevent the cycle
    auto clique_vertices = std::set<size_t>();
    auto clique_indices = std::vector<unsigned int>(vertex_map.size(), 0);
    auto parent = std::vector<size_t>(vertex_count_, 0);
    size_t prev_vertex = 0;

    for (unsigned int i = 0; i < machine_seq.size(); ++i) {
      unsigned int tid = machine_seq[i];
      size_t next_vertex = vertex_map[tid][clique_indices[tid]];
      ++clique_indices[tid];

      if (prev_vertex == 0) // don't add edge from source to first step
      {
        prev_vertex = next_vertex;
        continue;
      }

      std::pair<bool, std::optional<std::vector<size_t>>> edge_attempt_res = reachable(next_vertex, prev_vertex, true);

      if (edge_attempt_res.first) {
        // debug
        LOG_F(INFO, "doing swaps for %i and %i", static_cast<int>(prev_vertex), static_cast<int>(next_vertex));
        const Identifier& left_iden = step_map_[prev_vertex];
        const Task::Step& left = m_tasks[left_iden.task_id].getSteps()[left_iden.index];
        const Identifier& right_iden = step_map_[next_vertex];
        const Task::Step& right = m_tasks[right_iden.task_id].getSteps()[right_iden.index];
        if (left.machine != right.machine)
          DLOG_F(WARNING, "initAddCliqueIncrementally, the conflict is not within the clique! (did not think this was possible)");
        // resolve with new rule that just switches the order i.e. first to right then to left
        if (left.task_id == right.task_id)
          DLOG_F(WARNING, "initAddCliqueIncrementally, there is a conflict on the same task! (did not think this was possible)");
        // resolve with new rule that just switches the order i.e. first to right then to left
        // end debug

        // change order, 'parent of previous' should point to 'next vertex'
        // 'next vertex' should point to 'previous vertex',' previous' should become current
        size_t p_parent = parent[prev_vertex];
        // correct successor edge of 'parent of previous'
        for (long& edge : graph_[p_parent]) {
          if (edge == vertex_count_ + prev_vertex) {
            edge = static_cast<long>(vertex_count_ + next_vertex);
            break;
          }
        }
        // add pred/succ edges to next_vertex
        graph_[next_vertex].push_back(static_cast<long>(vertex_count_ + prev_vertex));
        graph_[next_vertex].push_back(-static_cast<long>(vertex_count_ + p_parent));
        parent[next_vertex] = p_parent;
        // correct predecessor edge of prev_vertex
        for (long& edge : graph_[prev_vertex]) {
          if (edge == -static_cast<long>(vertex_count_ + p_parent)) {
            edge = -static_cast<long>(vertex_count_ + next_vertex);
            break;
          }
        }
        parent[prev_vertex] = next_vertex;
        // previous becomes current
        size_t temp = prev_vertex;
        prev_vertex = next_vertex;
        next_vertex = temp;
        if (containsCycle()) {
          LOG_F(INFO, "Graph contains Cycles after swaps");
        }
      }
      else {
        // add new edges in the elevated range
        graph_[prev_vertex].push_back(static_cast<long>(vertex_count_ + next_vertex));
        graph_[next_vertex].push_back(-static_cast<long>(vertex_count_ + prev_vertex));
        parent[next_vertex] = prev_vertex;
      }
      prev_vertex = next_vertex;


      std::pair<bool, std::optional<std::vector<size_t>>> res = reachable(100, 15, true); // 9 8 -> , 1 4 -> 15
      if (res.first) {
        std::cout << "Cycle from " << 100 << " to " << 15 << ":\n";
        for (size_t v : res.second.value()) {
          std::cout << v << " (" << problem_pointer_->getTasks()[step_map_[v].task_id].getSteps()[step_map_[v].index].machine << "), ";
        }
        std::cout << "\n";
      }

      if (containsCycle()) {
        LOG_F(INFO, "Graph contains Cycles!");
        return;
      }

    }

    
    // likely need to change cliques a bit, add set of vertices that are in it,
    // need intermediate view that has the vertices in order
  }

}

/*
        std::cout << "Cycle is by adding edge from " << prev_vertex << " to " << next_vertex << ":\n";
        for (size_t v : edge_attempt_res.second.value()) {
          std::cout << v << " (" << problem_pointer_->getTasks()[step_map_[v].task_id].getSteps()[step_map_[v].index].machine << "), ";
        }
        std::cout << "\n";



        size_t p_left_v = 0;
        size_t p_right_v = 0;

        auto& cycle_path = edge_attempt_res.second.value();
        for (unsigned int i = 0; i < cycle_path.size() - 1; ++i) {
          p_left_v = cycle_path[i];
          p_right_v = cycle_path[i + 1];
          const Task::Step& p_left = m_tasks[step_map_[p_left_v].task_id].getSteps()[step_map_[p_left_v].index];
          const Task::Step& p_right = m_tasks[step_map_[p_right_v].task_id].getSteps()[step_map_[p_right_v].index];

          if (right.task_id == p_left.task_id && left.task_id == p_right.task_id) {
            DLOG_F(INFO, "found transition");
          }
        }
*/