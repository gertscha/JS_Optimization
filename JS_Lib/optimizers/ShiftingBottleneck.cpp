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
    task_dac_ = DacExtender(graph_);
  }


  void ShiftingBottleneck::Run()
  {
    ++restart_count_;
    Initialize();
    while (!CheckTermination()) {
      Iterate();
    }

    graph_paths_info_.update();
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    
    std::cout << "Critical Path is:\n";
    for (size_t vertex : critical_path) {
      std::cout << "(" << step_map_[vertex].task_id << "," << step_map_[vertex].index << ") ";
    }
    std::cout << "\n";

    best_solution_ = std::make_shared<Solution>(SolutionConstructor(graph_, step_map_, problem_pointer_, prefix_));
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

    applyCliquesWithTopoSort();

    //printStepMap(std::cout);
    //printVertexRelations(std::cout);

    if (containsCycle()) {
      LOG_F(INFO, "Graph contains Cycles (Initialize)!");
      return;
    }
    auto new_sol = std::make_shared<SolutionConstructor>(graph_, step_map_, problem_pointer_, prefix_);
    if (best_solution_->isInitialized() == false || new_sol->getMakespan() < best_solution_->getMakespan())
      best_solution_ = new_sol;
  }


  void ShiftingBottleneck::Iterate()
  {
    /*
    std::cout << "Timings are:\n";
    unsigned int v_id = 0;
    for (const PathsInfo::Timing& t : graph_paths_info_.getTimings()) {
      std::cout << "(" << step_map_[v_id].task_id << ", " << step_map_[v_id].index << "): ";
      t.print(std::cout);
      std::cout << "\n";
      ++v_id;
    }
    std::cout << "\n";
    */

    ++total_iterations_;

    graph_paths_info_.update();
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    /*
    std::cout << "Critical Path is:\n";
    for (size_t vertex : critical_path) {
      std::cout << "(" << step_map_[vertex].task_id << "," << step_map_[vertex].index << ") ";
    }
    std::cout << "\n";
    */
    markModified();
    const auto& tasks = problem_pointer_->getTasks();

    auto swap_targets = std::vector<unsigned int>();
    for (unsigned int i = 2; i < critical_path.size() - 1; ++i) {
      size_t left_vert = critical_path[i - 1];
      size_t right_vert = critical_path[i];
      const Identifier& left_iden = step_map_[left_vert];
      const Identifier& right_iden = step_map_[right_vert];
      if (left_iden.task_id != right_iden.task_id) {
        const Task::Step& left_step = tasks[left_iden.task_id].getSteps()[left_iden.index];
        const Task::Step& right_step = tasks[right_iden.task_id].getSteps()[right_iden.index];
        if (left_step.machine == right_step.machine) {
          swap_targets.push_back(i);
        }
      }
    }

    auto dist = std::uniform_int_distribution<>(0, static_cast<int>(swap_targets.size()) - 1);

    int to_swap = swap_targets[dist(generator_)];

    size_t left_vert = critical_path[to_swap - 1];
    size_t right_vert = critical_path[to_swap];
    DLOG_F(INFO, "Swapping direction of edges (%i, %i)", static_cast<int>(left_vert), static_cast<int>(right_vert));
    swapVertexRelation(left_vert, right_vert);

    if (containsCycle()) {
      LOG_F(INFO, "Graph contains Cycles (Iterate)!");
      return;
    }

  }


  bool ShiftingBottleneck::CheckTermination()
  {
    if (termination_criteria_.iteration_limit >= 0
      && total_iterations_ >= static_cast<unsigned int>(termination_criteria_.iteration_limit))
    {
      DLOG_F(INFO, "reached iteration limit");
      return true;
    }
    if (termination_criteria_.percentage_threshold >= 0.0)
    {
      long lowerBound = this->problem_pointer_->getBounds().getLowerBound();
      long threshold = (long)ceil((1.0 + termination_criteria_.percentage_threshold) * lowerBound);
      if (best_solution_->getMakespan() <= threshold) {
        DLOG_F(INFO, "reached fitness threshold after %i iterations", total_iterations_);
        return true;
      }
    }
    return false;
  }


  void ShiftingBottleneck::applyCliquesWithTopoSort()
  {
    markModified();

    DacExtender topo = task_dac_;
    auto undirected_edges = std::vector<std::pair<size_t, size_t>>();

    // get all undirected edges based on the cliques
    for (MachineClique& clique : cliques_) {
      const auto& set = clique.getCliqueMembers();
      for (auto it1 = set.begin(); it1 != set.end(); ++it1) {
        auto it2 = it1; // no duplicates
        for (++it2; it2 != set.end(); ++it2) // no self edges with preincrement
        {
          undirected_edges.emplace_back(*it1, *it2);
        }
      }
    }
    // turn all the undirected edges into directed ones
    for (std::pair<size_t, size_t> p : undirected_edges) {
      auto directed_edge = topo.insertEdge(p.first, p.second);
      // add the edge to the graph_
      graph_[directed_edge.first].push_back(static_cast<long>(directed_edge.second + vertex_count_));
      graph_[directed_edge.second].push_back(-static_cast<long>(directed_edge.first + vertex_count_));
    }
  }


  void ShiftingBottleneck::swapVertexRelation(size_t left, size_t right)
  {
    graph_[left].push_back(-static_cast<long>(right + vertex_count_)); // left has right as predecessor
    graph_[right].push_back(static_cast<long>(left + vertex_count_)); // right has left as successor
    // delete left has right as successor edge
    for (auto it = graph_[left].begin(); it != graph_[left].end(); ) {
      if (*it == static_cast<long>(right + vertex_count_)) {
        it = graph_[left].erase(it);
      }
      else {
        ++it;
      }
    }
    // delete right has left as predecessor edge
    for (auto it = graph_[right].begin(); it != graph_[right].end(); ) {
      if (*it == -static_cast<long>(left + vertex_count_)) {
        it = graph_[right].erase(it);
      }
      else {
        ++it;
      }
    }
  }



}
