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
    swaps_to_do_ = std::vector<std::pair<size_t, size_t>>();
    task_map_ = std::vector<std::vector<size_t>>(problem_pointer_->getTaskCount());
    for (size_t i = 0; i < task_map_.size(); ++i) {
      task_map_[i] = std::vector<size_t>(problem_pointer_->getTasks()[i].size());
    }
    for (size_t i = 1; i < vertex_count_ - 1; ++i) {
      const Identifier& iden = step_map_[i];
      task_map_[iden.task_id][iden.index] = i;
    }
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

    applyCliquesWithTopoSort(true);

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
    ++total_iterations_;
    graph_paths_info_.update();

    markModified();
    
    auto dist = std::uniform_int_distribution<>(0, 4);
    int select = dist(generator_);
    
    swaps_to_do_.clear();
    switch (select)
    {
      case 0:
      case 1:
        collectSwapsLongBlocks();
        break;
      case 2:
      case 3:
        collectSwapsMachineReorder();
        break;
      case 4:
        collectSwapsImproveTask();
        break;
      default:
        DLOG_F(WARNING, "Invalid selection in Iterate()");
    }
    if (swaps_to_do_.empty()) {
      DLOG_F(INFO, "No swaps after switch on %i", select);
      collectSwapsLongBlocks();
    }

    if (swaps_to_do_.empty())
      DLOG_F(WARNING, "No swaps to do!");

    for (auto& p : swaps_to_do_) {
      swapVertexRelation(p.first, p.second);
    }

    if (containsCycle()) {
      LOG_F(INFO, "Graph contains Cycles (Iterate), abort!");
      std::cout << "swaps were: ";
      for (auto& p : swaps_to_do_) {
        std::cout << "(" << p.first << "," << p.second << ") ";
        swapVertexRelation(p.second, p.first);
      }
      std::cout << "\n";
      total_iterations_ += 100000;
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


  std::shared_ptr<Solution> ShiftingBottleneck::getBestSolution()
  {
    graph_paths_info_.update();
    if (best_solution_->getMakespan() > graph_paths_info_.getMakespan())
      best_solution_ = std::make_shared<Solution>(SolutionConstructor(graph_, step_map_, problem_pointer_, prefix_));

    return best_solution_;
  }


  void ShiftingBottleneck::applyCliquesWithTopoSort(bool randomize)
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
    if (randomize) {
      // shuffle insertion order
      std::shuffle(undirected_edges.begin(), undirected_edges.end(), generator_);
    }
    // turn all the undirected edges into directed ones
    for (std::pair<size_t, size_t> p : undirected_edges) {
      // determine orientation
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


  void ShiftingBottleneck::collectSwapsLongBlocks()
  {
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    const auto& tasks = problem_pointer_->getTasks();

    size_t change_vertex = 0;
    unsigned int prev_machine = problem_pointer_->getMachineCount() + 1;
    unsigned int machine_count = 0;

    for (unsigned int i = 1; i < critical_path.size() - 1; ++i) {
      size_t vert = critical_path[i];
      const Identifier& iden = step_map_[vert];
      const Task::Step& step = tasks[iden.task_id].getSteps()[iden.index];
      if (step.machine != prev_machine) {
        machine_count = 0;
        prev_machine = step.machine;
        change_vertex = vert;
      }
      else if (step.machine == prev_machine) {
        ++machine_count;
        if (machine_count == 1 && i != 2) {
          swaps_to_do_.push_back({ change_vertex, vert });
        }
      }
    }
  }


  void ShiftingBottleneck::collectSwapsMachineReorder()
  {
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    const auto& tasks = problem_pointer_->getTasks();
    
    unsigned int prev_machine = problem_pointer_->getMachineCount() + 1;

    for (unsigned int i = 2; i < critical_path.size() - 1; ++i) {
      size_t left_vert = critical_path[i - 1];
      size_t right_vert = critical_path[i];
      const Identifier& left_iden = step_map_[left_vert];
      const Identifier& right_iden = step_map_[right_vert];
      if (left_iden.task_id != right_iden.task_id) {
        const Task::Step& left_step = tasks[left_iden.task_id].getSteps()[left_iden.index];
        const Task::Step& right_step = tasks[right_iden.task_id].getSteps()[right_iden.index];
        if (left_step.machine == right_step.machine) {
          if (left_step.machine != prev_machine) {
            swaps_to_do_.push_back({ left_vert, right_vert });
          }
          prev_machine = left_step.machine;
        }
      }
    }
  }


  void ShiftingBottleneck::collectSwapsImproveTask()
  {
    const auto& timings = graph_paths_info_.getTimings();
    const auto& tasks = problem_pointer_->getTasks();
    unsigned int t_count = problem_pointer_->getTaskCount();

    // find sequences of critical taks steps
    auto sequences = std::vector<std::vector<std::vector<size_t>>>(t_count);
    for (unsigned int i = 0; i < t_count; ++i) {
      sequences[i] = std::vector<std::vector<size_t>>();
      sequences[i].emplace_back(std::vector<size_t>());
      for (unsigned int j = 0; j < task_map_[i].size(); ++j)
      {
        size_t vert = task_map_[i][j];
        const PathsInfo::Timing& t = timings[vert];
        // if critical step
        if (t.ESD == t.LSD && t.EFD == t.LFD) {
          sequences[i].back().push_back(vert);
        }
        else {
          if (!sequences[i].back().empty()) {
            sequences[i].emplace_back(std::vector<size_t>());
          }
        }
      }
    }
    // select swap targets
    auto modified_machines = std::vector<bool>(problem_pointer_->getMachineCount(), false);
    for (auto& task_seq : sequences) {
      for (auto& seq : task_seq) {
        if (seq.size() >= 2) {
          size_t direct_pred = getDirectElevatedPredecessor(seq[0], graph_);
          if (direct_pred == 0)
            continue;
          // can only check other things once we now direct predecessor is not 0
          unsigned int mid = getStepFromVertex(direct_pred).machine;
          // limit to one change per machine, multiple swaps may invalidate previous swaps
          // i.e. direct predecessor may be wrong
          if (modified_machines[mid])
            continue;
          modified_machines[mid] = true;
          swaps_to_do_.push_back({ direct_pred, seq[0] });
        }
      }
    }
  }


  void ShiftingBottleneck::collectSwapsImproveMachine()
  {

  }



}
