#include "ShiftingBottleneck.h"

#include <set>
#include <cmath>
#include <iostream>

#include "loguru.hpp"

#include "Task.h"
#include "RandomUtility.h"


namespace JSOptimizer {


  ShiftingBottleneck::ShiftingBottleneck(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix)
    : Optimizer(problem, terminationCriteria), GraphRep(problem, terminationCriteria),
      prefix_(namePrefix), seed_(seed), temperature_(1.0), cooled_off_(false), stale_counter_(0), total_iterations_(0)
  {
    generator_ = std::mt19937_64(seed);
    swap_selection_ = std::uniform_int_distribution<>(0, 3);
    zero_one_dist_ = std::uniform_real_distribution<>(0.0, 1.0);

    best_solution_ = std::make_shared<Solution>();
    task_dac_ = DacExtender(graph_);
    swap_options_ = std::vector<std::pair<size_t, size_t>>();

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
    // add machine clique edges (randomized)
    applyCliquesWithTopoSort(false);

    if (containsCycle()) {
      LOG_F(INFO, "Graph contains Cycles (Initialize)!");
      return;
    }
    auto new_sol = std::make_shared<SolutionConstructor>(graph_, step_map_, problem_pointer_, prefix_);
    if (best_solution_->isInitialized() == false || new_sol->getMakespan() < best_solution_->getMakespan())
      best_solution_ = new_sol;

    // other setup
    // init (temp - 10) iterations where all swaps are done and sol is accepted
    temperature_ = 200.0;
    cooled_off_ = false;
    stale_counter_ = 0;
  }


  void ShiftingBottleneck::Iterate()
  {
    DLOG_F(INFO, "Iterate");
    ++total_iterations_;
    graph_paths_info_.update();

    swap_options_.clear();
    int select = swap_selection_(generator_);
    switch (select)
    {
      case 0:
        collectSwapsMachineReorder();
        break;
      case 1:
        collectSwapsLongBlocks();
        break;
      case 2:
        collectSwapsImproveTask();
        break;
      case 3:
        collectSwapsImproveMachine();
        break;
      default:
        DLOG_F(WARNING, "Invalid 'select' in Iterate()");
    }
    if (swap_options_.empty()) {
      DLOG_F(WARNING, "No swaps after switch on %i", select);
      collectSwapsLongBlocks();
    }
    if (swap_options_.empty())
      ABORT_F("No swaps to do!");

    // select the number of swaps based on temperature
    //int floored_temp = static_cast<int>(floor(temperature_ + 1.0));
    //size_t count = swap_options_.size() < floored_temp ? swap_options_.size() : floored_temp;
    //auto swaps_selected = Utility::randomPullUnique(swap_options_, count, generator_);

    // do the swaps
    markModified();
    for (auto& p : swap_options_) {
      swapVertexRelation(p.first, p.second);
    }
    // debug, error catching
    if (containsCycle()) {
      LOG_F(WARNING, "Graph contains Cycles (Iterate), Aborting!");
      std::cout << "swaps were: ";
      for (auto& p : swap_options_) {
        std::cout << "(" << p.first << "," << p.second << ") ";
        swapVertexRelation(p.second, p.first);
      }
      std::cout << "\n";
      total_iterations_ = UINT_MAX;;
    }

    // calculate the cost
    graph_paths_info_.update();
    long cost = graph_paths_info_.getMakespan() - best_solution_->getMakespan();
    // take if better, or with probability dependent on temperature
    bool kept = false;
    if (cost < 0) {
      kept = true;
      stale_counter_ = 0;
      best_solution_ = std::make_shared<Solution>(SolutionConstructor(graph_, step_map_, problem_pointer_, prefix_));
    }
    else if (temperature_ > 0.0) {
      // swaps made solution worse, keep anyway with some probability
      double acceptance_prob = exp(-(static_cast<double>(cost)) / temperature_);
      if (zero_one_dist_(generator_) <= acceptance_prob) {
        kept = true;
      }
    }
    else {
      ++stale_counter_;
    }

    if (!kept) {
      // undo
      for (auto& p : swap_options_) {
        swapVertexRelation(p.second, p.first);
      }
    }
    else {
      if (!cooled_off_) {
        if (temperature_ > 10.0)
          temperature_ -= 1.0; // inital_temp - 10 iterations
        else if (temperature_ > 1.0)
          temperature_ -= 0.045; // 200 iterations
        else
          temperature_ -= 0.01; // 100 iterations

        if (temperature_ < 0.0) {
          temperature_ = 0.0;
          cooled_off_ = true;
        }
      }
    }
  }


  bool ShiftingBottleneck::CheckTermination()
  {
    if (stale_counter_ > 30) {
      LOG_F(INFO, "reached stable solution");
      return true;
    }
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
    if (left != getDirectElevatedPredecessor(right, graph_)) {
      DLOG_F(ERROR, "cannot swap if not direct predecessor!");
      return;
    }

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


  /*/////////////////////
      Swaps Selectors
  /////////////////////*/

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
          swap_options_.push_back({ change_vertex, vert });
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
            swap_options_.push_back({ left_vert, right_vert });
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
          swap_options_.push_back({ direct_pred, seq[0] });
        }
      }
    }
  }


  void ShiftingBottleneck::collectSwapsImproveMachine()
  {
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    const auto& tasks = problem_pointer_->getTasks();

    unsigned int prev_tid = tasks.size() + 1;

    for (unsigned int i = 2; i < critical_path.size() - 1; ++i) {
      size_t left_vert = critical_path[i - 1];
      size_t right_vert = critical_path[i];
      const Task::Step& left_step = getStepFromVertex(left_vert);
      const Task::Step& right_step = getStepFromVertex(right_vert);
      if (left_step.task_id != prev_tid && left_step.task_id == right_step.task_id
          && left_step.machine != right_step.machine) {
        size_t direct_pred = getDirectElevatedPredecessor(left_vert, graph_);
        if (direct_pred == 0)
          continue;
        swap_options_.push_back({ direct_pred, left_vert });
        prev_tid = left_step.task_id;
      }
    }
  }



}
