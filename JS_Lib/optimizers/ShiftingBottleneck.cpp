#include "ShiftingBottleneck.h"

#include <iostream>
#include <cmath>

#include "loguru.hpp"

#include "Utility.h"


namespace JSOptimizer {


  ShiftingBottleneck::ShiftingBottleneck(Problem* problem, const TerminationCriteria& terminationCriteria,
                                        std::string namePrefix, unsigned int seed)
    : GraphRep(problem, terminationCriteria, std::string("ShiftingBottleneck_") + namePrefix, seed),
      temperature_(1.0), cooled_off_(false), stale_counter_(0),
      stale_threshold_(100), total_iterations_(0), current_best_make_span_(-1)
  {
    LOG_F(INFO, "Init ShiftingBottleneck for %s with seed %i", problem->getName().c_str(), seed);
    generator_ = std::mt19937_64(seed);
    swap_selection_dist_ = std::uniform_int_distribution<>(0, 4);
    zero_one_dist_ = std::uniform_real_distribution<>(0.0, 1.0);

    best_solution_ = std::make_shared<Solution>();
    task_dac_ = DacExtender(graph_);
    swap_options_ = std::vector<std::pair<size_t, size_t>>();

    // init task map
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
    Initialize();
    while (!CheckTermination())
    {
      Iterate();

      if (stale_counter_ >= stale_threshold_) {
        DLOG_F(INFO, "no improvement for %i iterations, restarting", stale_counter_);
        Initialize();
      }
    }
  }


  void ShiftingBottleneck::Initialize()
  {
    ++restart_count_;
    //reset the graph
    MarkModified();
    graph_ = graph_only_task_pred_;
    // add machine clique edges (randomized)
    ApplyCliquesWithTopoSort(true);

    // debug, error catching
    if (ContainsCycle()) {
      LOG_F(WARNING, "Graph contains Cycles (Initialize)!");
      return;
    }

    graph_paths_info_.Update();
    // prepare tracking of the best results in the current run
    current_best_make_span_ = graph_paths_info_.getMakespan();

    if (best_solution_->isInitialized() == false) {
      try {
        best_solution_ = std::make_shared<SolutionConstructor>(graph_, step_map_, problem_pointer_, prefix_);
      }
      catch (std::runtime_error e) {
        std::string what = e.what();
        LOG_F(ERROR, "Failed to build Solution in Initialize(): %s", what.c_str());
        ABORT_F("Bad Internal State");
      }
    }

    // other setup
    // init (temp - 10) iterations where all swaps are done and sol is accepted
    temperature_ = 100.0;
    cooled_off_ = false;
    stale_counter_ = 0;
    stale_threshold_ = 150;
  }


  void ShiftingBottleneck::Iterate()
  {
    ++total_iterations_;
    // calculate timings and critical path (should be a no-op if 
    // sol was accepted, and give the same as previously otherwise
    // thus making a copy would be more efficient)
    graph_paths_info_.Update();

    swap_options_.clear();
    int select = swap_selection_dist_(generator_);

    // limit the options if solution is stale
    if (stale_counter_ > 50) {
      select = select % 2;
    }

    switch (select)
    {
      case 0:
      case 2:
        CollectSwapsMachineBlockStart();
        break;
      case 1:
        CollectSwapsMachineBlockReorder();
        break;
      case 3:
        //break;
      case 4:
        CollectSwapsImproveMachineForwardSwap();
        //collectSwapsImproveTask();
        break;
      default:
        DLOG_F(WARNING, "Invalid 'select' in Iterate()");
    }
    // ensure to do have some swaps if at all possible
    if (swap_options_.empty()) {
      DLOG_F(INFO, "Faild to find any swap options with inital approach");
      CollectSwapsMachineBlockStart();
      if (swap_options_.empty()) {
        CollectSwapsMachineBlockReorder();
        if (swap_options_.empty()) {
            CollectSwapsImproveMachineForwardSwap();
          //if (swap_options_.empty()) {
            //collectSwapsImproveTask();
          //}
        }
      }
    }
    if (swap_options_.empty()) {
      LOG_F(INFO, "No more swaps available, restarting");
      stale_counter_ = stale_threshold_ + 1;
      return;
    }
    // select the number of swaps based on temperature
    int floored_temp = static_cast<int>(floor(temperature_ + 1.0));
    size_t count = swap_options_.size() < floored_temp ? swap_options_.size() : floored_temp;
    auto selected_indices = Utility::randomPullUniqueFromRange<size_t>
                                      (0, swap_options_.size() - 1, count, generator_);
    // do the swaps
    for (size_t i : selected_indices) {
      auto& p = swap_options_[i];
      SwapVertexRelation(p.first, p.second);
    }

    // debug, error catching
    if (ContainsCycle()) {
      LOG_F(WARNING, "Graph contains Cycles (Iterate), undoing swaps (from %i) and aborting!", select);
      for (int i = static_cast<int>(selected_indices.size()) - 1; i >= 0; --i) {
        auto& p = swap_options_[selected_indices[i]];
        SwapVertexRelation(p.second, p.first);
      }
      total_iterations_ = UINT_MAX - 10;
    }

    // calculate the cost (compared only to current run)
    graph_paths_info_.Update();
    long cost = graph_paths_info_.getMakespan() - current_best_make_span_;
    // take if better, or with probability dependent on temperature
    bool kept = false;
    if (cost < 0) {
      kept = true;
      stale_counter_ = 0;
      current_best_make_span_ = graph_paths_info_.getMakespan();
      //DLOG_F(INFO, "Found better solution for current run, ms: %i, it: %i", current_best_make_span_, total_iterations_);
      // update best overall solution if better
      if (current_best_make_span_ < best_solution_->getMakespan()) {
        try {
          best_solution_ = std::make_shared<Solution>(SolutionConstructor(graph_, step_map_, problem_pointer_, prefix_));
        }
        catch (std::runtime_error e) {
          std::string what = e.what();
          LOG_F(ERROR, "Failed to build Solution during Iterate(): %s", what.c_str());
          LOG_F(WARNING, "trying to recover");
          stale_counter_ = stale_threshold_ + 1;
          return;
        }
      }
    }
    else if (temperature_ > 0.0) {
      // swaps made solution worse, keep anyway with some probability
      double acceptance_prob = exp(-(static_cast<double>(cost)) / temperature_);
      if (zero_one_dist_(generator_) <= acceptance_prob) {
        kept = true;
      }
    }

    if (!kept) {
      ++stale_counter_;
      // undo swaps
      for (int i = static_cast<int>(selected_indices.size()) - 1; i >= 0; --i) {
        auto& p = swap_options_[selected_indices[i]];
        SwapVertexRelation(p.second, p.first);
      }
    }
    
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


  bool ShiftingBottleneck::CheckTermination()
  {
    if (termination_criteria_.restart_limit >= 0
        && static_cast<long>(restart_count_) >= termination_criteria_.restart_limit) {
      DLOG_F(INFO, "reached restart limit");
      return true;
    }
    if (termination_criteria_.iteration_limit >= 0
      && static_cast<long>(total_iterations_) >= termination_criteria_.iteration_limit)
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


  void ShiftingBottleneck::ApplyCliquesWithTopoSort(bool randomize)
  {
    MarkModified();

    DacExtender topo = task_dac_;
    auto undirected_edges = std::vector<std::pair<size_t, size_t>>();

    // get all undirected edges based on the cliques
    for (const std::set<size_t>& clique : cliques_) {
      for (auto it1 = clique.begin(); it1 != clique.end(); ++it1) {
        auto it2 = it1; // no duplicates
        for (++it2; it2 != clique.end(); ++it2) // no self edges with preincrement
        {
          undirected_edges.emplace_back(*it1, *it2);
        }
      }
    }
    if (randomize) {
      size_t length = undirected_edges.size() - 1;
      // shuffle insertion order
      std::shuffle(undirected_edges.begin(), undirected_edges.end(), generator_);
      // shuffle initial orientation of half the edges
      auto indices = Utility::randomPullUniqueFromRange<size_t>(0, length, (length-1) / 2, generator_);
      for (size_t index : indices) {
        auto& p = undirected_edges[index];
        size_t left = p.first;
        p.first = p.second;
        p.second = left;
      }
    }
    // turn all the undirected edges into directed ones
    for (std::pair<size_t, size_t> p : undirected_edges) {
      // determine orientation
      auto directed_edge = topo.InsertEdge(p.first, p.second);
      // add the edge to the graph_
      graph_[directed_edge.first].push_back(static_cast<long>(directed_edge.second + vertex_count_));
      graph_[directed_edge.second].push_back(-static_cast<long>(directed_edge.first + vertex_count_));
    }
  }


  void ShiftingBottleneck::SwapVertexRelation(size_t left, size_t right)
  {
    MarkModified();
    //DLOG_F(INFO, "swapping %i with %i", left, right);

    size_t pred = getDirectElevatedPredecessor(right, graph_);
    if (left != pred) {
      //PrintVertexRelations(std::cout);
      DLOG_F(ERROR, "predecessor %i unexpected, expected %i!", static_cast<int>(left), static_cast<int>(pred));
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

  void ShiftingBottleneck::CollectSwapsMachineBlockStart()
  {
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    const auto& tasks = problem_pointer_->getTasks();
    unsigned int m_count = problem_pointer_->getMachineCount();
  
    auto modified_machines = std::vector<bool>(m_count, false);
    size_t prev_vertex = 0;
    unsigned int prev_machine = m_count + 1;
    unsigned int prev_tid = 0;
    unsigned int counter = 0;

    for (unsigned int i = 1; i < critical_path.size() - 1; ++i) {
      size_t vert = critical_path[i];
      const Task::Step& step = getStepFromVertex(vert);
      if (step.machine != prev_machine) {
        counter = 0;
        prev_machine = step.machine;
        prev_vertex = vert;
        prev_tid = step.task_id;
      }
      else if (step.machine == prev_machine) {
        ++counter;
        if (counter == 1 && !modified_machines[step.machine]
            && step.task_id != prev_tid) {
          modified_machines[step.machine] = true;
          swap_options_.push_back({ prev_vertex, vert });
        }
      }
    }
  }


  void ShiftingBottleneck::CollectSwapsMachineBlockReorder()
  {
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    const auto& tasks = problem_pointer_->getTasks();
    unsigned int m_count = problem_pointer_->getMachineCount();

    auto modified_machines = std::vector<bool>(m_count, false);
    unsigned int modified_machine_counter = 0;
    unsigned int prev_machine = m_count + 1;
    auto sequences = std::vector<std::vector<size_t>>();
    sequences.emplace_back(std::vector<size_t>());
    // add sequences on the same machine, ignores first element
    // as that cases is covered with collectSwapsLongBlocks()
    for (unsigned int i = 1; i < critical_path.size() - 1; ++i) {
      size_t vert = critical_path[i];
      const Task::Step& step = getStepFromVertex(vert);
      if (step.machine == prev_machine) {
        sequences.back().push_back(vert);
      }
      else {
        prev_machine = step.machine;
        if (!sequences.back().empty()) {
          sequences.emplace_back(std::vector<size_t>());
        }
      }
    }
    for (auto& seq : sequences) {
      if (seq.size() >= 2 && modified_machine_counter < m_count)
      {
        bool success = false;
        unsigned int attempts = 0;
        // dist constructor has inclusive upper bound and + 1 has to be valid index
        unsigned int valid_indices = static_cast<unsigned int>(seq.size()) - 2;
        auto dist = std::uniform_int_distribution<>(0, valid_indices);
        unsigned int left_ind = dist(generator_);
        // we have valid_indices + 1 different sequential pairs
        while (!success && attempts < valid_indices + 1) {
          size_t left_vert = seq[left_ind];
          size_t right_vert = seq[left_ind + 1];
          const Task::Step& left_step = getStepFromVertex(left_vert);
          const Task::Step& right_step = getStepFromVertex(right_vert);
          if (left_step.task_id != right_step.task_id
              && !modified_machines[left_step.machine]) {
            modified_machines[left_step.machine] = true;
            ++modified_machine_counter;
            success = true;
            swap_options_.push_back({ left_vert, right_vert });
          }
          else {
            left_ind = (++left_ind) % (valid_indices + 1);
            ++attempts;
          }
        }
      }
    }
  }

  /*
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
  */

  void ShiftingBottleneck::CollectSwapsImproveMachineForwardSwap()
  {
    const auto& critical_path = graph_paths_info_.getCriticalPath();
    const auto& tasks = problem_pointer_->getTasks();
    unsigned int m_count = problem_pointer_->getMachineCount();

    //auto modified_machines = std::vector<bool>(m_count, false);
    unsigned int prev_tid = static_cast<unsigned int>(tasks.size());

    for (unsigned int i = 2; i < critical_path.size() - 1; ++i) {
      size_t left_vert = critical_path[i - 1];
      size_t right_vert = critical_path[i];
      const Task::Step& left_step = getStepFromVertex(left_vert);
      const Task::Step& right_step = getStepFromVertex(right_vert);
      if (left_step.task_id != prev_tid && left_step.task_id == right_step.task_id
          && left_step.machine != right_step.machine) {
        size_t direct_pred = getDirectElevatedPredecessor(left_vert, graph_);
        if (direct_pred == 0)// || modified_machines[left_step.machine])
          continue;
        swap_options_.push_back({ direct_pred, left_vert });
        //modified_machines[left_step.machine] = true;
        prev_tid = left_step.task_id;
      }
    }
  }



}
