#include "RandomSearchMachine.h"

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  JSOptimizer::RandomSearchMachine::RandomSearchMachine(Problem* problem, const TerminationCriteria& crit,
                                                        std::string namePrefix, unsigned int seed)
    : MachineOrderRep(problem, crit, std::string("RandomSearchMachine_") + namePrefix, seed),
      total_iterations_(0), checked_solutions_(0), valid_solutions_found_(0)
  {
    generator_ = std::mt19937(seed);
    try {
      best_solution_ = std::make_shared<SolutionConstructor>(cliques_, step_map_, problem_pointer_, prefix_);
    }
    catch (std::runtime_error e) {
      std::string what = e.what();
      LOG_F(ERROR, "Failed to build Solution in RandomSearchM Constructor: %s", what.c_str());
      ABORT_F("Bad Internal State");
    }
    LOG_F(INFO, "Init RandomSearchMachine for %s with seed %i", problem->getName().c_str(), seed);
  }

  void RandomSearchMachine::Run()
  {
    Initialize();
    while (!CheckTermination()) {
      Iterate();
    }
    DLOG_F(INFO, "RandomSearchMachine evaluated %i solutions and found %i valid solutions", checked_solutions_, valid_solutions_found_);
  }

  void RandomSearchMachine::Initialize()
  {
    for (MachineClique& clique : cliques_) {
      auto& rep = clique.getMachineOrder();
      std::shuffle(rep.begin(), rep.end(), generator_);
    }
    ++restart_count_;
  }

  void RandomSearchMachine::Iterate()
  {
    ++total_iterations_;

    //prev_sol_state_ = std::vector<std::vector<unsigned int>>(m_count_);
    unsigned int i = 0;
    for (MachineClique& clique : cliques_) {
      std::vector<unsigned int>& rep = clique.getMachineOrder();
      //prev_sol_state_[i] = rep;
      ++i;
      std::shuffle(rep.begin(), rep.end(), generator_);
    }

    std::shared_ptr<Solution> new_sol(nullptr);
    try {
      ++checked_solutions_;
      new_sol = std::make_shared<SolutionConstructor>(cliques_, step_map_, problem_pointer_, prefix_);
    }
    catch (std::runtime_error e) {
      std::string what = e.what();
      LOG_F(ERROR, "Failed to build Solution during Iterate(): %s", what.c_str());
      LOG_F(WARNING, "trying to recover");
      Initialize();
      return;
    }
    if (!new_sol->isInitialized()) {
      return;
    }
    ++valid_solutions_found_;
    if (new_sol->getMakespan() < best_solution_->getMakespan()) {
      DLOG_F(INFO, "Found new improved Solution in RandomSearchMachine");
      best_solution_ = new_sol;
    }
  }

  bool RandomSearchMachine::CheckTermination()
  {
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


}