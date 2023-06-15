#include "RandomSearch.h"

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


    JSOptimizer::RandomSearch::RandomSearch(Problem* problem, const TerminationCriteria& crit,
                                            std::string namePrefix, unsigned int seed)
      : GlobalOrderRep(problem, crit, std::string("RandomSearch_") + namePrefix, seed),
        total_iterations_(0)
    {
      generator_ = std::mt19937(seed);
      try {
        best_solution_ = std::make_shared<SolutionConstructor>(sequential_exec_, problem_pointer_, prefix_);
      }
      catch (std::runtime_error e) {
        std::string what = e.what();
        LOG_F(ERROR, "Failed to build Solution in RandomSearch Constructor: %s", what.c_str());
        ABORT_F("Bad Internal State");
      }
    }

    void RandomSearch::Initialize()
    {
      cur_sol_state_ = sequential_exec_;
      ++restart_count_;
    }

    void RandomSearch::Iterate()
    {
      ++total_iterations_;

      std::shuffle(cur_sol_state_.begin(), cur_sol_state_.end(), generator_);

      std::shared_ptr<Solution> new_sol(nullptr);
      try {
        new_sol = std::make_shared<SolutionConstructor>(cur_sol_state_, problem_pointer_, prefix_);
      }
      catch (std::runtime_error e) {
        std::string what = e.what();
        LOG_F(ERROR, "Failed to build Solution during Iterate(): %s", what.c_str());
        LOG_F(WARNING, "trying to recover");
        Initialize();
        return;
      }

      if (new_sol->getMakespan() < best_solution_->getMakespan()) {
        best_solution_ = new_sol;
      }
    }

    bool RandomSearch::CheckTermination()
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