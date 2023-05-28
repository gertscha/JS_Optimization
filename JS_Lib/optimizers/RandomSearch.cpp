#include "RandomSearch.h"

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


    JSOptimizer::RandomSearch::RandomSearch(Problem* problem, Optimizer::TerminationCriteria& crit, unsigned int seed, std::string namePrefix)
      : Optimizer(problem, crit), GlobalOrderRep(problem, crit), initialized(false), prefix_(namePrefix), seed_(seed), total_iterations_(0)
    {
      generator_ = std::mt19937(seed);
      best_solution_ = std::make_shared<Solution>();
    }

    void RandomSearch::Initialize()
    {
      initialized = true;
      cur_sol_state_ = sequential_exec_;
      
      if (best_solution_.get()->isInitialized() == false) {
        best_solution_ = std::make_shared<Solution>(SolutionConstructor(cur_sol_state_, problem_pointer_, prefix_));
      }

    }

    void RandomSearch::Iterate()
    {
      if (!initialized)
        Initialize();

      std::shuffle(cur_sol_state_.begin(), cur_sol_state_.end(), generator_);

      auto* new_sol = new GlobalOrderRep::SolutionConstructor(cur_sol_state_, problem_pointer_, prefix_);

      if (new_sol->getMakespan() <= best_solution_->getMakespan()) {
        best_solution_ = std::make_shared<Solution>(*new_sol);
      }
      else
        delete new_sol;
      
      ++total_iterations_;
    }

    bool RandomSearch::CheckTermination()
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


}