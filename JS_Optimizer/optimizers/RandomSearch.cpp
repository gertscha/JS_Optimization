#include "RandomSwap.h"

#include "loguru.hpp"

#include "Task.h"
#include "RandomSearch.h"


namespace JSOptimizer {


  


    JSOptimizer::RandomSearch::RandomSearch(Problem* problem, Optimizer::TerminationCriteria& crit, unsigned int seed, std::string namePrefix)
      : Optimizer(problem, crit), GlobalOrderRep(problem, crit), best_internal_solution_(nullptr), prefix_(namePrefix), seed_(seed), total_iterations_(0)
    {
      generator_ = std::mt19937(seed);
      best_solution_ = Solution();
    }

    RandomSearch::~RandomSearch()
    {
      delete best_internal_solution_;
    }

    const Solution& RandomSearch::runOptimizer()
    {
      Optimizer::Run();

      return getBestSolution();
    }

    void RandomSearch::Initialize()
    {
      cur_sol_state_ = sequential_exec_;
      
      if (best_internal_solution_ == nullptr) {
        best_internal_solution_ = new GlobalOrderRep::InternalSolution(cur_sol_state_, *problem_pointer_);
      }

    }

    void RandomSearch::Iterate()
    {
      std::shuffle(cur_sol_state_.begin(), cur_sol_state_.end(), generator_);

      auto* new_sol = new GlobalOrderRep::InternalSolution(cur_sol_state_, *problem_pointer_);

      if (new_sol->getMakespan() <= best_internal_solution_->getMakespan()) {
        delete best_internal_solution_;
        best_internal_solution_ = new_sol;
      }
      else
        delete new_sol;
      
      ++total_iterations_;
    }

    bool RandomSearch::CheckTermination()
    {
      if (termination_criteria_.iteration_limit >= 0
          && total_iterations_ > static_cast<unsigned int>(termination_criteria_.iteration_limit))
      {
        DLOG_F(INFO, "reached iteration limit");
        return true;
      }
      if (termination_criteria_.percentage_threshold >= 0.0)
      {
        long lowerBound = this->problem_pointer_->getBounds().getLowerBound();
        long threshold = (long)ceil((1.0 + termination_criteria_.percentage_threshold) * lowerBound);
        long best_makespan = best_internal_solution_->getMakespan();
        if (best_makespan <= threshold) {
          DLOG_F(INFO, "reached fitness threshold after %i iterations", total_iterations_);
          return true;
        }
      }
      return false;
    }

    const Solution& RandomSearch::getBestSolution()
    {
      if (best_solution_.isInitialized()) {
        if (best_internal_solution_->getMakespan() < best_solution_.getMakespan())
          best_solution_ = GlobalOrderRep::SolutionConstructor(*best_internal_solution_, "RandomSearch");
      }
      else
        best_solution_ = GlobalOrderRep::SolutionConstructor(*best_internal_solution_, "RandomSearch");

      return best_solution_;
    }

}