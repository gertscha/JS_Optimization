#include "RandomSwap.h"

#include <tuple>
#include <cmath>
#include <algorithm>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  RandomSwap::RandomSwap(Problem* problem, Optimizer::TerminationCriteria& crit, unsigned int seed,
                         std::string namePrefix)
    : GlobalOrderRep(problem, crit, namePrefix),
      seed_(seed), temperature_(0.0), total_iterations_(0), stale_counter_(0)
  {
    generator_ = std::mt19937(seed);

    cur_sol_state_ = std::vector<unsigned int>();
    best_solution_ = std::make_shared<SolutionConstructor>(sequential_exec_, problem_pointer_, prefix_);

    zero_one_dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    zero_stepCnt_dist_ = std::uniform_int_distribution<>(0, (unsigned int)step_count_ - 1);
  }


	// does multiple runs and offers some other options
  void RandomSwap::Run()
  {
    Initialize();
		while (!CheckTermination()) {

      Iterate();

      if (stale_counter_ > 50) {
        LOG_F(INFO, "no improvement for %i iterations, restarting", stale_counter_);
        Initialize();
      }
		}
    DLOG_F(INFO, "RandomSwap finished with %i iterations", total_iterations_);
  }

  void RandomSwap::Initialize()
  {
    ++restart_count_;
    // copy master seqentialExec to create a solState
    cur_sol_state_ = sequential_exec_;
    // create a random ordering to init (i.e. random start)
    std::shuffle(cur_sol_state_.begin(), cur_sol_state_.end(), generator_);

    // make a internal solution
    auto new_sol = std::make_shared<Solution>(SolutionConstructor(cur_sol_state_, problem_pointer_, prefix_));

    if (new_sol->getMakespan() <= best_solution_->getMakespan()) {
      best_solution_ = new_sol;
    }
    temperature_ = 5.0;
    stale_counter_ = 0;
  }

  void RandomSwap::Iterate()
  {
    ++total_iterations_;
    // save current solState
    std::vector<unsigned int> prev_sol_state = cur_sol_state_;

		// randomly swap consecutive elements, fewer with lower temperature
		unsigned int numSwaps = (int)floor(temperature_ / 0.5) + 1;
		auto itB = cur_sol_state_.begin();
		for (unsigned int i = 0; i < numSwaps; ++i) {
			unsigned int rngIndex = zero_stepCnt_dist_(generator_);
			if (rngIndex == step_count_ - 1)
				std::iter_swap(itB, itB + rngIndex);
			else
				std::iter_swap(itB + rngIndex, itB + rngIndex + 1);
		}

		// build solution
    auto newSol = std::make_shared<SolutionConstructor>(cur_sol_state_, problem_pointer_, prefix_);

		// calc cost
		long cost = newSol->getMakespan() - best_solution_->getMakespan();
		// take if better, or with probability dependent on temperature
    bool kept = false;
		if (cost < 0) {
      best_solution_ = newSol;
      stale_counter_ = 0;
      kept = true;
		}
		else if (temperature_ > 0.0) {
			// newSol is worse, keep anyway with some probability
			double acceptance_prob = exp(-(static_cast<double>(cost)) / temperature_);
			if (zero_one_dist_(generator_) <= acceptance_prob) {
        kept = true;
			}
		}
		
    temperature_ -= 0.005;
		
    if (!kept) {
		  cur_sol_state_ = prev_sol_state;
      ++stale_counter_;
    }

  }

	// returns true if termination criteria reached
  bool RandomSwap::CheckTermination()
  {
		if (termination_criteria_.restart_limit >= 0 && (long)restart_count_ >= termination_criteria_.restart_limit) {
      DLOG_F(INFO, "reached restart limit");
      return true;
		}
		if (termination_criteria_.iteration_limit >= 0 && (long)total_iterations_ >= termination_criteria_.iteration_limit) {
      DLOG_F(INFO, "reached iteration limit");
      return true;
		}
		if (termination_criteria_.percentage_threshold >= 0.0) {
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
