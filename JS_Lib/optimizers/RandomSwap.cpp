#include "RandomSwap.h"

#include <tuple>
#include <cmath>
#include <algorithm>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  RandomSwap::RandomSwap(Problem* problem, const TerminationCriteria& crit,
                         std::string namePrefix, unsigned int seed)
    : GlobalOrderRep(problem, crit, std::string("RandomSwap_") + namePrefix, seed),
      cooled_off_(false), temperature_(0.0), total_iterations_(0), stale_counter_(0)
  {
    generator_ = std::mt19937(seed);

    cur_sol_state_ = std::vector<unsigned int>();
    try {
      best_solution_ = std::make_shared<SolutionConstructor>(sequential_exec_, problem_pointer_, prefix_);
    } catch (std::runtime_error e) {
      std::string what = e.what();
      LOG_F(ERROR, "Failed to build Solution in RandomSwap Constructor: %s", what.c_str());
      ABORT_F("Bad Internal State");
    }

    zero_one_dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    zero_stepCnt_dist_ = std::uniform_int_distribution<>(0, (unsigned int)step_count_ - 1);

    LOG_F(INFO, "Init RandomSwap for %s with seed %i", problem->getName().c_str(), seed);
  }


	// does multiple runs and offers some other options
  void RandomSwap::Run()
  {
    Initialize();
		while (!CheckTermination()) {

      Iterate();

      if (stale_counter_ >= 100 && temperature_ == 0.0) {
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
    std::shared_ptr<Solution> new_sol(nullptr);
    try {
      new_sol = std::make_shared<Solution>(SolutionConstructor(cur_sol_state_, problem_pointer_, prefix_));
    } catch (std::runtime_error e) {
      std::string what = e.what();
      LOG_F(ERROR, "Failed to build Solution during Initialize(): %s", what.c_str());
      ABORT_F("Bad Internal State");
    }

    if (new_sol->getMakespan() <= best_solution_->getMakespan()) {
      best_solution_ = new_sol;
    }
    temperature_ = 5.0;
    stale_counter_ = 0;
    cooled_off_ = false;
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
    std::shared_ptr<Solution> new_sol(nullptr);
    try {
      new_sol = std::make_shared<SolutionConstructor>(cur_sol_state_, problem_pointer_, prefix_);
    }
    catch (std::runtime_error e) {
      std::string what = e.what();
      LOG_F(ERROR, "Failed to build Solution during Iterate(): %s", what.c_str());
      LOG_F(WARNING, "trying to recover");
      stale_counter_ = 101;
      return;
    }

		// calc cost
		long cost = new_sol->getMakespan() - best_solution_->getMakespan();
		// take if better, or with probability dependent on temperature
    bool kept = false;
		if (cost < 0) {
      best_solution_ = new_sol;
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
    else if (temperature_ < 0.0) {
      cooled_off_ = true;
      temperature_ = 0.0;
    }
		
    if (!cooled_off_)
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
