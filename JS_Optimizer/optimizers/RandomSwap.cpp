#include "RandomSwap.h"

#include <tuple>
#include <cmath>
#include <algorithm>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  RandomSwap::RandomSwap(Problem* problem, Optimizer::TerminationCriteria& crit, unsigned int seed, std::string namePrefix)
    : Optimizer(problem, crit), GlobalOrderRep(problem, crit), prefix_(namePrefix), seed_(seed), temperature_(0.0), total_iterations_(0), internal_sol_max_cnt_(3)
  {
		using InternalSolWrapper = Utility::Wrapper<GlobalOrderRep::InternalSolution>;
		
		generator_ = std::mt19937(seed);
		solutions_heap_ = Utility::Heap<InternalSolWrapper>();

		sequential_exec_ = std::vector<unsigned int>();
		step_count_ = 0;
		for (const Task& t : problem->getTasks()) {
			step_count_ += t.size();
			sequential_exec_.insert(sequential_exec_.end(), t.size(), t.getId());
		}

		zero_one_dist_ = std::uniform_real_distribution<>(0.0, 1.0);
		zero_stepCnt_dist_ = std::uniform_int_distribution<>(0, (unsigned int)step_count_ -1);

		best_internal_solution_ = new GlobalOrderRep::InternalSolution(sequential_exec_, *problem_pointer_);
		solutions_heap_.add(InternalSolWrapper(best_internal_solution_));

		best_solution_ = Solution();
  }

  RandomSwap::~RandomSwap()
  {
    for (auto& wrap : solutions_heap_.getElements())
      delete wrap.pointer;
  }


	// does multiple runs and offers some other options
  const Solution& RandomSwap::runOptimizer(unsigned int num_sols)
  {
		internal_sol_max_cnt_ = num_sols;

    unsigned int iterations_sum = 0;

		while (true) {

			temperature_ = 5.0;
      Initialize();
			while (temperature_ > 0.0)
			{
        Iterate();
				++total_iterations_;
			}
      iterations_sum += total_iterations_;
      if (CheckTermination())
        break;

      total_iterations_ = 0;
			++restart_count_;
		}
    DLOG_F(INFO, "runOptimizer finished with %i iterations", iterations_sum);

		return getBestSolution();
  }


  void RandomSwap::Iterate()
  {
  	// save current solState
		prev_sol_state_ = cur_sol_state_;

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
    auto* newSol = new GlobalOrderRep::InternalSolution(cur_sol_state_, *problem_pointer_);
		// calc cost
		long cost = best_internal_solution_->getMakespan() - newSol->getMakespan();
		// take if better, or with probability dependent on temperature
		if (cost >= 0) {
      //m_bestInternal = newSol;
			goto label_accepted_sol;
		}
		else if (cost < 0 && temperature_ > 0.0) {
			// newSol is worse
			double exponential = exp(-((double)cost) / temperature_);
			if (zero_one_dist_(generator_) < exponential) {
				goto label_accepted_sol;
			}
		}
		// did not take the solution, roll back
		cur_sol_state_ = prev_sol_state_;
		delete newSol;
		return;

	label_accepted_sol:
		updateInternalSols(newSol, internal_sol_max_cnt_);
		temperature_ -= 0.005;
  }

	// returns true if termination criteria reached
  bool RandomSwap::CheckTermination()
  {
		if (termination_criteria_.restart_limit >= 0 && restart_count_ > (unsigned int)termination_criteria_.restart_limit) {
      DLOG_F(INFO, "reached restart limit");
      return true;
		}
		if (termination_criteria_.iteration_limit >= 0 && total_iterations_ > (unsigned int)termination_criteria_.iteration_limit) {
      DLOG_F(INFO, "reached iteration limit");
      return true;
		}
		if (termination_criteria_.percentage_threshold >= 0.0 && best_internal_solution_ != nullptr) {
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


  void RandomSwap::Initialize()
  {		
		// copy master seqentialExec to create a solState
		cur_sol_state_ = sequential_exec_;
		// create a random ordering to init (i.e. random start)
		std::shuffle(cur_sol_state_.begin(), cur_sol_state_.end(), generator_);

		// make a internal solution
    auto* sol_ptr = new GlobalOrderRep::InternalSolution(cur_sol_state_, *problem_pointer_);
		
		// update internal state with the new solution
		updateInternalSols(sol_ptr, internal_sol_max_cnt_);

  }


	void RandomSwap::updateInternalSols(GlobalOrderRep::InternalSolution* solptr, unsigned int limit)
	{
    using InternalSolWrapper = Utility::Wrapper<GlobalOrderRep::InternalSolution>;

		if (best_internal_solution_->getMakespan() >= solptr->getMakespan()) {
			best_internal_solution_ = solptr;
		}

		size_t num_elems = solutions_heap_.size();
		if (num_elems + 1 > limit) {
      auto discard = solutions_heap_.replace(InternalSolWrapper(solptr));
			delete discard.pointer;
		}
		else {
			solutions_heap_.add(InternalSolWrapper(solptr));
		}
	}


	std::vector<Solution> RandomSwap::getAllStoredSolutions()
	{
    using InternalSolWrapper = Utility::Wrapper<GlobalOrderRep::InternalSolution>;

		auto sol_vec = std::vector<Solution>();
		std::vector<InternalSolWrapper> heap_copy = solutions_heap_.getElements();
		std::sort(heap_copy.begin(), heap_copy.end()); // uses operator< from the wrapper

		for (unsigned int i = 0; i < heap_copy.size(); ++i) {
			sol_vec.emplace_back(SolutionConstructor(*(heap_copy[i].pointer), "RandomSwap"));
		}
		return sol_vec;
	}


	// return best, updates best if internalbest is better
	const Solution& RandomSwap::getBestSolution()
	{
    using InternalSolWrapper = Utility::Wrapper<GlobalOrderRep::InternalSolution>;
    
    if (solutions_heap_.size() > 0) {
      std::vector<InternalSolWrapper> heap_copy = solutions_heap_.getElements();
      std::sort(heap_copy.begin(), heap_copy.end()); // uses operator< from the wrapper
      if (!best_solution_.isInitialized() || heap_copy[0].pointer->getMakespan() < best_solution_.getMakespan())
        best_solution_ = SolutionConstructor(*(heap_copy[0].pointer), "RandomSwap");
    }
    else {
      InternalSolution sol = InternalSolution(sequential_exec_, *problem_pointer_);
      best_solution_ = SolutionConstructor(sol, "SequentialExecution_");
    }
    
    return best_solution_;
	}


}