#ifndef OPTIMIZERS_RANDOMSWAP_H_
#define OPTIMIZERS_RANDOMSWAP_H_

#include <vector>
#include <random>

#include "GlobalOrderRep.h"


namespace JSOptimizer {


	class RandomSwap : public GlobalOrderRep
	{
	public:

    RandomSwap(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria,
               std::string namePrefix, unsigned int seed);

    ~RandomSwap() {}

    // not copyable
    RandomSwap(const RandomSwap&) = delete;
    RandomSwap& operator=(const RandomSwap&) = delete;
    // not moveable
    RandomSwap(RandomSwap&&) = delete;
    RandomSwap& operator=(RandomSwap&&) = delete;


		// do multiple runs according to parameters, returns best solution
		void Run() override;
		
		// initializes an optimization run
		void Initialize() override;

		// performs an optimization iteration
		void Iterate() override;

		// returns true if termination criteria reached
		bool CheckTermination() override;
		
    std::shared_ptr<Solution> getBestSolution() override { return best_solution_; }

    std::string getOptimizerName() const override { return "RandomSwap"; };

	private:

		unsigned int seed_;
		double temperature_; // simulated annealing temp
		unsigned int total_iterations_;
    unsigned int stale_counter_;

		std::mt19937 generator_;
		std::uniform_real_distribution<> zero_one_dist_;
		std::uniform_int_distribution<> zero_stepCnt_dist_;

		// representation of solution states
		std::vector<unsigned int> cur_sol_state_;

    std::shared_ptr<Solution> best_solution_;

	}; // RandomSwap


}

#endif  // OPTIMIZERS_RANDOMSWAP_H_
