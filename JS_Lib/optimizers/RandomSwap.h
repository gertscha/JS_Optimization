#ifndef OPTIMIZERS_RANDOMSWAP_H_
#define OPTIMIZERS_RANDOMSWAP_H_

#include <string>
#include <vector>
#include <random>

#include "GlobalOrderRep.h"
#include "Solution.h"
#include "Problem.h"


namespace JSOptimizer {


	class RandomSwap : public GlobalOrderRep
	{
	public:

    RandomSwap(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix);

    ~RandomSwap() {}

    // not copyable
    RandomSwap(const RandomSwap&) = delete;
    RandomSwap& operator=(const RandomSwap&) = delete;
    // not moveable
    RandomSwap(RandomSwap&&) = delete;
    RandomSwap& operator=(RandomSwap&&) = delete;


		// do multiple runs according to parameters, returns best solution
		virtual void Run();
		
		// initializes an optimization run
		virtual void Initialize();

		// performs an optimization iteration
		virtual void Iterate();

		// returns true if termination criteria reached
		virtual bool CheckTermination();
		
    virtual std::shared_ptr<Solution> getBestSolution() { return best_solution_; }

	private:

		std::string prefix_;
		unsigned int seed_;
		double temperature_; // simulated annealing temp
		unsigned int total_iterations_;

		std::mt19937 generator_;
		std::uniform_real_distribution<> zero_one_dist_;
		std::uniform_int_distribution<> zero_stepCnt_dist_;

		// representation of solution states
		std::vector<unsigned int> cur_sol_state_;

    std::shared_ptr<Solution> best_solution_;

	}; // RandomSwap


}

#endif  // OPTIMIZERS_RANDOMSWAP_H_