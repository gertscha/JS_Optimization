#ifndef OPTIMIZERS_RANDOMSEARCH_H_
#define OPTIMIZERS_RANDOMSEARCH_H_

#include <string>
#include <vector>
#include <random>

#include "GlobalOrderRep.h"
#include "Solution.h"
#include "Problem.h"


namespace JSOptimizer {


	class RandomSearch : public GlobalOrderRep
	{
	public:

    RandomSearch(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix);

    ~RandomSearch() {}

    // not copyable
    RandomSearch(const RandomSearch&) = delete;
    RandomSearch& operator=(const RandomSearch&) = delete;
    // not moveable
    RandomSearch(RandomSearch&&) = delete;
    RandomSearch& operator=(RandomSearch&&) = delete;
		
		// initializes an optimization run
		virtual void Initialize();

		// performs an optimization iteration
		virtual void Iterate();

		// returns true if termination criteria reached
		virtual bool CheckTermination();
		
    // get the best solution, is nullptr if Initialize() was not called
    virtual std::shared_ptr<Solution> getBestSolution() { return best_solution_; }

	private:
    bool initialized;
		std::string prefix_;
		unsigned int seed_;
		unsigned int total_iterations_;

		std::mt19937 generator_;

		// representation of solution states
		std::vector<unsigned int> cur_sol_state_;

		// best solution
    std::shared_ptr<Solution> best_solution_;

	};


}

#endif  // OPTIMIZERS_RANDOMSEARCH_H_
