#ifndef OPTIMIZERS_RANDOMSEARCH_H_
#define OPTIMIZERS_RANDOMSEARCH_H_

#include <string>
#include <vector>
#include <random>

#include "GlobalOrderRep.h"
#include "Solution.h"
#include "Problem.h"


namespace JSOptimizer {

	/*
	* This optimizer uses a list (length total number of Steps) as search space
	* entries in the list are task ids, which are then constructed into the exec order
	* per machine
	*/
	class RandomSearch : public GlobalOrderRep
	{
	public:

    RandomSearch(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix);

    ~RandomSearch();

    // not copyable
    RandomSearch(const RandomSearch&) = delete;
    RandomSearch& operator=(const RandomSearch&) = delete;
    // not moveable
    RandomSearch(RandomSearch&&) = delete;
    RandomSearch& operator=(RandomSearch&&) = delete;


		// run until terminiation Cirteria reached (ignores restart limit), returns best solution
		const Solution& runOptimizer();
		
		// initializes an optimization run
		virtual void Initialize();

		// performs an optimization iteration
		virtual void Iterate();

		// returns true if termination criteria reached
		virtual bool CheckTermination();
		
		virtual const Solution& getBestSolution();

	private:

		std::string prefix_;
		unsigned int seed_;
		unsigned int total_iterations_;

		std::mt19937 generator_;

		// representation of solution states
		std::vector<unsigned int> cur_sol_state_;

		// current minumum fitness internal solution
    GlobalOrderRep::InternalSolution* best_internal_solution_;

		// may have unitialized solution and problemRep members
		Solution best_solution_;


	};




}

#endif  // OPTIMIZERS_RANDOMSEARCH_H_
