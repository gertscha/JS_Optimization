#ifndef OPTIMIZERS_RANDOMSEARCHMACHINE_H_
#define OPTIMIZERS_RANDOMSEARCHMACHINE_H_

#include <string>
#include <vector>
#include <random>

#include "MachineOrderRep.h"
#include "Solution.h"
#include "Problem.h"


namespace JSOptimizer {


	class RandomSearchMachine : public MachineOrderRep
	{
	public:

    RandomSearchMachine(Problem* problem, const TerminationCriteria& terminationCriteria,
                 std::string namePrefix, unsigned int seed);

    ~RandomSearchMachine() {}

    // not copyable
    RandomSearchMachine(const RandomSearchMachine&) = delete;
    RandomSearchMachine& operator=(const RandomSearchMachine&) = delete;
    // not moveable
    RandomSearchMachine(RandomSearchMachine&&) = delete;
    RandomSearchMachine& operator=(RandomSearchMachine&&) = delete;
		
    // do a single run
    void Run() override;

		// initializes an optimization run
		void Initialize() override;

		// performs an optimization iteration
		void Iterate() override;

		// returns true if termination criteria reached
		bool CheckTermination() override;
		
    // get the best solution, is nullptr if Initialize() was not called
    std::shared_ptr<Solution> getBestSolution() override { return best_solution_; }

    std::string getOptimizerName() const override { return "RandomSearchMachine"; };


	private:
		unsigned int total_iterations_;
    unsigned int checked_solutions_;
    unsigned int valid_solutions_found_;

		std::mt19937 generator_;

		// representation of solution states
		//std::vector<std::vector<unsigned int>> prev_sol_state_;

		// best solution
    std::shared_ptr<Solution> best_solution_;

	};


}

#endif  // OPTIMIZERS_RANDOMSEARCHMACHINE_H_
