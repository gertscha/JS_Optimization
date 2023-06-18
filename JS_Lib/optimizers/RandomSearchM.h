#ifndef OPTIMIZERS_RANDOMSEARCHM_H_
#define OPTIMIZERS_RANDOMSEARCHM_H_

#include <string>
#include <vector>
#include <random>

#include "MachineOrderRep.h"
#include "Solution.h"
#include "Problem.h"


namespace JSOptimizer {


	class RandomSearchM : public MachineOrderRep
	{
	public:

    RandomSearchM(Problem* problem, const TerminationCriteria& terminationCriteria,
                 std::string namePrefix, unsigned int seed);

    ~RandomSearchM() {}

    // not copyable
    RandomSearchM(const RandomSearchM&) = delete;
    RandomSearchM& operator=(const RandomSearchM&) = delete;
    // not moveable
    RandomSearchM(RandomSearchM&&) = delete;
    RandomSearchM& operator=(RandomSearchM&&) = delete;
		
    // uses default Run() implementation

		// initializes an optimization run
		void Initialize() override;

		// performs an optimization iteration
		void Iterate() override;

		// returns true if termination criteria reached
		bool CheckTermination() override;
		
    // get the best solution, is nullptr if Initialize() was not called
    std::shared_ptr<Solution> getBestSolution() override { return best_solution_; }

    std::string getOptimizerName() const override { return "RandomSearchM"; };


	private:
		unsigned int total_iterations_;

		std::mt19937 generator_;

		// representation of solution states
		std::vector<std::vector<unsigned int>> prev_sol_state_;

		// best solution
    std::shared_ptr<Solution> best_solution_;

	};


}

#endif  // OPTIMIZERS_RANDOMSEARCHM_H_
