#ifndef OPTIMIZERS_SHIFTINGBOTTLENECK_H_
#define OPTIMIZERS_SHIFTINGBOTTLENECK_H_

#include <string>
#include <vector>
#include <random>

#include "GraphRep.h"
#include "Solution.h"
#include "Problem.h"


namespace JSOptimizer {

	class ShiftingBottleneck : public GraphRep
	{
	public:

    ShiftingBottleneck(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix);

		~ShiftingBottleneck() {}

    // not copyable
    ShiftingBottleneck(const ShiftingBottleneck&) = delete;
    ShiftingBottleneck& operator=(const ShiftingBottleneck&) = delete;
    // not moveable
    ShiftingBottleneck(ShiftingBottleneck&&) = delete;
    ShiftingBottleneck& operator=(ShiftingBottleneck&&) = delete;


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

		std::mt19937_64 generator_;

		// may have unitialized solution and problemRep members
    std::shared_ptr<Solution> best_solution_;

    void applyCliquesWithTopoSort();

	};


}

#endif  // OPTIMIZERS_SHIFTINGBOTTLENECK_H_
