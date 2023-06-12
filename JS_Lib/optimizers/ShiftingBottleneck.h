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
		
    virtual std::shared_ptr<Solution> getBestSolution();

	private:

		std::string prefix_;
		unsigned int seed_;
		double temperature_; // simulated annealing temp
		unsigned int total_iterations_;

		std::mt19937_64 generator_;

    // map task id, index to vertices
    std::vector<std::vector<size_t>> task_map_;

		// may have unitialized solution and problemRep members
    std::shared_ptr<Solution> best_solution_;

    DacExtender task_dac_;
    
    std::vector<std::pair<size_t, size_t>> swaps_to_do_;

    void applyCliquesWithTopoSort(bool randomize_insertion_order);
    // swap edges between left and right, must be direct successors (elevated)
    void swapVertexRelation(size_t left, size_t right);
    
    // optimization heuristics
    // swap first and second element of critical path block on same machine
    void collectSwapsLongBlocks();
    // swap different task on same machine
    void collectSwapsMachineReorder();
    // swap first element of sequences of same task forward
    // swap tasks that are in critical path blocks and have float to predecessor
    void collectSwapsImproveTask();
    // swap last task on machine if critical path changes machine but task remains
    void collectSwapsImproveMachine();

	};
  

}

#endif  // OPTIMIZERS_SHIFTINGBOTTLENECK_H_
