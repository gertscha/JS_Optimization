#ifndef OPTIMIZERS_SHIFTINGBOTTLENECK_H_
#define OPTIMIZERS_SHIFTINGBOTTLENECK_H_

#include <random>

#include "GraphRep.h"


namespace JSOptimizer {


	class ShiftingBottleneck : public GraphRep
	{
	public:

    ShiftingBottleneck(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria,
                       unsigned int seed, std::string namePrefix);

		~ShiftingBottleneck() {}

    // not copyable
    ShiftingBottleneck(const ShiftingBottleneck&) = delete;
    ShiftingBottleneck& operator=(const ShiftingBottleneck&) = delete;
    // not moveable
    ShiftingBottleneck(ShiftingBottleneck&&) = delete;
    ShiftingBottleneck& operator=(ShiftingBottleneck&&) = delete;


		// do a run, stops if stable solution or if termination criteria reached
		void Run() override;
		
		// initializes an optimization run
		void Initialize() override;

		// performs an optimization iteration
		void Iterate() override;

		// returns true if termination criteria reached
		bool CheckTermination() override;
		
    std::shared_ptr<Solution> getBestSolution() override { return best_solution_; }

	private:

		unsigned int seed_;
		double temperature_; // simulated annealing temp
    bool cooled_off_;
    unsigned int stale_counter_;
    unsigned int stale_threshold_;
		unsigned int total_iterations_;

		std::mt19937_64 generator_;
    std::uniform_int_distribution<> swap_selection_dist_;
    std::uniform_real_distribution<> zero_one_dist_;

		// may have unitialized solution and problemRep members
    std::shared_ptr<Solution> best_solution_;
    // the best solution of the current run
    long current_best_make_span_;

    // copy of the basic dac with no machine edges, stored for restarts
    DacExtender task_dac_;
    
    // swap finding options get put in here
    std::vector<std::pair<size_t, size_t>> swap_options_;
    // map task id, index to vertices, used during swap finding
    std::vector<std::vector<size_t>> task_map_;

    void applyCliquesWithTopoSort(bool randomize_insertion_order);
    // swap edges between left and right, must be direct successors (elevated)
    void swapVertexRelation(size_t left, size_t right);
    
    // optimization heuristics
    // swap first and second element of critical path block on same machine (once per machine)
    void collectSwapsMachineBlockStart();
    // swap random steps of different task on same machine in critical path block
    void collectSwapsMachineBlockReorder();
    // swap first element of sequences of same task forward
    // swap tasks that are in critical path blocks and have float to predecessor
    void collectSwapsImproveTask();
    // swap last task on machine forward if critical path changes machine but task remains
    void collectSwapsImproveMachineForwardSwap();

	};
  

}

#endif  // OPTIMIZERS_SHIFTINGBOTTLENECK_H_
