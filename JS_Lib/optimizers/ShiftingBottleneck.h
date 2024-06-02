#ifndef OPTIMIZERS_SHIFTINGBOTTLENECK_H_
#define OPTIMIZERS_SHIFTINGBOTTLENECK_H_

#include <random>

#include "GraphRep.h"


namespace JSOptimizer
{

  class ShiftingBottleneck : public GraphRep
  {
  public:

    ShiftingBottleneck(
      Problem* problem,
      const TerminationCriteria& terminationCriteria,
      std::string namePrefix,
      unsigned int seed
    );

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
    bool CheckTermination() const override;

    std::shared_ptr<Solution> getBestSolution() override { return best_solution_; }

    std::string getOptimizerName() const override { return "ShiftingBottleneck"; };


  private:
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

    void ApplyCliquesWithTopoSort(bool randomize_insertion_order);
    // swap edges between left and right, must be direct successors (elevated)
    void SwapVertexRelation(size_t left, size_t right);

    // optimization heuristics
    // swap first and second element of critical path block on same machine (once per machine)
    void CollectSwapsMachineBlockStart();
    // swap random tasks of different jobs on same machine in critical path block
    void CollectSwapsMachineBlockReorder();
    // swap last job on machine forward if critical path changes machine but job remains
    void CollectSwapsImproveMachineForwardSwap();

  };


}

#endif  // OPTIMIZERS_SHIFTINGBOTTLENECK_H_
