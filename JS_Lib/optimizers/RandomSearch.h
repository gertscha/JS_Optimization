#ifndef OPTIMIZERS_RANDOMSEARCH_H_
#define OPTIMIZERS_RANDOMSEARCH_H_

#include <string>
#include <vector>
#include <random>

#include "GlobalOrderRep.h"
#include "Solution.h"
#include "Problem.h"


namespace JSOptimizer
{

  class RandomSearch : public GlobalOrderRep
  {
  public:

    RandomSearch(
      Problem* problem,
      const TerminationCriteria& terminationCriteria,
      std::string namePrefix,
      unsigned int seed
    );

    ~RandomSearch() {}

    // not copyable
    RandomSearch(const RandomSearch&) = delete;
    RandomSearch& operator=(const RandomSearch&) = delete;
    // not moveable
    RandomSearch(RandomSearch&&) = delete;
    RandomSearch& operator=(RandomSearch&&) = delete;

    // uses default Run() implementation

    // initializes an optimization run
    void Initialize() override;

    // performs an optimization iteration
    void Iterate() override;

    // returns true if termination criteria reached
    bool CheckTermination() const override;

    // get the best solution, is nullptr if Initialize() was not called
    std::shared_ptr<Solution> getBestSolution() override { return best_solution_; }

    std::string getOptimizerName() const override { return "RandomSearch"; };


  private:
    unsigned int total_iterations_;

    std::mt19937 generator_;

    // representation of solution states
    std::vector<unsigned int> cur_sol_state_;

    // best solution
    std::shared_ptr<Solution> best_solution_;

  };


}

#endif  // OPTIMIZERS_RANDOMSEARCH_H_
