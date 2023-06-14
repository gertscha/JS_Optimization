#ifndef UTILITY_STATSCOLLECTOR_H_
#define UTILITY_STATSCOLLECTOR_H_

#include <type_traits>
#include <string>
#include <vector>
#include <memory>

#include "Optimizer.h"
#include "Solution.h"


namespace JSOptimizer::Utility {

  template <typename T>
  concept OptimizerObject = std::is_base_of<Optimizer, T>::value;

  class StatsCollector {
  public:
    
    StatsCollector(Optimizer::TerminationCriteria termination_criteria);

    // only pointers/references of Optimizer may exist to prevent object slicing 
    void addRun(Optimizer& optimizer);

    void log();

  private:

    struct RunStat {
      std::string problem_name;
      std::string optimizer_name;
      long best_makespan;
      std::shared_ptr<Solution> solution;
    };

    std::vector<RunStat> run_stats_;
    Optimizer::TerminationCriteria criteria_;


  };

}

#endif // UTILITY_STATSCOLLECTOR_H_