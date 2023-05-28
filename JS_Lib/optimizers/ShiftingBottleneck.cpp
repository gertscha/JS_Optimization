#include "ShiftingBottleneck.h"



#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  ShiftingBottleneck::ShiftingBottleneck(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix)
    : Optimizer(problem, terminationCriteria), GraphRep(problem, terminationCriteria),
      prefix_(namePrefix), seed_(seed), temperature_(1.0), total_iterations_(0)
  {
    generator_ = std::mt19937(seed);
    best_solution_ = std::make_shared<Solution>();
  }

  void ShiftingBottleneck::Run()
  {

  }

  void ShiftingBottleneck::Initialize()
  {

  }

  void ShiftingBottleneck::Iterate()
  {

  }

  bool ShiftingBottleneck::CheckTermination()
  {
    
    return false;
  }

}