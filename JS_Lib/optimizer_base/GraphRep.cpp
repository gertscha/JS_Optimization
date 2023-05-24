#include "GraphRep.h"


namespace JSOptimizer {


  GraphRep::GraphRep(Problem* problem, Optimizer::TerminationCriteria& criteria)
    : Optimizer(problem, criteria)
  {

  }


  GraphRep::InternalSolution::InternalSolution()
  {


  }


  GraphRep::SolutionConstructor::SolutionConstructor(const InternalSolution& solution, const std::string& prefix)
  {


  }


}