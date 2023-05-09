#include "DisjunctiveGraphRep.h"


namespace JSOptimizer {


  DisjunctiveGraphRep::DisjunctiveGraphRep(Problem* problem, Optimizer::TerminationCriteria& criteria)
    : Optimizer(problem, criteria)
  {

  }


  DisjunctiveGraphRep::InternalSolution::InternalSolution()
  {


  }


  DisjunctiveGraphRep::SolutionConstructor::SolutionConstructor(const InternalSolution& solution, const std::string& prefix)
  {


  }


}