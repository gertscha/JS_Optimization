#include "ShiftingBottleneck.h"

#include <iostream>

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
    ++restart_count_;
    Initialize();
    while (!CheckTermination()) {
      Iterate();
    }
  }

  void ShiftingBottleneck::Initialize()
  {
    for (GraphRep::MachineClique& clique : cliques_) {
      auto& order = clique.getMachineOrder();
      std::shuffle(order.begin(), order.end(), generator_);
    }
    applyCliqueOrdersToGraph();

    //printStepMap(std::cout);
    //printVertexRelations(std::cout);

    if (containsCycle()) {
      LOG_F(INFO, "Graph contains Cycles!");
      return;
    }

    auto new_sol = std::make_shared<SolutionConstructor>(graph_, step_map_, problem_pointer_, prefix_);
    if (best_solution_->isInitialized() == false || new_sol->getMakespan() < best_solution_->getMakespan())
      best_solution_ = new_sol;

  }

  void ShiftingBottleneck::Iterate()
  {

  }

  bool ShiftingBottleneck::CheckTermination()
  {
    
    return true;
  }

}