#include "ShiftingBottleneck.h"

#include <iostream>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  ShiftingBottleneck::ShiftingBottleneck(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix)
    : Optimizer(problem, terminationCriteria), GraphRep(problem, terminationCriteria),
      prefix_(namePrefix), seed_(seed), temperature_(1.0), total_iterations_(0)
  {
    generator_ = std::mt19937_64(seed);
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
    //reset the graph
    graph_ = graph_only_task_pred_;
    // randomize all the cliques
    for (GraphRep::MachineClique& clique : cliques_) {
      auto& order = clique.getMachineOrder();
      std::shuffle(order.begin(), order.end(), generator_);
    }

    // choose random clique and apply it to the graph
    std::uniform_int_distribution<size_t> ind_dist(0, cliques_.size());
    size_t index = ind_dist(generator_);
    applyCliqueToGraph(cliques_[index]);

    // try to add other cliques, modify them if the schedule is unfeasable
    for (unsigned int i = 0; i < cliques_.size(); ++i) {
      if (i != index) {
        initAddCliqueIncrementally(cliques_[i]);
      }
    }

    //applyAllCliquesToGraph();

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

  void ShiftingBottleneck::initAddCliqueIncrementally(MachineClique& clique)
  {
    // try to add edges for all other cliques, if it creates a cycle
    // find closest step from the same clique and swap them
    
    // likely need to change cliques a bit, add set of vertices that are in it,
    // need intermediate view that has the vertices in order
  }

}