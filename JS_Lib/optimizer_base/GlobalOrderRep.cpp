#include "GlobalOrderRep.h"

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {


  GlobalOrderRep::GlobalOrderRep(Problem* problem, Optimizer::TerminationCriteria& crit)
    : Optimizer(problem, crit)
  {
    // init members
    sequential_exec_ = std::vector<unsigned int>();
    step_count_ = 0;
    // set members
    for (const Task& t : problem->getTasks()) {
      step_count_ += t.size();
      sequential_exec_.insert(sequential_exec_.end(), t.size(), t.getId());
    }

  }


  GlobalOrderRep::SolutionConstructor::SolutionConstructor(const std::vector<unsigned int>& sol, const Problem* const problem, const std::string& prefix)
  {
    // init members
    Solution::task_count_ = problem->getTaskCount();
    Solution::machine_count_ = problem->getMachineCount();
    Solution::name_ = prefix + problem->getName();
    Solution::initalized_ = true;
    Solution::makespan_ = 0;

    // setup solution matrix, contains uninitalized Steps
    Solution::solution_ = std::vector<std::vector<Solution::Step>>(machine_count_);
    const auto& machine_step_counts = problem->getStepCountForMachines();
    for (unsigned int i = 0; i < machine_count_; ++i) {
      solution_[i] = std::vector<Solution::Step>();
      solution_[i].reserve(machine_step_counts[i]);
    }

    // fill internal_sol_steps_ with the tasks,step and duration information
    // track the current index for each task
    const std::vector<Task>& problem_tasks = problem->getTasks();
    auto taskProgress = std::vector<size_t>(task_count_, 0);
    for (unsigned int i = 0; i < sol.size(); ++i)
    {
      unsigned int tid = sol[i];
      const Task& t = problem_tasks[tid];
      const Task::Step& s = t.getSteps()[taskProgress[tid]];
      ++taskProgress[tid];
      // create SolStep, times set to uninitalized (i.e. -1)
      solution_[s.machine].emplace_back(Solution::Step(tid, s.index, s.machine, -1, -1));
    }

    // determine timings
    calculateTimings(*problem);

    // init the problemRep vectors to correct size (filling happens during first validate call)
    Solution::problem_view_ = std::vector<std::vector<Solution::Step*>>(Solution::task_count_);
    for (unsigned int i = 0; i < Solution::task_count_; ++i) {
      Solution::problem_view_[i] = std::vector<Solution::Step*>(taskProgress[i]);
    }

  }


}