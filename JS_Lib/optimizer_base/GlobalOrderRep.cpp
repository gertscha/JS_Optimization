#include "GlobalOrderRep.h"

#include "loguru.hpp"

#include "Job.h"


namespace JSOptimizer
{

  GlobalOrderRep::GlobalOrderRep(
    Problem* problem,
    const TerminationCriteria& crit,
    std::string prefix,
    unsigned int seed
  )
    : Optimizer(problem, crit, prefix, seed)
  {
    // init members
    sequential_exec_ = std::vector<unsigned int>();
    task_count_ = 0;
    // set members
    for (const Job& t : problem->getJobs())
    {
      task_count_ += t.size();
      sequential_exec_.insert(sequential_exec_.end(), t.size(), t.getId());
    }

  }


  GlobalOrderRep::SolutionConstructor::SolutionConstructor(
    const std::vector<unsigned int>& sol,
    const Problem* const problem,
    const std::string& prefix
  )
  {
    // init members
    Solution::job_count_ = problem->getJobCount();
    Solution::machine_count_ = problem->getMachineCount();
    Solution::name_ = prefix + problem->getName();
    Solution::initialized_ = true;
    Solution::makespan_ = 0;

    // setup solution matrix, contains uninitialized Steps
    Solution::solution_ = std::vector<std::vector<Solution::SolTask>>(machine_count_);
    const auto& machine_task_counts = problem->getTaskCountForMachines();
    for (unsigned int i = 0; i < machine_count_; ++i)
    {
      solution_[i] = std::vector<Solution::SolTask>();
      solution_[i].reserve(machine_task_counts[i]);
    }

    // fill sol_tasks with the jobs tasks info, leave the timing as -1
    // track the current index for each task
    const std::vector<Job>& problem_jobs = problem->getJobs();
    auto jobProgress = std::vector<size_t>(job_count_, 0);
    for (unsigned int i = 0; i < sol.size(); ++i)
    {
      unsigned int jobid = sol[i];
      const Job& j = problem_jobs[jobid];
      const Job::Task& t = j.getTasks()[jobProgress[jobid]];
      ++jobProgress[jobid];
      // create SolTask, times set to uninitialized (i.e. -1)
      solution_[t.machine].emplace_back(Solution::SolTask(jobid, t.index, t.machine, -1, -1));
    }

    // determine timings, ignore return value because this GlobalOrderRep is always a valid solution
    CalculateTimings(*problem);

    // init the problemRep vectors to correct size (filling happens during first validate call)
    Solution::problem_view_ = std::vector<std::vector<Solution::SolTask*>>(Solution::job_count_);
    for (unsigned int i = 0; i < Solution::job_count_; ++i)
    {
      Solution::problem_view_[i] = std::vector<Solution::SolTask*>(jobProgress[i]);
    }

  }


}