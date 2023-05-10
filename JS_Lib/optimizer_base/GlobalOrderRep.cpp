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


  GlobalOrderRep::InternalSolution::InternalSolution(const std::vector<unsigned int>& sol, const Problem& p)
  {
    // prepare some variables for easy access
    const std::vector<Task>& problem_tasks = p.getTasks();
    const std::vector<size_t>& machine_step_counts = p.getStepCountForMachines();
    InternalSolution::num_tasks_ = p.getTaskCount();
    InternalSolution::num_machines_ = p.getMachineCount();
    InternalSolution::problem_name_ = p.getName();
    // create and reserve memory
    internal_sol_steps_ = std::vector<std::vector<InternalSolution::Step>>(num_machines_);
    for (unsigned int i = 0; i < num_machines_; ++i) {
      internal_sol_steps_[i] = std::vector<InternalSolution::Step>();
      internal_sol_steps_[i].reserve(machine_step_counts[i]);
    }
    // fill internal_sol_steps_ with the tasks,step and duration information
    // track the current index for each task
    auto taskProgress = std::vector<size_t>(num_tasks_, 0);
    for (unsigned int i = 0; i < sol.size(); ++i)
    {
      unsigned int tid = sol[i];
      const Task& t = problem_tasks[tid];
      const Task::Step& s = t.getSteps()[taskProgress[tid]];
      taskProgress[tid]++;
      // create shuffelSolStep, end_time set to uninitalized (i.e. -1)
      internal_sol_steps_[s.machine].emplace_back(InternalSolution::Step(tid, s.index, s.duration, -1));
    }
    // determine endtimes
    // (index, endtime) pairs to check if a SolStep is the next and what the bound is (index refers to the next index)
    auto task_progress_info = std::vector<std::pair<size_t, long>>(num_tasks_, std::make_pair(0, 0));
    // tracks next index that has no endtime for each machine
    auto machine_progress_info = std::vector<size_t>(num_machines_, 0);
    // track progress, avoid endless loop for malformed solution
    bool overall_progress = false; // tracks if overall progress is made
    auto row_done = std::vector<bool>(num_machines_, false);
    unsigned int row_done_count = 0;
    // iterate and cascade times
    while (row_done_count != num_machines_) {
      // cascade endtimes, for each machine
      overall_progress = false;
      for (unsigned int i = 0; i < num_machines_; ++i) {
        if (!row_done[i]) {
          // check if row is done
          if (machine_progress_info[i] >= internal_sol_steps_[i].size()) {
            row_done_count++;
            row_done[i] = true;
            continue;
          }
          // get current step for the machine
          InternalSolution::Step& s = internal_sol_steps_[i][machine_progress_info[i]];
          // bind the (index, endTime) pair to references for easy access
          auto& [pInd, pEndT] = task_progress_info[s.task_id];
          // if this step is next (i.e. predecessor is done)
          if (s.step_index == pInd) {
            // if it has no predecessor on the machine, only its task predecessor is relevant
            if (machine_progress_info[i] == 0) {
              s.end_time = pEndT + s.duration;
            }
            // has a predecessor on machine and (maybe) on task (pEndT has the endTime or 0 if first step of a task)
            else {
              long predEndTime = std::max(pEndT, internal_sol_steps_[i][machine_progress_info[i] - 1].end_time);
              s.end_time = predEndTime + s.duration;
            }
            machine_progress_info[i]++;
            pEndT = s.end_time;
            pInd++;
            overall_progress = true;
          }
        }
      }
      if (!overall_progress)
        break;
    }
    if (row_done_count != num_machines_)
      ABORT_F("GlobalOrderRep::InternalSolution constructor failed, was the input invalid?");
    // set completion time
    makespan_ = -1;
    for (unsigned int i = 0; i < num_machines_; ++i) {
      long endT = internal_sol_steps_[i].back().end_time;
      if (endT > makespan_)
        makespan_ = endT;
    }
  }


  GlobalOrderRep::SolutionConstructor::SolutionConstructor(const InternalSolution& sol, const std::string& prefix)
  {
    using IntSolStep = GlobalOrderRep::InternalSolution::Step;
    // set other members
    Solution::initalized_ = true;
    Solution::task_count_ = sol.getTaskCount();
    Solution::machine_count_ = sol.getMachineCount();
    std::string problem_name = sol.getProblemName();
    Solution::name_ = prefix + problem_name;
    Solution::makespan_ = sol.getMakespan();
    // prepare variables
    const std::vector<std::vector<IntSolStep>>& ssSolSteps = sol.getSteps();
    size_t outerLen = ssSolSteps.size();
    Solution::solution_ = std::vector<std::vector<Solution::Step>>(outerLen);
    // track step count per task
    auto task_lengths = std::vector<size_t>(Solution::task_count_, 0);
    //build solution
    for (unsigned int i = 0; i < outerLen; ++i)
    {
      size_t len = ssSolSteps[i].size();
      Solution::solution_[i] = std::vector<Solution::Step>(len);
      for (unsigned int j = 0; j < len; ++j) {
        IntSolStep s = ssSolSteps[i][j];
        long start_time = s.end_time - s.duration;
        ++task_lengths[s.task_id];
        Solution::solution_[i][j] = Solution::Step(s.task_id, s.step_index, i, start_time, s.end_time);
      }
    }
    // init the problemRep vectors to correct size (filling happens during first validate call)
    Solution::problem_view_ = std::vector<std::vector<Solution::Step*>>(Solution::task_count_);
    for (unsigned int i = 0; i < Solution::task_count_; ++i) {
      Solution::problem_view_[i] = std::vector<Solution::Step*>(task_lengths[i]);
    }

  }


}