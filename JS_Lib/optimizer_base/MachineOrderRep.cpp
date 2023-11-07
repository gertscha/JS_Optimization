#include "MachineOrderRep.h"

#include <iostream>

#include "loguru.hpp"

#include "Task.h"


namespace JSOptimizer {

  MachineOrderRep::MachineClique::MachineClique(unsigned int machineId, unsigned int taskCnt)
    : machine_(machineId)
  {
    machine_order_ = std::vector<unsigned int>();
    clique_members_ = std::set<size_t>();
    vertex_map_ = std::vector<std::vector<size_t>>(taskCnt);
    for (unsigned int i = 0; i < taskCnt; ++i) {
      vertex_map_[i] = std::vector<size_t>();
    }
  }


  MachineOrderRep::MachineOrderRep(Problem* problem, const TerminationCriteria& crit,
    std::string prefix, unsigned int seed)
    : Optimizer(problem, crit, prefix, seed)
  {
    m_count_ = problem->getMachineCount();
    task_count_ = 0;
    for (const Job& t : problem->getJobs()) {
      task_count_ += t.size();
    }
    cliques_ = std::vector<MachineClique>();
    cliques_.reserve(m_count_);
    for (unsigned int i = 0; i < m_count_; ++i) {
      cliques_.emplace_back(MachineClique(i, problem->getJobCount()));
    }
    task_map_ = std::vector<Identifier>();
    task_map_.reserve(task_count_);

    size_t task_id = 0;
    for (const Job& t : problem->getJobs()) {
      unsigned int jid = t.getId();
      for (const Job::Task& s : t.getTasks()) {
        task_map_.emplace_back(Identifier(jid, s.index));
        MachineClique& s_clique = cliques_[s.machine];
        s_clique.clique_members_.insert(task_id);
        s_clique.machine_order_.push_back(jid);
        s_clique.vertex_map_[jid].push_back(task_id);
        ++task_id;
      }
    }

  }


  MachineOrderRep::SolutionConstructor::SolutionConstructor(const std::vector<MachineClique>& cliques,
      const std::vector<Identifier>& map, const Problem* const problem, const std::string& prefix)
  {
    // init members
    Solution::job_count_ = problem->getJobCount();
    Solution::machine_count_ = problem->getMachineCount();
    Solution::name_ = prefix + problem->getName();
    Solution::initialized_ = true;
    Solution::makespan_ = 0;

    // setup solution matrix, contains uninitalized Steps
    Solution::solution_ = std::vector<std::vector<Solution::SolStep>>(machine_count_);
    const auto& machine_task_counts = problem->getTaskCountForMachines();
    for (unsigned int i = 0; i < machine_count_; ++i) {
      solution_[i] = std::vector<Solution::SolStep>();
      solution_[i].reserve(machine_task_counts[i]);
    }

    // fill internal_sol_steps_ with the tasks,step and duration information
    // track the current index for each task
    for (unsigned int i = 0; i < machine_count_; ++i)
    {
      unsigned int machine = cliques[i].getMachine();
      std::vector<std::vector<size_t>> v_map = cliques[i].getVertexMap();
      auto jobProgress = std::vector<size_t>(job_count_, 0);
      for (unsigned int jid : cliques[i].getMachineOrder()) {
        const Identifier& iden = map[v_map[jid][jobProgress[jid]]];
        ++jobProgress[jid];
        // create SolStep, times set to uninitalized (i.e. -1)
        solution_[machine].emplace_back(Solution::SolStep(jid, iden.index, machine, -1, -1));
      }
      //std::cout << "Machine " << machine << ": ";
        //std::cout << "(" << iden.job_id << ", " << iden.index << ") ";
      //std::cout << "\n";
    }

    // determine timings, allow for invalid schedule to be checked
    if (!CalculateTimings(*problem)) {
      initialized_ = false;
      makespan_ = -1;
      return;
    }

    // init the problemRep vectors to correct size (filling happens during first validate call)
    Solution::problem_view_ = std::vector<std::vector<Solution::SolStep*>>(Solution::job_count_);
    for (unsigned int i = 0; i < Solution::job_count_; ++i) {
      Solution::problem_view_[i] = std::vector<Solution::SolStep*>(problem->getJobs()[i].size());
    }

  }


}