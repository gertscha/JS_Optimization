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
    step_count_ = 0;
    for (const Task& t : problem->getTasks()) {
      step_count_ += t.size();
    }
    cliques_ = std::vector<MachineClique>();
    cliques_.reserve(m_count_);
    for (unsigned int i = 0; i < m_count_; ++i) {
      cliques_.emplace_back(MachineClique(i, problem->getTaskCount()));
    }
    step_map_ = std::vector<Identifier>();
    step_map_.reserve(step_count_);

    size_t step_id = 0;
    for (const Task& t : problem->getTasks()) {
      unsigned int tid = t.getId();
      for (const Task::Step& s : t.getSteps()) {
        step_map_.emplace_back(Identifier(tid, s.index));
        MachineClique& s_clique = cliques_[s.machine];
        s_clique.clique_members_.insert(step_id);
        s_clique.machine_order_.push_back(tid);
        s_clique.vertex_map_[tid].push_back(step_id);
        ++step_id;
      }
    }

  }


  MachineOrderRep::SolutionConstructor::SolutionConstructor(const std::vector<MachineClique>& cliques,
      const std::vector<Identifier>& map, const Problem* const problem, const std::string& prefix)
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
    for (unsigned int i = 0; i < machine_count_; ++i)
    {
      unsigned int machine = cliques[i].getMachine();
      std::vector<std::vector<size_t>> v_map = cliques[i].getVertexMap();
      auto taskProgress = std::vector<size_t>(task_count_, 0);
      for (unsigned int tid : cliques[i].getMachineOrder()) {
        const Identifier& iden = map[v_map[tid][taskProgress[tid]]];
        ++taskProgress[tid];
        // create SolStep, times set to uninitalized (i.e. -1)
        solution_[machine].emplace_back(Solution::Step(tid, iden.index, machine, -1, -1));
      }
      //std::cout << "Machine " << machine << ": ";
        //std::cout << "(" << iden.task_id << ", " << iden.index << ") ";
      //std::cout << "\n";
    }

    // determine timings, allow for invalid schedule to be checked
    try {
      calculateTimings(*problem);
    }
    catch (std::runtime_error e) {
      std::string what = e.what();
      std::string expected = "Solution::calculateTimings(): failed to complete";
      int cmp_res = what.compare(expected);
      if (cmp_res == 0) {
        initalized_ = false;
        makespan_ = -1;
        return;
      }
      else {
        throw std::runtime_error(what);
      }
    }

    // init the problemRep vectors to correct size (filling happens during first validate call)
    Solution::problem_view_ = std::vector<std::vector<Solution::Step*>>(Solution::task_count_);
    for (unsigned int i = 0; i < Solution::task_count_; ++i) {
      Solution::problem_view_[i] = std::vector<Solution::Step*>(problem->getTasks()[i].size());
    }

  }


}