#include "ShuffleStep.h"

#include <Problem.h>
#include <Task.h>

#include "loguru.hpp"

#include <tuple>
#include <cmath>
#include <algorithm>

namespace JSOptimzer {

	template <typename T>
    T remove_at(std::vector<T>& v, typename std::vector<T>::size_type n)
    {
        T ans = std::move_if_noexcept(v[n]);
        v[n] = std::move_if_noexcept(v.back());
        v.pop_back();
        return ans;
    }


    ShuffleStep::ShuffleStep(Problem* problem, Optimizer::TerminationCriteria& crit, unsigned int seed, std::string namePrefix)
        : Optimizer(problem, crit), m_prefix(namePrefix), m_seed(seed), m_temperature(0), m_totalIterations(0), m_bestInternal(nullptr)
    {
		using ssSol = ShuffleStep::ShuffleSolution;
		
		m_bestSolution = Solution();
		m_generator = std::mt19937(seed);
		m_solStates = std::vector<std::vector<unsigned int>>();
		m_solutionsHeap = Utility::Heap<ShuffleSolPtr>();

		m_stepCount = 0;
		for (const Task& t : problem->getTasks()) {
			m_stepCount += t.size();
		}
		m_seqExec = std::vector<unsigned int>();
		m_seqExec.reserve(m_stepCount);
		for (const Task& t : problem->getTasks()) {
			for (const Task::Step& s : t.getSteps()) {
				m_seqExec.push_back(s.taskId);
			}
		}

    }

    ShuffleStep::~ShuffleStep()
    {
		delete m_bestInternal;
		for (ShuffleSolPtr wrap : m_solutionsHeap.getElements())
			delete wrap.ptr;
    }


	// does multiple runs and offers some other options
    const Solution& ShuffleStep::runOptimizer(unsigned int num_restarts, bool logToFile)
    {
		return getBestSolution();
    }


    void ShuffleStep::iterate()
    {

		std::uniform_int_distribution<> distrib(1, 6);
		int rng = distrib(m_generator);
    }


    void ShuffleStep::initialize()
    {
		// if this is a restart, save the previous solState
		if (!m_curSolState.empty()) {

		}
		
		// copy master seqentialExec to create a solState
		m_curSolState = m_seqExec;
		// create a random ordering for each machine, to init (i.e. random start)
		std::shuffle(m_curSolState.begin(), m_curSolState.end(), m_generator);

		// make first solution
		ShuffleSolution* ssSolptr = new ShuffleSolution(m_curSolState, *m_problem);
		
		// create inital solution if there is none
		if (!m_bestSolution.isInitialized()) {
			m_bestSolution = SolutionConstructor(*ssSolptr, *this);
		}

    }

	// returns true if termination criteria reached
    bool ShuffleStep::checkTermination()
    {
		if (m_terminationCrit.restartLimit >= 0 && m_restarts > (unsigned int)m_terminationCrit.restartLimit)
			return true;
		if (m_terminationCrit.iterationLimit >= 0 && m_totalIterations > (unsigned int)m_terminationCrit.iterationLimit)
			return true;
		if (m_terminationCrit.percentageThreshold != 0 && m_bestInternal != nullptr) {
			long lowerBound = this->m_problem->getBounds().getLowerBound();
			long threshold = (long)ceil((1.0 + m_terminationCrit.percentageThreshold) * m_bestInternal->getFitness());
			if (threshold <= lowerBound)
				return true;
		}
		
		return false;
    }

	// return best, update best if internalbest is better
	const Solution& ShuffleStep::getBestSolution()
	{
		if (m_bestInternal != nullptr) {
			if (!(m_bestSolution.isInitialized()))
				m_bestSolution = SolutionConstructor(*m_bestInternal, *this);
			else if (m_bestInternal->getFitness() < m_bestSolution.getCompletetionTime())
				m_bestSolution = SolutionConstructor(*m_bestInternal, *this);
		}
		return m_bestSolution;
	}

	// does single run
	void ShuffleStep::run()
	{
		m_restarts++;
		initialize();
		while (!checkTermination()) {
			iterate();
		}
	}

	/*
	* build internal solution
	*/
    ShuffleStep::ShuffleSolution::ShuffleSolution(const std::vector<unsigned int>& sol, const Problem& p)
    {
		// prepare some variables for easy access
		const std::vector<Task>& pTaskL = p.getTasks();
		const std::vector<size_t>& machineStepCnts = p.getStepCountForMachines();
		unsigned int mCnt = p.getMachineCnt();
		unsigned int tCnt = p.getTaskCnt();
        // create and reserve memory, m_shuffleSol
		m_shuffelSol = std::vector<std::vector<ShuffleSolStep>>(mCnt);
		for (unsigned int i = 0; i < mCnt; ++i) {
			m_shuffelSol[i] = std::vector<ShuffleSolStep>();
			m_shuffelSol[i].reserve(machineStepCnts[i]);
		}
		// fill m_shuffelSol
		// track the current index for each task
		auto taskProgress = std::vector<size_t>(tCnt, 0);
		for (unsigned int i = 0; i < sol.size(); ++i)
		{
			unsigned int tid = sol[i];
			const Task& t = pTaskL[tid];
			const Task::Step& s = t.getSteps()[taskProgress[tid]];
			taskProgress[tid]++;
			m_shuffelSol[s.machine].emplace_back(ShuffleSolStep(tid, s.index, s.duration, -1));
		}

		std::cout << "print partial m_shuffelSol" << std::endl;
		for (unsigned int i = 0; i < m_shuffelSol.size(); ++i) {
			size_t length = m_shuffelSol[i].size();
			std::cout << "[";
			for (unsigned int j = 0; j < length; ++j) {
				ShuffleSolStep c = m_shuffelSol[i][j];
				std::cout << "(" << c.taskId << ", " << c.stepIndex << ", " << c.duration << "), ";
			}
			std::cout << "\b\b]" << std::endl;
		}

		// determine endtimes
		// track progress, avoid endless loop for malformed solution
		bool timingProgress = false;
		// (index, endtime) pairs to check if a SolStep is the next and what the bound is (index refers to the next index)
		auto taskPredEndT = std::vector<std::pair<size_t, long>>(p.getTaskCnt(), std::make_pair(0,0));
		// tracks next index that has no endtime for each machine
		auto machineProgress = std::vector<size_t>(mCnt, 0);
		// mark if a machine has no more steps that are missing the endTime
		auto rowDone = std::vector<bool>(mCnt, false);
		unsigned int rowDoneCnt = 0;
		// iterate and cascade times
		while (rowDoneCnt != mCnt)
		{
			// cascade endtimes, for each machine
			for (unsigned int i = 0; i < mCnt; ++i)
			{
				if (!rowDone[i])
				{
					// check if row is done
					if (machineProgress[i] >= m_shuffelSol[i].size()) {
						rowDoneCnt++;
						rowDone[i] = true;
						continue;
					}
					// get current step for the machine
					ShuffleSolStep& s = m_shuffelSol[i][machineProgress[i]];
					auto& [pInd, pEndT] = taskPredEndT[s.taskId];
					// if this step is next (i.e. predecessor is done)
					if (s.stepIndex == pInd) {
						// if it has no predecessor on the machine, only its task predecessor is relevant
						if (machineProgress[i] == 0) {
							s.endTime = pEndT + s.duration;
						}
						// has a predecessor on machine and (maybe) on task (pEndT has the endTime or 0 if first step of a task)
						else {
							long predEndTime = std::max(pEndT, m_shuffelSol[i][machineProgress[i] - 1].endTime);
							s.endTime = predEndTime + s.duration;
						}
						pEndT = s.endTime;
						pInd++;
						timingProgress = true;
					}
				}
			}
		}
        // set completion time
		m_completetionTime = -1;
		for (unsigned int i = 0; i < tCnt; ++i) {
			long endT = m_shuffelSol[i][m_shuffelSol[i].size() - 1].endTime;
			if (endT > m_completetionTime)
				m_completetionTime = endT;
		}
    }


	/*
	* Build general Solution
	*/
    ShuffleStep::SolutionConstructor::SolutionConstructor(const ShuffleSolution& sol, const ShuffleStep& o)
    {
		using ssSolStep = ShuffleStep::ShuffleSolution::ShuffleSolStep;
		// set other members
		m_initalized = true;
		m_taskCnt = (unsigned int)o.m_problem->getTaskCnt();
		m_machineCnt = (unsigned int)o.m_problem->getMachineCnt();
		m_name = o.m_prefix + "ShuffleStep_On_" + o.m_problem->getName();
		m_completionTime = sol.getFitness();
		//build m_solution
		const std::vector<std::vector<ssSolStep>>& ssSolSteps = sol.getSolSteps();
		size_t outerLen = ssSolSteps.size();
		m_solution = std::vector<std::vector<SolStep>>(outerLen);
		for (unsigned int i = 0; i < outerLen; ++i)
		{
			size_t len = ssSolSteps[i].size();
			m_solution[i] = std::vector<SolStep>(len);
			for (unsigned int j = 0; j < len; ++j) {
				ssSolStep s = ssSolSteps[i][j];
				long startTime = s.endTime - s.duration;
				m_solution[i][j] = SolStep{ s.taskId, s.stepIndex, i, startTime, s.endTime };
			}
		}
		// init the problemRep vectors to correct size
		const std::vector<Task>& taskVec = o.m_problem->getTasks();
		m_problemRep = std::vector<std::vector<SolStep*>>(m_taskCnt);
		for (unsigned int i = 0; i < m_taskCnt; ++i) {
			m_problemRep[i] = std::vector<SolStep*>(taskVec[i].size());
		}
		// fill problemRep
		fillProblemRep();
    }



}