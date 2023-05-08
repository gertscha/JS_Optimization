#include "ShuffleStep.h"

#include <Problem.h>
#include <Task.h>

#include "loguru.hpp"

#include <tuple>
//#include <cmath>
#include <algorithm>

namespace JSOptimzer {


    ShuffleStep::ShuffleStep(Problem* problem, Optimizer::TerminationCriteria& crit, unsigned int seed, std::string namePrefix)
        : Optimizer(problem, crit), m_prefix(namePrefix), m_seed(seed), m_temperature(0), m_totalIterations(0), m_internalSolMaxCnt(3)
    {
		using ssSolWrapper = Utility::Wrapper<ShuffleStep::ShuffleSolution>;
		
		m_bestSolution = Solution();
		m_generator = std::mt19937(seed);
		m_solStates = std::vector<std::vector<unsigned int>>();
		m_solutionsHeap = Utility::Heap<ssSolWrapper>();

		m_seqExec = std::vector<unsigned int>();
		m_stepCount = 0;
		for (const Task& t : problem->getTasks()) {
			m_stepCount += t.size();
			m_seqExec.insert(m_seqExec.end(), t.size(), t.getId());
		}

		m_bestInternal = new ShuffleSolution(m_seqExec, *m_problem);
		m_solutionsHeap.add(ssSolWrapper(m_bestInternal));
    }

    ShuffleStep::~ShuffleStep()
    {
		for (auto& wrap : m_solutionsHeap.getElements())
			delete wrap.ptr;
    }


	// does multiple runs and offers some other options
    const Solution& ShuffleStep::runOptimizer(unsigned int num_restarts, unsigned int num_sols, bool logToFile)
    {
		using ssSolWrapper = Utility::Wrapper<ShuffleStep::ShuffleSolution>;

		m_internalSolMaxCnt = num_sols;

		for (unsigned int round = 0; round < num_restarts; ++round) {

			m_temperature = 100;
			initialize();
			//while (!checkTermination()) {
			//	iterate();
			//}

			m_restarts++;
		}

		return getBestSolution();
    }


    void ShuffleStep::iterate()
    {
		using ssSolWrapper = Utility::Wrapper<ShuffleStep::ShuffleSolution>;

		std::uniform_real_distribution distrib(0.0, 1.0);
		//double rng = distrib(m_generator);
		
		// random shuffle
		unsigned int num_splits = 3;
		// split into ranges, and shuffle each, get multiple options


		// guild solutons
		ShuffleSolution* newSol1 = new ShuffleSolution(m_curSolState, *m_problem);

		long cost1 = m_bestInternal->getFitness() - newSol1->getFitness();


		// take if better or with probability dependent on temp
		if (cost1 < 0) {
			// newSol is worse
			double exponential = exp(-((double)cost1) / m_temperature);
			if (distrib(m_generator) < exponential) {

			}
		}



		// decrease temp


		m_totalIterations++;

		// make sure everything is saved as termination criteria could be hit

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

    void ShuffleStep::initialize()
    {		
		using ssSolWrapper = Utility::Wrapper<ShuffleStep::ShuffleSolution>;
		
		// copy master seqentialExec to create a solState
		m_curSolState = m_seqExec;
		// create a random ordering to init (i.e. random start)
		std::shuffle(m_curSolState.begin(), m_curSolState.end(), m_generator);

		// make a internal solution
		ShuffleSolution* ssSolptr = new ShuffleSolution(m_curSolState, *m_problem);
		
		// create inital solution if there is none
		if (!m_bestSolution.isInitialized()) {
			m_bestSolution = SolutionConstructor(*ssSolptr, *this);
		}
		// update internal state with the new solution
		updateInternalSols(ssSolptr, m_internalSolMaxCnt);

    }

	void ShuffleStep::updateInternalSols(ShuffleSolution* solptr, unsigned int limit)
	{
		using ssSolWrapper = Utility::Wrapper<ShuffleStep::ShuffleSolution>;

		if (m_bestInternal->getFitness() > solptr->getFitness()) {
			m_bestInternal = solptr;
		}

		size_t numElems = m_solutionsHeap.size();
		if (numElems > limit) {
			ssSolWrapper discard = m_solutionsHeap.replace(ssSolWrapper(solptr));
			delete discard.ptr;
		}
		else {
			m_solutionsHeap.add(ssSolWrapper(solptr));
		}
	}

	// return best, updates best if internalbest is better
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

	void ShuffleStep::saveToFileAllStoredSolutions(const std::string& filepath, const std::string& filenamePrefix)
	{
		using ssSolWrapper = Utility::Wrapper<ShuffleStep::ShuffleSolution>;

		std::vector<ssSolWrapper> heapCopy = m_solutionsHeap.getElements();
		std::sort(heapCopy.begin(), heapCopy.end()); // uses operator< from the wrapper
		unsigned int rank = 1;
		for (ssSolWrapper wrap : heapCopy) {
			std::string solname = filenamePrefix + "Rank" + std::to_string(rank) + ".txt";
			SolutionConstructor(*wrap.ptr, *this).saveToFile(filepath, solname);
			rank++;
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
		size_t mCnt = p.getMachineCnt();
		size_t tCnt = p.getTaskCnt();
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
			// create shuffelSolStep, endTime set to uninitalized (i.e. -1)
			m_shuffelSol[s.machine].emplace_back(ShuffleSolStep(tid, s.index, s.duration, -1));
		}
		// determine endtimes
		// (index, endtime) pairs to check if a SolStep is the next and what the bound is (index refers to the next index)
		auto preceedingTask = std::vector<std::pair<size_t, long>>(p.getTaskCnt(), std::make_pair(0,0));
		// tracks next index that has no endtime for each machine
		auto machineProgress = std::vector<size_t>(mCnt, 0);
		// track progress, avoid endless loop for malformed solution
		bool timingProgress = false; // overall progress
		auto rowDone = std::vector<bool>(mCnt, false);
		unsigned int rowDoneCnt = 0;
		// iterate and cascade times
		while (rowDoneCnt != mCnt) {
			// cascade endtimes, for each machine
			timingProgress = false;
			for (unsigned int i = 0; i < mCnt; ++i) {
				if (!rowDone[i]) {
					// check if row is done
					if (machineProgress[i] >= m_shuffelSol[i].size()) {
						rowDoneCnt++;
						rowDone[i] = true;
						continue;
					}
					// get current step for the machine
					ShuffleSolStep& s = m_shuffelSol[i][machineProgress[i]];
					// bind the (index, endTime) pair to references for easy access
					auto& [pInd, pEndT] = preceedingTask[s.taskId];
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
						machineProgress[i]++;
						pEndT = s.endTime;
						pInd++;
						timingProgress = true;
					}
				}
			}
			if (!timingProgress)
				break;
		}
		if (rowDoneCnt != mCnt)
			ABORT_F("ShuffleSolution constructor failed, was the input invalid?");
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