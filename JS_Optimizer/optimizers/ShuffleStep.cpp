#include "ShuffleStep.h"

#include <Problem.h>
#include <Task.h>

#include "loguru.hpp"

#include <tuple>
#include <cmath>

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
        : Optimizer(problem, crit), m_prefix(namePrefix), m_seed(seed), m_temperature(0), m_current(nullptr)
    {
		m_generator = std::mt19937(seed);
		m_best = Solution();
		
		size_t taskCnt = problem->getTaskCnt();
		m_machineCnt = problem->getMachineCnt();
        
		m_foundSolutionsList = std::vector<ShuffleSolution*>();

        // init m_machineStepLists
		// reserve space
		const std::vector<size_t>& lengths = problem->getStepCountForMachines();
		m_machineStepLists = std::vector<std::vector<StepIdentifier>>(m_machineCnt);
		for (unsigned int i = 0; i < m_machineCnt; ++i) {
			m_machineStepLists[i] = std::vector<StepIdentifier>();
			m_machineStepLists[i].reserve(lengths[i] + 1);
		}
		// push_back the StepIdentifiers for each Step in the problem at the correct machine
		// this groups the identifiers for each task since we traverse them by task
		for (const Task& t : problem->getTasks()) {
			for (const Task::Step& s : t.getSteps()) {
				unsigned int predT = s.index == 0 ? 1 : 0;
				m_machineStepLists[s.machine].emplace_back(StepIdentifier(t.getId(), s.index, 1, predT));
			}
		}
		// init m_masterStepSets
		// setup outer vector
		m_masterStepBags = std::vector<std::vector<size_t>>(m_machineCnt);
		// find the starting index for each task within m_machineStepLists for each machine
		for (unsigned int i = 0; i < m_machineCnt; ++i) {
			m_masterStepBags[i] = std::vector<size_t>();
			m_masterStepBags[i].reserve(lengths[i] + 1);
			size_t curInd = 0;
			unsigned int curTid = m_machineStepLists[i][0].taskId;
			for (size_t j = 0; j < lengths[i]; ++j) {
				// repeat the same index until new taskId is reached (represents a bag)
				if (m_machineStepLists[i][j].taskId != curTid) {
					curInd = j;
					curTid = m_machineStepLists[i][j].taskId;
				}
				m_masterStepBags[i].push_back(curInd);
			}
		}
		///*
		std::cout << "print m_machineStepLists" << std::endl;
		for (unsigned int i = 0; i < m_machineCnt; ++i) {
			size_t length = m_machineStepLists[i].size();
			std::cout << "[";
			for (unsigned int j = 0; j < length; ++j) {
				StepIdentifier c = m_machineStepLists[i][j];
				std::cout << "(" << c.taskId << ", " << c.stepIndex << ", " << c.it << ", " <<  c.predT << "), ";
			}
			std::cout << "\b\b]" << std::endl;
		}

		std::cout << "print m_masterStepBags" << std::endl;
		for (unsigned int i = 0; i < m_machineCnt; ++i) {
			unsigned int length = m_masterStepBags[i].size();
			std::cout << "[";
			for (unsigned int j = 0; j < length; ++j) {
				std::cout << m_masterStepBags[i][j] << ", ";
			}
			std::cout << "\b\b]" << std::endl;
		}
		//*/

    }

    ShuffleStep::~ShuffleStep()
    {
		delete m_current;
		for (ShuffleSolution* ptr : m_foundSolutionsList)
			delete ptr;
    }


	// does multiple runs and offers some other options
    const Solution& ShuffleStep::runOptimizer(unsigned int num_restarts, bool logToFile)
    {
        
		return m_best;
    }


    void ShuffleStep::iterate()
    {

		std::uniform_int_distribution<> distrib(1, 6);
		int rng = distrib(m_generator);
    }


    void ShuffleStep::initialize()
    {
		// copy master machineStepBags to create a solState
		m_solState = m_masterStepBags;
		// create a random ordering for each machine, to init (i.e. random start)
		for (auto& mList : m_solState) {
			std::shuffle(mList.begin(), mList.end(), m_generator);
		}

		std::cout << "print m_solState" << std::endl;
		for (unsigned int i = 0; i < m_solState.size(); ++i) {
			unsigned int length = m_solState[i].size();
			std::cout << "[";
			for (unsigned int j = 0; j < length; ++j) {
				std::cout << m_solState[i][j] << ", ";
			}
			std::cout << "\b\b]" << std::endl;
		}

		// get first solution
		m_current = new ShuffleSolution(m_solState, *m_problem, *this);
		
		// create inital solution if there is none
		if (!m_best.isInitialized()) {
			m_best = SolutionConstructor(*m_current, *this);
		}

    }

	// returns true if termination criteria reached
    bool ShuffleStep::checkTermination()
    {
		if (m_terminationCrit.restartLimit >= 0 && m_restarts > (unsigned int)m_terminationCrit.restartLimit)
			return true;
		if (m_terminationCrit.iterationLimit >= 0 && m_totalIterations > (unsigned int)m_terminationCrit.iterationLimit)
			return true;
		if (m_terminationCrit.percentageThreshold != 0 && m_current != nullptr) {
			long lowerBound = this->m_problem->getBounds().getLowerBound();
			long threshold = (long)ceil((1.0 + m_terminationCrit.percentageThreshold) * m_current->getFitness());
			if (m_current != nullptr && threshold <= lowerBound)
				return true;
		}
		
		return false;
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
    ShuffleStep::ShuffleSolution::ShuffleSolution(const std::vector<std::vector<size_t>>& solState,
													const Problem& p, const ShuffleStep& o)
    {
		// prepare some variables for easy access
		const std::vector<Task>& pTaskL = p.getTasks();
		const std::vector<std::vector<StepIdentifier>>& oStepI = o.m_machineStepLists;
		unsigned int mCnt = o.m_machineCnt;
		unsigned int tCnt = p.getTaskCnt();
		CHECK_F(mCnt == p.getMachineCnt(), "machine cnt mismatch");
        // create and fill m_shuffleSol
		m_shuffelSol = std::vector<std::vector<ShuffleSolStep>>(mCnt);
		for (unsigned int i = 0; i < mCnt; ++i)
		{
			size_t len = solState[i].size();
			m_shuffelSol[i] = std::vector<ShuffleSolStep>(len);
			// track indices/occurences for sol to enable indexing into m_machineStepLists
			auto prog = std::vector<size_t>(tCnt, 0);
			// define the SolSteps
			for (unsigned int j = 0; j < len; ++j)
			{
				size_t baseInd = solState[i][j];
				StepIdentifier base = oStepI[i][baseInd];
				unsigned int tid = base.taskId;
				StepIdentifier step = oStepI[i][baseInd + prog[tid]];
				prog[tid]++;
				CHECK_F(tid == step.taskId, "wrong offset %i %i", tid, step.taskId);
				m_shuffelSol[i][j] = { tid, step.stepIndex, pTaskL[tid].getSteps()[step.stepIndex].duration, -1 };
			}
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
		bool complete = false;
		bool progress = false;
		auto rowEnds = std::vector<bool>(o.m_machineCnt, false);
		unsigned int rowEndCnt = 0;
		size_t colInd = 0;
		// (index, endtime) pairs to check if a SolStep is the next and what the bound is (index refers to the next index)
		auto taskPredEndT = std::vector<std::pair<size_t, long>>(p.getTaskCnt(), std::make_pair(0,0));
		// iterate column by column and cascade times
		while (!complete)
		{
			progress = false;
			// cascade endtimes
			for (unsigned int i = 0; i < o.m_machineCnt; ++i)
			{
				if (!rowEnds[i]) {
					if (colInd < m_shuffelSol[i].size()) {
						// access colInd column for the machines that still have steps
						ShuffleSolStep& s = m_shuffelSol[i][colInd];
						auto& [pInd, pEndT] = taskPredEndT[s.taskId];
						if (s.stepIndex == pInd)
						{
							if (colInd == 0) {
								s.endTime = s.duration;
								pEndT = s.endTime;
								pInd++;
								progress = true;
							}
							else if (m_shuffelSol[i][colInd - 1].endTime != -1) {
								long predEndTime = std::max(pEndT, m_shuffelSol[i][colInd - 1].endTime);
								s.endTime = predEndTime + s.duration;
								pEndT = s.endTime;
								pInd++;
								progress = true;
							}
						}
					}
					else {
						rowEndCnt++;
						rowEnds[i] = true;
					}
				}
			}
			// go to next col
			colInd++;
			// restart if no progress can be made
			if (rowEndCnt >= o.m_machineCnt || progress == false) {
				rowEnds = std::vector<bool>(o.m_machineCnt, false);
				colInd = 0;
			}
			// check if complete
			complete = true;
			for (unsigned int i = 0; i < o.m_machineCnt; ++i) {
				if (complete && m_shuffelSol[i][m_shuffelSol[i].size() - 1].endTime == -1) {
					complete = false;
				}
			}
		}
        // set completion time
		m_completetionTime = -1;
		for (unsigned int i = 0; i < o.m_machineCnt; ++i) {
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
		m_machineCnt = (unsigned int)o.m_machineCnt;
		m_name = o.m_prefix + "ShuffleStep_On_" + o.m_problem->getName();
		m_completionTime = sol.getFitness();
		//build m_solution
		const std::vector<std::vector<ssSolStep>>& ssSolSteps = sol.m_shuffelSol;
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
			m_problemRep[i] = std::vector<SolStep*>(taskVec[i].getSteps().size());
		}
		// fill problemRep
		fillProblemRep();
    }



}