#include "ShuffleStep.h"

#include <Problem.h>
#include <Task.h>

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
        : Optimizer(problem, crit) // super class constructor
    {
        size_t taskCnt = problem->getTaskCnt();
        size_t machineCnt = problem->getMachineCnt();
        
        // init m_machineStepLists
		// reserve space
		m_machineStepLists = std::vector<std::vector<StepIdentifier>>(machineCnt);
		for (unsigned int i = 0; i < machineCnt; ++i) {
			m_machineStepLists[i] = std::vector<StepIdentifier>();
			m_machineStepLists[i].reserve(problem->getStepCountForMachines()[i] + 1);
		}
		// push_back the StepIdentifiers for each Step in the problem at the correct machine
		for (const Task& t : problem->getTasks()) {
			for (const Task::Step& s : t.getSteps()) {
				m_machineStepLists[s.machine].push_back({0, t.getId(), s.index});
			}
		}
		// group the identifiers for each task, add eof element
		for (std::vector<StepIdentifier> vec : m_machineStepLists) {
			std::sort(vec.begin(), vec.end());
			vec.push_back({ -1, 0, 0 });
		}

		// init m_masterStepSets
		// find the starting index for each task within m_machineStepLists for each machine




		//std::mt19937 rnggen(seed); // Standard mersenne_twister_engine
		//std::uniform_int_distribution<> distrib(1, 6);
		std::uniform_int_distribution<> distrib(1, 6);
        
        m_seed = seed;
        m_prefix = namePrefix;
        m_generator = std::mt19937(seed);
        m_best = Solution();
    }

    ShuffleStep::~ShuffleStep()
    {
        
    }


    const Solution& ShuffleStep::runOptimizer()
    {
        return m_best;
    }

    void ShuffleStep::initialize()
    {
    }

    void ShuffleStep::iterate()
    {
    }

    bool ShuffleStep::checkTermination()
    {
        return false;
    }


    ShuffleStep::ShuffleSolution::ShuffleSolution(const Problem& p, const std::vector<std::vector<StepIdentifier*>>& solState)
    {
        // fill m_shuffleSol

        // set completion time
		m_completetionTime = -1;
    }

    long ShuffleStep::ShuffleSolution::getFitness()
    {
        return m_completetionTime;
    }

    ShuffleStep::SolutionConstructor::SolutionConstructor(const ShuffleSolution& sol)
    {

    }



}