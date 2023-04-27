#include "ShuffleStep.h"

#include <Problem.h>


namespace JSOptimzer {
	
    template <typename T>
    T remove_at(std::vector<T>& v, typename std::vector<T>::size_type n)
    {
        T ans = std::move_if_noexcept(v[n]);
        v[n] = std::move_if_noexcept(v.back());
        v.pop_back();
        return ans;
    }
    


    ShuffleStep::SolutionConstructor::SolutionConstructor(std::string name, unsigned int taskCnt, unsigned int machineCnt)
        : Solution(taskCnt, machineCnt, name)
    {
        // vectors are left uninitialized
    }


    ShuffleStep::ShuffleStep(Problem* problem, Optimizer::TerminationCriteria crit, unsigned int seed, std::string namePrefix)
        : Optimizer(problem, crit) // super class constructor
    {
        unsigned int taskCnt = problem->getTaskCnt();
        unsigned int machineCnt = problem->getMachineCnt();
        
        // init the vectors
        
        m_seed = seed;
        m_prefix = namePrefix;
        m_generator = std::mt19937(seed);
        m_best = new SolutionConstructor(problem->getName(), taskCnt, machineCnt);
    }

    ShuffleStep::~ShuffleStep()
    {
        delete m_best;
        for (Solution* ptr : m_foundSolutionsList) {
            delete ptr;
        }
    }


    const Solution& ShuffleStep::runOptimizer()
    {
        return *m_best;
    }

    void ShuffleStep::initialize() const
    {
    }

    void ShuffleStep::iterate() const
    {
    }

    bool ShuffleStep::checkTermination() const
    {
        return false;
    }

}