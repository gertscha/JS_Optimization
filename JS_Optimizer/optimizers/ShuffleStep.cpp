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


    ShuffleStep::ShuffleStep(Problem* problem, Optimizer::TerminationCriteria crit, unsigned int seed, std::string namePrefix)
        : Optimizer(problem, crit) // super class constructor
    {
        size_t taskCnt = problem->getTaskCnt();
        size_t machineCnt = problem->getMachineCnt();
        
        // init the vectors
        
        m_seed = seed;
        m_prefix = namePrefix;
        m_generator = std::mt19937(seed);
        m_best = new Solution();
    }

    ShuffleStep::~ShuffleStep()
    {
        delete m_best;
        
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


    ShuffleStep::ShuffleSolution::ShuffleSolution(const Problem& p, const std::vector<std::vector<machineStep*>>& solState)
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