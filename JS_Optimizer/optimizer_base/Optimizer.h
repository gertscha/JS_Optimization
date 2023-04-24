#pragma once


namespace JSOptimzer {
	class Problem;
	class Solution;

	/*
	* This is the base class for Optimizers
	* provides an implementation of the run function
	*/
	class Optimizer
	{
	public:
		
		struct TerminationCriteria {
			int iterationLimit;
			double percentageThreshold;
		};
		
		Optimizer(Problem* problem, const TerminationCriteria& criteria, unsigned int restartCnt)
			: m_problem(problem), m_terminationCrit(criteria), m_restarts(restartCnt)
		{}

		/*
		* calls initalize once and then iterate until a termination criteria is reached
		* repeats this restarts times
		*/
		virtual void run()
		{
			unsigned int it = 0;
			do {
				initialize();
				while (checkTermination()) {
					iterate();
				}
			} while (it < m_restarts);
		}
		
		// initializes an optimization run
		virtual void initialize() = 0;

		// performs an optimization iteration
		virtual void iterate() = 0;

		// returns true if termination criteria reached
		virtual bool checkTermination() = 0;


		// best solution knwon
		virtual const Solution& getBestSolution() = 0;

		const Problem& getProblem() const { return *m_problem; }
		const TerminationCriteria& getTerminationCriteria() const { return m_terminationCrit; }
		unsigned int getRestartCount() const { return m_restarts; }
		
	protected:
		Problem* m_problem;
		const TerminationCriteria m_terminationCrit;
		unsigned int m_restarts;
	};

}
