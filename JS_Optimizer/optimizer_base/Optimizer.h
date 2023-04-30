#pragma once

#include <iostream>

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
		
		/*
		* defining -1 for the limits ignores them
		* if the found solution is within percentageThreshold of the lower bound, terminate
		* disable percentageThreshold by setting it 0
		*/
		struct TerminationCriteria {
			long iterationLimit;
			long restartLimit;
			double percentageThreshold;
		};
		
		// takes ownership of TerminationCriteria
		Optimizer(Problem* problem, TerminationCriteria& criteria)
			: m_problem(problem), m_terminationCrit(criteria)
		{}

		virtual ~Optimizer() {}

		/*
		* calls initalize once and then iterate until a termination criteria is reached
		* tracks how many times this has been done
		*/
		virtual void run()
		{
			m_restarts++;
			initialize();
			while (checkTermination()) {
				iterate();
			}
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
		TerminationCriteria& m_terminationCrit;
		unsigned int m_restarts = 0;
	};

}
