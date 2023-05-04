#pragma once

#include "Optimizer.h"
#include "Solution.h"

#include <string>
#include <vector>
#include <random>
#include <functional>
#include <queue>

namespace JSOptimzer {

	class Problem;

	/*
	* This optimizer uses a list (length total number of Steps) as search space
	* entries in the list are task ids, which are then constructed into the exec order
	* per machine
	*/
	class ShuffleStep : public Optimizer
	{
	public:

		ShuffleStep(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix);

		~ShuffleStep();

		// do multiple runs according to parameters, returns best solution
		const Solution& runOptimizer(unsigned int num_restarts, bool logToFile);

		// does a single run
		virtual void run();
		
		// initializes an optimization run
		virtual void initialize();

		// performs an optimization iteration
		virtual void iterate();

		// returns true if termination criteria reached
		virtual bool checkTermination();
		
		virtual const Solution& getBestSolution() { return m_best; }

	private:

		// internal solution representation for this optimizer
		class ShuffleSolution
		{
			friend class SolutionConstructor;
		public:
			struct ShuffleSolStep {
				unsigned int taskId;
				size_t stepIndex;
				unsigned int duration;
				long endTime;

				// has constructor to allow efficient creation with emplace_back on vectors
				ShuffleSolStep(unsigned int taskId, size_t stepIndex, unsigned int duration, long endTime)
					:taskId(taskId), stepIndex(stepIndex), duration(duration), endTime(endTime) {}
			};

			ShuffleSolution(const std::vector<unsigned int>& solState, const Problem& problem);

			long getFitness() const { return m_completetionTime; }
			const std::vector<std::vector<ShuffleSolStep>>& getSolSteps() const { return m_shuffelSol; }

		private:
			// rows correspond to machines, columns to steps, in order
			std::vector<std::vector<ShuffleSolStep>> m_shuffelSol;
			
			// the fitness value of this solution
			long m_completetionTime;
		};

		std::string m_prefix;
		int m_temperature;
		unsigned int m_totalIterations;
		size_t m_stepCount;

		unsigned int m_seed;
		std::mt19937 m_generator;

		// representation of the sequential solution to the problem
		std::vector<unsigned int> m_seqExec;

		// representation of the current solution state
		std::vector<unsigned int> m_curSolState;

		// the list of solutions, as point in Search Space
		std::vector<std::vector<unsigned int>> m_solStates;

		// list of solutions that were saved, should be kept sorted low to high duration
		std::priority_queue<ShuffleSolution*, std::vector<ShuffleSolution*>, std::function<bool(ShuffleSolution*, ShuffleSolution*)>> m_solutionsList;

		// may have unitialized solution and problemRep members
		Solution m_best;


		class SolutionConstructor : public Solution
		{
			friend class ShuffleSolution;
		public:
			// construct a generic Solution from the internal representation
			SolutionConstructor(const ShuffleSolution& solIntern, const ShuffleStep& optimizer);
		};

	};

}
