#pragma once

#include "Optimizer.h"
#include "Solution.h"

#include <string>
#include <vector>
#include <random>
#include <span>
#include <iterator>
#include <tuple>


namespace JSOptimzer {

	class Problem;

	class ShuffleStep : public Optimizer
	{
	public:

		ShuffleStep(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix);

		~ShuffleStep();

		// run until TerminationCriteria reached, returns best solution
		const Solution& runOptimizer();

		virtual const Solution& getBestSolution() { return m_best; }
		
	private:
		// initializes an optimization run
		virtual void initialize();

		// performs an optimization iteration
		virtual void iterate();

		// returns true if termination criteria reached
		virtual bool checkTermination();

	private:

		struct StepIdentifier {
			unsigned int it; // iterations
			unsigned int taskId;
			unsigned int stepIndex;
		};

		// internal solution representation for this optimizer
		class ShuffleSolution
		{
		public:
			ShuffleSolution(const Problem& p, const std::vector<std::vector<StepIdentifier*>>& solState);

			long getFitness();

			struct ShuffleSolStep {
				unsigned int taskId;
				unsigned int stepIndex;
				unsigned int duration;
				unsigned int endTime;
			};
		private:
			// rows correspond to machines, columns to steps, in order
			std::vector<std::vector<ShuffleSolStep*>> m_shuffelSol;
			
			// the fitness value of this solution
			long m_completetionTime;
		};

		class SolutionConstructor : public Solution
		{
		public:
			// construct a generic Solution from the internal representation
			SolutionConstructor(const ShuffleSolution& sol);
		};

		int m_temperature;

		std::mt19937 m_generator;
		unsigned int m_seed;
		std::string m_prefix;

		// list for each machine, tasks grouped by task ids for lookup
		std::vector<std::span<StepIdentifier>> m_machineStepLists;

		// Step Sets of Tasks per machine to copy and use in init across resets
		// if there are multiple steps by the same task, all pointers point to the lowest index
		// m_machineStepLists is then used to step through them and maintain the precedence order
		std::vector<std::vector<StepIdentifier*>> m_masterStepSets;

		// the current solution state, might make sense to encapsulate in a class
		std::vector<std::vector<StepIdentifier*>> m_solState;

		// list of solutions that were saved
		std::vector<ShuffleSolution> m_foundSolutionsList;

		// may have unitialized solution and problemRep members
		Solution m_best;

	};

}
