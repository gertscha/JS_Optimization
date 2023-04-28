#pragma once

#include "Optimizer.h"
#include "Solution.h"

#include <string>
#include <vector>
#include <random>


namespace JSOptimzer {

	class Problem;

	class ShuffleStep : public Optimizer
	{
	public:

		ShuffleStep(Problem* problem, Optimizer::TerminationCriteria terminationCriteria, unsigned int seed, std::string namePrefix);

		~ShuffleStep();

		const Solution& runOptimizer();

		// initializes an optimization run
		virtual void initialize() const;

		// performs an optimization iteration
		virtual void iterate() const;

		// returns true if termination criteria reached
		virtual bool checkTermination() const;


		// best solution knwon
		virtual const Solution& getBestSolution() const { return *m_best; }

	private:
		//std::mt19937 rnggen(seed); // Standard mersenne_twister_engine
		//std::uniform_int_distribution<> distrib(1, 6);
		struct machineStep {
			unsigned int it;
			unsigned int taskId;
			unsigned int stepIndex;
		};

		// internal solution representation for this optimizer
		class ShuffleSolution
		{
		public:
			ShuffleSolution(const Problem& p, const std::vector<std::vector<machineStep*>>& solState);

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

		// list (c style array) for each machine, tasks grouped by task ids for pointer based lookup
		std::vector<machineStep*> m_stepLists;

		// Step Sets of Tasks per machine to copy and use in init across resets
		std::vector<std::vector<machineStep*>> m_masterStepSets;

		// the current solution state, might make sense to encapsulate in a class
		std::vector<std::vector<machineStep*>> m_solState;

		// list of solutions that were saved
		std::vector<ShuffleSolution> m_foundSolutionsList;

		// may have unitialized solution and problemRep members
		Solution* m_best;

	};

}
