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

		class SolutionConstructor : public Solution
		{
		public:
			SolutionConstructor(std::string name, unsigned int taskCnt, unsigned int machineCnt);

		};

		struct machineStep {
			unsigned int it;
			unsigned int taskId;
			unsigned int stepIndex;
		};

		struct machineStepElement {

		};

		int m_temperature;

		std::mt19937 m_generator;
		unsigned int m_seed;
		std::string m_prefix;

		// list for each machine, grouped by task ids for lookup
		std::vector<std::vector<machineStep>> m_stepLists;

		// Step Sets to copy and use in init across resets
		std::vector<std::vector<machineStepElement>> m_masterStepSets;

		// the current solution state, might make sense to encapsulate in a class
		std::vector<std::vector<machineStepElement>> m_solState;

		// list of solutions that were saved
		std::vector<Solution*> m_foundSolutionsList;

		// may have unitialized solution and problemRep members
		Solution*  m_best;

	};

}
