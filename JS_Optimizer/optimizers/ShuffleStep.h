#pragma once

#include "Optimizer.h"
#include "Solution.h"

#include <string>
#include <vector>
#include <random>
#include <functional>

namespace JSOptimzer {

	class Problem;

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

		class SolutionConstructor;

		struct StepIdentifier {

			StepIdentifier(unsigned int taskId, size_t stepIndex, unsigned int it, unsigned predTask)
				:taskId(taskId), stepIndex(stepIndex), it(it), predT(predTask) {}

			unsigned int taskId;
			size_t stepIndex;
			unsigned int machine;
			unsigned int it; // iterations


			bool operator< (const StepIdentifier& rhs) { return (this->taskId <= rhs.taskId) && (this->stepIndex < rhs.stepIndex); }
		};

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
			};

			ShuffleSolution(const std::vector<std::vector<size_t>>& solState,
							const Problem& problem, const ShuffleStep& optimizer);

			long getFitness() const { return m_completetionTime; }
			const std::vector<std::vector<ShuffleSolStep>>& getSolSteps() { return m_shuffelSol; }

		private:
			// rows correspond to machines, columns to steps, in order
			std::vector<std::vector<ShuffleSolStep>> m_shuffelSol;
			
			// the fitness value of this solution
			long m_completetionTime;
		};


		int m_temperature;
		size_t m_machineCnt;
		unsigned int m_totalIterations;

		std::mt19937 m_generator;
		unsigned int m_seed;
		std::string m_prefix;

		// the two internal problem representations, contain the same objects
		std::vector<std::vector<StepIdentifier>> m_taskStepView;
		std::vector<std::vector<std::reference_wrapper<StepIdentifier>>> m_machineStepView;

		// Step Bags of Tasks per machine to copy and use in init across resets
		// if there are multiple steps by the same task, inidices go to the lowest index
		// m_machineStepLists is then used to step through them and maintain the precedence order
		std::vector<std::vector<size_t>> m_masterStepBags;

		// the current solution state first initialized by initialize()
		std::vector<std::vector<size_t>> m_solState;

		// the current solution
		ShuffleSolution* m_current;

		// list of solutions that were saved, should not overlap with current
		std::vector<ShuffleSolution*> m_foundSolutionsList;

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
