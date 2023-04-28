#pragma once

#include <string>
#include <vector>



namespace JSOptimzer {
	class Task;

	/*
	* basic description of a Problem, essentially a list of Tasks
	* a specific Optimizer should use this class to construct its
	* internal representation of the problem
	*/
	class Problem {
	public:

		class Bounds
		{
			friend Problem;
		public:

			const int longestTaskId;
			const int longestMachineId;
			const long TaskLowerBound;
			const long MachineLowerBound;
			const long sequentialUpperBound;
			const std::vector<int>& getMachinelowerBounds() const { return machineBounds; }
			const Problem& getProblem() const { return (*problem); }
			long getLowerBound() const { return (TaskLowerBound > MachineLowerBound) ? TaskLowerBound : MachineLowerBound; }

		private:
			Problem* problem;
			std::vector<int> machineBounds;

			Bounds(int lTId, int lMId, long TlB, long MlB, long SuB, Problem* problem);
		};

		/*
		 * extract Problem description from file
		 * name is optional, takes filename by default
		 */
		Problem(const std::string& filepath, const std::string& filename, std::string problemName = "");

		/*
		* destructor
		*/
		~Problem();

		/*
		* Getters
		*/
		
		unsigned int getTaskCnt() const { return m_numTasks; }

		unsigned int getMachineCnt() const { return m_numMachines; }

		const std::vector<Task>& getTasks() const { return m_tasks; }

		const std::vector<unsigned int>& getTaskStepCounts() const { return m_taskStepCounts; }

		const Problem::Bounds& getBounds() const { return *lowerBound; }

		const std::string& getName() const { return m_name; }

	

		friend std::ostream& operator<<(std::ostream& os, const Problem& dt);

	private:
		unsigned int m_numTasks;
		unsigned int m_numMachines;

		std::vector<Task> m_tasks;
		std::vector<unsigned int> m_taskStepCounts;
		Problem::Bounds* lowerBound;
		std::string m_name;

		// used in constructor
		void parseFileAndInitTaskVec(std::ifstream& file);

	};

}
